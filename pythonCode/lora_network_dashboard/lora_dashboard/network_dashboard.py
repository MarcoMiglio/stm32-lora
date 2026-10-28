from __future__ import annotations

import csv
import json
import queue
import sys
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Set, Tuple, Union

import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import serial
import serial.tools.list_ports

# =============================================================================
# PROTOCOL CONFIGURATION - copied from the original notebook
# =============================================================================
BAUDRATE = 115200

SYNC_WORD_BYTES = 0
MASK_BYTES = 1
NODE_ID_BYTES = 1
PKT_ID_BYTES = 2
RSSI_BYTES = 2
SENSOR_PLD_BYTES = 36

SYNC_WORD_POS = 0
MASK_POS = SYNC_WORD_POS + SYNC_WORD_BYTES
ALARM_BIT_POS = 0
ACK_BIT_POS = 1
RETX_BIT_POS = 2

MASK_ALARM_BIT = 1 << ALARM_BIT_POS
MASK_ALARM_ACK = 1 << ACK_BIT_POS
MASK_RETX_BIT = 1 << RETX_BIT_POS

NODE_ID_POS = MASK_POS + MASK_BYTES
PKT_ID_MSB_POS = NODE_ID_POS + NODE_ID_BYTES
PKT_ID_LSB_POS = NODE_ID_POS + NODE_ID_BYTES + 1
SMPL_DATA_POS = PKT_ID_MSB_POS + PKT_ID_BYTES
RSSI_POS = SMPL_DATA_POS + SENSOR_PLD_BYTES
BC_ID1_POS = RSSI_POS + RSSI_BYTES

SYNC_WORD_ENV = b"\xAA\x55"
ENV_NODE_PYL_SIZE = 42
SYNC_WORD_BC = b"\x11\xAA"

# Localization V1: wait briefly for multiple copies of same ENV_ID + PKT_ID.
LOCALIZATION_COLLECTION_MS = 350
LOCALIZATION_HISTORY_TTL_S = 5.0

# Fixed colors by node type.
# Last-seen age is intentionally NOT encoded by color because expected
# transmission intervals depend on firmware operating mode (NORMAL/ALARM/etc.).
ROOT_COLOR = "#3B82F6"
BC_COLOR   = "#22A06B"
ENV_COLOR  = "#F59E0B"
ENV_ALARM_COLOR = "#EF4444"

Receiver = Union[str, int]  # "ROOT" or BC node ID

# Runtime directory.
# - When launched as a normal .py file: folder containing this source file.
# - When packaged by PyInstaller: folder containing LoRaDashboard.exe.
#
# Keeping writable files next to the executable makes the onedir build portable:
# the whole LoRaDashboard folder can be copied to another Windows PC.
if getattr(sys, "frozen", False):
    APP_DIR = Path(sys.executable).resolve().parent
else:
    APP_DIR = Path(__file__).resolve().parent

SETTINGS_PATH = APP_DIR / "dashboard_settings.json"
DEFAULT_LOG_DIR = APP_DIR / "logs"


@dataclass
class Packet:
    timestamp: datetime
    reception: str  # DIRECT / FORWARDED
    mask: int
    env_id: int
    pkt_id: int
    payload: bytes
    rssi: int
    bc_sequence: List[int] = field(default_factory=list)
    link_rssi: Optional[int] = None

    @property
    def alarm(self) -> bool:
        return bool(self.mask & MASK_ALARM_BIT)

    @property
    def alarm_ack(self) -> bool:
        return bool(self.mask & MASK_ALARM_ACK)

    @property
    def retransmission(self) -> bool:
        return bool(self.mask & MASK_RETX_BIT)

    @property
    def first_receiver(self) -> Receiver:
        if self.reception == "DIRECT" or not self.bc_sequence:
            return "ROOT"
        return self.bc_sequence[0]

    @property
    def csv_type(self) -> str:
        return "ENV" if self.reception == "DIRECT" else "BC"


@dataclass
class NodeInfo:
    node_type: str
    node_id: Union[str, int]
    first_seen: datetime
    last_seen: datetime


@dataclass
class EnvAlarmState:
    alarm_active: bool = False
    alarm_pkt_id: Optional[int] = None
    latest_pkt_id: Optional[int] = None


@dataclass
class LocationEstimate:
    env_id: int
    pkt_id: int
    logical_position: float
    receivers: Tuple[Receiver, ...]
    method: str = "first_receiver_midpoint"


# =============================================================================
# LOCALIZATION ENGINE
# -----------------------------------------------------------------------------
# THIS IS THE MODULE TO REPLACE/EXTEND IN THE FUTURE.
#
# Inputs:
#   env_id      : ENV that generated the packet
#   pkt_id      : packet ID used to group copies of same transmission
#   receivers   : nodes that directly heard the ENV transmission.
#                 Each item is "ROOT" or the FIRST BC in a BC sequence.
#   bc_order    : current graphical BC order from ROOT outwards
#
# Output:
#   LocationEstimate.logical_position
#       0.0 = ROOT
#       1.0 = first BC on map
#       2.0 = second BC on map
#       1.5 = halfway between first and second BC
#
# V1 algorithm:
#   - one receiver  -> ENV on that receiver
#   - >1 receivers -> ENV at midpoint between extreme receivers
#   - RSSI is acquired and stored, but intentionally NOT used here.
# =============================================================================
class LocalizationEngine:
    @staticmethod
    def localize(env_id: int, pkt_id: int, receivers: Set[Receiver], bc_order: List[int]) -> Optional[LocationEstimate]:
        if not receivers:
            return None

        positions: Dict[Receiver, float] = {"ROOT": 0.0}
        for idx, bc_id in enumerate(bc_order, start=1):
            positions[bc_id] = float(idx)

        valid = [r for r in receivers if r in positions]
        if not valid:
            return None

        vals = [positions[r] for r in valid]
        logical_position = (min(vals) + max(vals)) / 2.0
        ordered = tuple(sorted(valid, key=lambda r: positions[r]))
        return LocationEstimate(env_id, pkt_id, logical_position, ordered)


class NetworkState:
    def __init__(self) -> None:
        self.nodes: Dict[Tuple[str, Union[str, int]], NodeInfo] = {}
        self.bc_order: List[int] = []
        self.env_locations: Dict[int, LocationEstimate] = {}
        self.env_alarm_states: Dict[int, EnvAlarmState] = {}

    @staticmethod
    def _pkt_is_newer(new_pkt_id: int, reference_pkt_id: int) -> bool:
        """Return True if new_pkt_id is newer in a uint16 sequence space."""
        delta = (new_pkt_id - reference_pkt_id) & 0xFFFF
        return 0 < delta < 0x8000

    def _update_env_alarm_state(self, packet: Packet) -> None:
        state = self.env_alarm_states.setdefault(packet.env_id, EnvAlarmState())

        # First packet ever seen for this ENV.
        if state.latest_pkt_id is None:
            state.latest_pkt_id = packet.pkt_id
            if packet.alarm:
                state.alarm_active = True
                state.alarm_pkt_id = packet.pkt_id
            return

        same_pkt = packet.pkt_id == state.latest_pkt_id
        newer_pkt = self._pkt_is_newer(packet.pkt_id, state.latest_pkt_id)

        # Ignore delayed packets older than the latest logical ENV state.
        if not same_pkt and not newer_pkt:
            return

        if newer_pkt:
            state.latest_pkt_id = packet.pkt_id

        if packet.alarm:
            # Any current/newer ALARM packet activates the visual alarm state.
            state.alarm_active = True
            state.alarm_pkt_id = packet.pkt_id
            return

        # A normal packet clears an alarm only when its PKT_ID is strictly newer
        # than the packet that activated the alarm. Copies of the same PKT_ID do
        # not clear it.
        if (
            state.alarm_active
            and state.alarm_pkt_id is not None
            and self._pkt_is_newer(packet.pkt_id, state.alarm_pkt_id)
        ):
            state.alarm_active = False
            state.alarm_pkt_id = None

    def _touch(self, node_type: str, node_id: Union[str, int], ts: datetime) -> None:
        key = (node_type, node_id)
        if key not in self.nodes:
            self.nodes[key] = NodeInfo(node_type, node_id, ts, ts)
        else:
            self.nodes[key].last_seen = ts

    def update_from_packet(self, packet: Packet) -> None:
        ts = packet.timestamp
        self._touch("ROOT", "ROOT", ts)
        self._touch("ENV", packet.env_id, ts)
        self._update_env_alarm_state(packet)
        for bc_id in packet.bc_sequence:
            self._touch("BC", bc_id, ts)
            if bc_id not in self.bc_order:
                self.bc_order.append(bc_id)

    def set_location(self, estimate: LocationEstimate) -> None:
        self.env_locations[estimate.env_id] = estimate

    def last_seen(self, node_type: str, node_id: Union[str, int]) -> Optional[datetime]:
        info = self.nodes.get((node_type, node_id))
        return info.last_seen if info else None

    def env_alarm_active(self, env_id: int) -> bool:
        state = self.env_alarm_states.get(env_id)
        return bool(state and state.alarm_active)


class CsvRecorder:
    HEADER = ["Timestamp", "Type", "Mask", "NodeID", "PktID", "Payload", "RSSI", "BC_Seq"]

    def __init__(self) -> None:
        self.file = None
        self.writer = None
        self.path: Optional[Path] = None
        self.rows_written = 0

    @property
    def is_recording(self) -> bool:
        return self.file is not None

    def start(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        exists = path.exists() and path.stat().st_size > 0
        self.file = path.open("a", newline="", encoding="utf-8")
        self.writer = csv.writer(self.file)
        self.path = path
        self.rows_written = 0
        if not exists:
            self.writer.writerow(self.HEADER)
            self.file.flush()

    def write(self, packet: Packet) -> None:
        if not self.is_recording or self.writer is None or self.file is None:
            return
        bc_hex = bytes(packet.bc_sequence).hex() if packet.bc_sequence else ""
        self.writer.writerow([
            packet.timestamp.strftime("%Y-%m-%d %H:%M:%S"),
            packet.csv_type,
            packet.mask,
            packet.env_id,
            packet.pkt_id,
            packet.payload.hex(),
            packet.rssi,
            bc_hex,
        ])
        self.file.flush()
        self.rows_written += 1

    def stop(self) -> None:
        if self.file is not None:
            self.file.flush()
            self.file.close()
        self.file = None
        self.writer = None
        self.path = None


class SerialWorker(threading.Thread):
    def __init__(self, port: str, out_queue: queue.Queue, baudrate: int = BAUDRATE) -> None:
        super().__init__(daemon=True)
        self.port = port
        self.baudrate = baudrate
        self.out_queue = out_queue
        self.stop_event = threading.Event()
        self.ser: Optional[serial.Serial] = None

    def stop(self) -> None:
        self.stop_event.set()
        if self.ser is not None:
            try:
                self.ser.cancel_read()
            except Exception:
                pass

    def _read_exact(self, n: int, timeout_s: float = 1.0) -> Optional[bytes]:
        if self.ser is None:
            return None
        out = bytearray()
        deadline = time.monotonic() + timeout_s
        while not self.stop_event.is_set() and len(out) < n:
            chunk = self.ser.read(n - len(out))
            if chunk:
                out.extend(chunk)
            elif time.monotonic() >= deadline:
                return None
        return bytes(out) if len(out) == n else None

    def run(self) -> None:
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            self.out_queue.put(("opened", self.port))

            while not self.stop_event.is_set():
                first = self.ser.read(1)
                if not first:
                    continue

                if first == SYNC_WORD_ENV[:1]:
                    second = self._read_exact(1)
                    if second == SYNC_WORD_ENV[1:2]:
                        payload = self._read_exact(ENV_NODE_PYL_SIZE)
                        if payload is None:
                            continue
                        self.out_queue.put(("packet", self._parse_env(payload)))
                        continue

                if first == SYNC_WORD_BC[:1]:
                    second = self._read_exact(1)
                    if second == SYNC_WORD_BC[1:2]:
                        payload = self._read_exact(BC_ID1_POS)
                        if payload is None:
                            continue
                        num_bytes = self._read_exact(1)
                        if num_bytes is None:
                            continue
                        bcs_num = num_bytes[0]
                        bc_seq = self._read_exact(bcs_num)
                        last_rssi = self._read_exact(RSSI_BYTES)
                        if bc_seq is None or last_rssi is None:
                            continue
                        self.out_queue.put(("packet", self._parse_bc(payload, bc_seq, last_rssi)))
                        continue

        except Exception as exc:
            if not self.stop_event.is_set():
                self.out_queue.put(("error", str(exc)))
        finally:
            if self.ser is not None:
                try:
                    self.ser.close()
                except Exception:
                    pass
            self.ser = None
            self.out_queue.put(("closed", None))

    @staticmethod
    def _parse_env(payload: bytes) -> Packet:
        mask = payload[MASK_POS]
        env_id = payload[NODE_ID_POS]
        pkt_id = (payload[PKT_ID_MSB_POS] << 8) | payload[PKT_ID_LSB_POS]
        sample = payload[SMPL_DATA_POS:RSSI_POS]
        rssi = int.from_bytes(payload[RSSI_POS:RSSI_POS + 2], "big", signed=True)
        return Packet(datetime.now(), "DIRECT", mask, env_id, pkt_id, sample, rssi, [], rssi)

    @staticmethod
    def _parse_bc(payload: bytes, bc_seq: bytes, last_rssi_bytes: bytes) -> Packet:
        mask = payload[MASK_POS]
        env_id = payload[NODE_ID_POS]
        pkt_id = (payload[PKT_ID_MSB_POS] << 8) | payload[PKT_ID_LSB_POS]
        sample = payload[SMPL_DATA_POS:RSSI_POS]
        rssi = int.from_bytes(payload[RSSI_POS:RSSI_POS + 2], "big", signed=True)
        link_rssi = int.from_bytes(last_rssi_bytes, "big", signed=True)
        return Packet(datetime.now(), "FORWARDED", mask, env_id, pkt_id, sample, rssi, list(bc_seq), link_rssi)


class Dashboard(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("LoRa Multi-Hop Network Dashboard")
        self.geometry("1480x880")
        self.minsize(1100, 700)

        self.network_state = NetworkState()
        self.csv_recorder = CsvRecorder()
        self.packets: List[Packet] = []
        self.event_queue: queue.Queue = queue.Queue()
        self.serial_worker: Optional[SerialWorker] = None
        self.localization_pending: Dict[Tuple[int, int], Dict[str, object]] = {}
        self.map_hit_regions: List[Tuple[float, float, float, float, str, Union[str, int]]] = []

        self.settings = self._load_settings()
        self._build_ui()
        self._refresh_ports()
        self._restore_settings()

        self.after(50, self._poll_worker_events)
        self.after(500, self._periodic_refresh)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    # ---------------------------------------------------------------- UI
    def _build_ui(self) -> None:
        self.columnconfigure(0, weight=1)
        self.rowconfigure(2, weight=1)

        serial_frame = ttk.LabelFrame(self, text="Serial connection", padding=8)
        serial_frame.grid(row=0, column=0, sticky="ew", padx=10, pady=(10, 5))
        serial_frame.columnconfigure(1, weight=1)

        ttk.Label(serial_frame, text="Port:").grid(row=0, column=0, padx=(0, 5))
        self.port_combo = ttk.Combobox(serial_frame, state="readonly", width=55)
        self.port_combo.grid(row=0, column=1, sticky="ew")
        ttk.Button(serial_frame, text="Refresh", command=self._refresh_ports).grid(row=0, column=2, padx=5)
        self.connect_btn = ttk.Button(serial_frame, text="Connect", command=self._toggle_connection)
        self.connect_btn.grid(row=0, column=3, padx=5)
        self.connection_status = ttk.Label(serial_frame, text="Not connected")
        self.connection_status.grid(row=0, column=4, padx=(12, 0))

        rec_frame = ttk.LabelFrame(self, text="CSV recording", padding=8)
        rec_frame.grid(row=1, column=0, sticky="ew", padx=10, pady=5)
        rec_frame.columnconfigure(1, weight=1)

        ttk.Label(rec_frame, text="Folder:").grid(row=0, column=0, sticky="w")
        self.folder_var = tk.StringVar()
        self.folder_entry = ttk.Entry(rec_frame, textvariable=self.folder_var)
        self.folder_entry.grid(row=0, column=1, sticky="ew", padx=5)
        ttk.Button(rec_frame, text="Browse...", command=self._choose_folder).grid(row=0, column=2)

        ttk.Label(rec_frame, text="Filename:").grid(row=1, column=0, sticky="w")
        self.filename_var = tk.StringVar()
        self.filename_entry = ttk.Entry(rec_frame, textvariable=self.filename_var)
        self.filename_entry.grid(row=1, column=1, sticky="ew", padx=5, pady=(5, 0))
        self.record_btn = ttk.Button(rec_frame, text="Start recording", command=self._toggle_recording)
        self.record_btn.grid(row=1, column=2, pady=(5, 0))
        self.record_status = ttk.Label(rec_frame, text="Not recording")
        self.record_status.grid(row=2, column=1, columnspan=2, sticky="w", padx=5, pady=(4, 0))

        paned = ttk.Panedwindow(self, orient=tk.HORIZONTAL)
        paned.grid(row=2, column=0, sticky="nsew", padx=10, pady=5)

        map_frame = ttk.LabelFrame(paned, text="Network map", padding=6)
        log_frame = ttk.LabelFrame(paned, text="Live log", padding=6)
        paned.add(map_frame, weight=1)
        paned.add(log_frame, weight=1)

        map_frame.rowconfigure(0, weight=1)
        map_frame.columnconfigure(0, weight=1)
        self.map_canvas = tk.Canvas(map_frame, background="white", highlightthickness=1, highlightbackground="#aaaaaa")
        self.map_canvas.grid(row=0, column=0, sticky="nsew")
        self.map_canvas.bind("<Button-1>", self._on_map_click)
        self.map_canvas.bind("<Configure>", lambda e: self._draw_map())
        ttk.Label(
            map_frame,
            text="Localization V1: first receiver only; multiple receivers -> midpoint. RSSI is stored but not used.",
            wraplength=550,
        ).grid(row=1, column=0, sticky="ew", pady=(6, 0))

        # Filters
        filter_frame = ttk.Frame(log_frame)
        filter_frame.pack(fill="x")
        for c in range(8):
            filter_frame.columnconfigure(c, weight=1 if c in (1, 3, 5, 7) else 0)

        self.env_var = tk.StringVar(value="Any ENV")
        self.env_combo = ttk.Combobox(filter_frame, textvariable=self.env_var, state="readonly", values=["Any ENV"], width=12)
        self.bc_var = tk.StringVar(value="Any BC")
        self.bc_combo = ttk.Combobox(filter_frame, textvariable=self.bc_var, state="readonly", values=["Any BC"], width=12)
        self.reception_var = tk.StringVar(value="Any reception")
        self.reception_combo = ttk.Combobox(filter_frame, textvariable=self.reception_var, state="readonly", values=["Any reception", "Direct", "Forwarded"], width=14)
        self.pkt_var = tk.StringVar()

        ttk.Label(filter_frame, text="ENV:").grid(row=0, column=0, sticky="w")
        self.env_combo.grid(row=0, column=1, sticky="ew", padx=(0, 5))
        ttk.Label(filter_frame, text="BC contains:").grid(row=0, column=2, sticky="w")
        self.bc_combo.grid(row=0, column=3, sticky="ew", padx=(0, 5))
        ttk.Label(filter_frame, text="Reception:").grid(row=0, column=4, sticky="w")
        self.reception_combo.grid(row=0, column=5, sticky="ew", padx=(0, 5))
        ttk.Label(filter_frame, text="PKT_ID:").grid(row=0, column=6, sticky="w")
        ttk.Entry(filter_frame, textvariable=self.pkt_var, width=10).grid(row=0, column=7, sticky="ew")

        self.alarm_var = tk.BooleanVar()
        self.ack_var = tk.BooleanVar()
        self.retx_var = tk.BooleanVar()
        self.time_enabled_var = tk.BooleanVar()
        self.from_var = tk.StringVar(value=(datetime.now().replace(microsecond=0)).strftime("%Y-%m-%d %H:%M:%S"))
        self.to_var = tk.StringVar(value=(datetime.now().replace(microsecond=0)).strftime("%Y-%m-%d %H:%M:%S"))

        ttk.Checkbutton(filter_frame, text="Alarm", variable=self.alarm_var, command=self._render_log).grid(row=1, column=0, sticky="w")
        ttk.Checkbutton(filter_frame, text="Alarm ACK", variable=self.ack_var, command=self._render_log).grid(row=1, column=1, sticky="w")
        ttk.Checkbutton(filter_frame, text="RETX", variable=self.retx_var, command=self._render_log).grid(row=1, column=2, sticky="w")
        ttk.Checkbutton(filter_frame, text="Time range", variable=self.time_enabled_var, command=self._render_log).grid(row=1, column=3, sticky="w")
        ttk.Label(filter_frame, text="From:").grid(row=1, column=4, sticky="w")
        ttk.Entry(filter_frame, textvariable=self.from_var, width=19).grid(row=1, column=5, sticky="ew", padx=(0, 5))
        ttk.Label(filter_frame, text="To:").grid(row=1, column=6, sticky="w")
        ttk.Entry(filter_frame, textvariable=self.to_var, width=19).grid(row=1, column=7, sticky="ew")

        self.auto_scroll_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(filter_frame, text="Auto-scroll", variable=self.auto_scroll_var).grid(row=2, column=0, sticky="w", pady=(4, 0))
        ttk.Button(filter_frame, text="Clear filters", command=self._clear_filters).grid(row=2, column=1, sticky="w", pady=(4, 0))
        self.filter_count = ttk.Label(filter_frame, text="0 / 0 packets")
        self.filter_count.grid(row=2, column=2, columnspan=6, sticky="w", padx=8, pady=(4, 0))

        # Binding changes for cumulative filtering
        self.env_combo.bind("<<ComboboxSelected>>", lambda e: self._render_log())
        self.bc_combo.bind("<<ComboboxSelected>>", lambda e: self._render_log())
        self.reception_combo.bind("<<ComboboxSelected>>", lambda e: self._render_log())
        for var in (self.pkt_var, self.from_var, self.to_var):
            var.trace_add("write", lambda *_: self._render_log())

        text_frame = ttk.Frame(log_frame)
        text_frame.pack(fill="both", expand=True, pady=(6, 0))
        text_frame.rowconfigure(0, weight=1)
        text_frame.columnconfigure(0, weight=1)
        self.log_text = tk.Text(text_frame, wrap="none", font=("Consolas", 9), state="disabled")
        ybar = ttk.Scrollbar(text_frame, orient="vertical", command=self.log_text.yview)
        xbar = ttk.Scrollbar(text_frame, orient="horizontal", command=self.log_text.xview)
        self.log_text.configure(yscrollcommand=ybar.set, xscrollcommand=xbar.set)
        self.log_text.grid(row=0, column=0, sticky="nsew")
        ybar.grid(row=0, column=1, sticky="ns")
        xbar.grid(row=1, column=0, sticky="ew")

        self.status_var = tk.StringVar(value="DISCONNECTED | packets received: 0 | NOT RECORDING")
        ttk.Label(self, textvariable=self.status_var, relief="sunken", anchor="w").grid(row=3, column=0, sticky="ew", padx=10, pady=(0, 10))

    # ----------------------------------------------------------- Settings
    def _load_settings(self) -> dict:
        try:
            if SETTINGS_PATH.exists():
                return json.loads(SETTINGS_PATH.read_text(encoding="utf-8"))
        except Exception:
            pass
        return {}

    def _restore_settings(self) -> None:
        self.folder_var.set(self.settings.get("log_folder", str(DEFAULT_LOG_DIR)))
        self.filename_var.set(self._default_filename())

    def _save_settings(self) -> None:
        data = {"log_folder": self.folder_var.get().strip()}
        try:
            SETTINGS_PATH.write_text(json.dumps(data, indent=2), encoding="utf-8")
        except Exception:
            pass

    @staticmethod
    def _default_filename() -> str:
        return datetime.now().strftime("network_log_%Y-%m-%d_%H-%M-%S.csv")

    # ------------------------------------------------------------- Serial
    def _refresh_ports(self) -> None:
        current = self.port_combo.get()
        self.port_map: Dict[str, str] = {}
        labels = []
        for p in serial.tools.list_ports.comports():
            label = f"{p.device}  -  {p.description}"
            labels.append(label)
            self.port_map[label] = p.device
        if not labels:
            labels = ["No serial ports found"]
        self.port_combo["values"] = labels
        if current in labels:
            self.port_combo.set(current)
        else:
            self.port_combo.current(0)

    def _toggle_connection(self) -> None:
        if self.serial_worker and self.serial_worker.is_alive():
            self.connection_status.config(text="Disconnecting...")
            self.serial_worker.stop()
            return

        label = self.port_combo.get()
        port = self.port_map.get(label)
        if not port:
            messagebox.showwarning("Serial", "Select a valid serial port first.")
            return

        self.serial_worker = SerialWorker(port, self.event_queue, BAUDRATE)
        self.connection_status.config(text=f"Connecting to {port}...")
        self.connect_btn.config(state="disabled")
        self.serial_worker.start()

    def _poll_worker_events(self) -> None:
        try:
            while True:
                kind, payload = self.event_queue.get_nowait()
                if kind == "opened":
                    self.connection_status.config(text=f"Connected: {payload} @ {BAUDRATE} bps")
                    self.connect_btn.config(text="Disconnect", state="normal")
                    self.port_combo.config(state="disabled")
                elif kind == "closed":
                    self.connection_status.config(text="Not connected")
                    self.connect_btn.config(text="Connect", state="normal")
                    self.port_combo.config(state="readonly")
                    self.serial_worker = None
                elif kind == "error":
                    messagebox.showerror("Serial error", str(payload))
                elif kind == "packet":
                    self._on_packet(payload)
        except queue.Empty:
            pass
        self.after(50, self._poll_worker_events)

    # ------------------------------------------------------------- Packets
    def _on_packet(self, packet: Packet) -> None:
        self.packets.append(packet)
        self.network_state.update_from_packet(packet)
        self.csv_recorder.write(packet)
        self._update_filter_choices(packet)
        self._update_localization(packet)

        if self._packet_passes_filters(packet):
            self._append_packet_to_log(packet)
        else:
            self._update_filter_count()
        self._draw_map()

    def _update_filter_choices(self, packet: Packet) -> None:
        env_label = f"ENV {packet.env_id}"
        vals = list(self.env_combo["values"])
        if env_label not in vals:
            vals.append(env_label)
            self.env_combo["values"] = vals
        vals = list(self.bc_combo["values"])
        for bc_id in packet.bc_sequence:
            label = f"BC {bc_id}"
            if label not in vals:
                vals.append(label)
        self.bc_combo["values"] = vals

    # --------------------------------------------------------- Localization
    def _update_localization(self, packet: Packet) -> None:
        key = (packet.env_id, packet.pkt_id)
        now = time.monotonic()
        for old_key in list(self.localization_pending):
            if now - float(self.localization_pending[old_key]["last_update"]) > LOCALIZATION_HISTORY_TTL_S:
                del self.localization_pending[old_key]

        entry = self.localization_pending.setdefault(key, {"receivers": set(), "last_update": now, "scheduled": False})
        receivers: Set[Receiver] = entry["receivers"]  # type: ignore[assignment]
        receivers.add(packet.first_receiver)
        entry["last_update"] = now
        if not entry["scheduled"]:
            entry["scheduled"] = True
            self.after(LOCALIZATION_COLLECTION_MS, lambda e=packet.env_id, p=packet.pkt_id: self._finalize_localization(e, p))

    def _finalize_localization(self, env_id: int, pkt_id: int) -> None:
        key = (env_id, pkt_id)
        entry = self.localization_pending.get(key)
        if not entry:
            return
        elapsed_ms = (time.monotonic() - float(entry["last_update"])) * 1000
        if elapsed_ms < LOCALIZATION_COLLECTION_MS - 5:
            self.after(max(10, int(LOCALIZATION_COLLECTION_MS - elapsed_ms)), lambda: self._finalize_localization(env_id, pkt_id))
            return
        entry["scheduled"] = False
        estimate = LocalizationEngine.localize(env_id, pkt_id, set(entry["receivers"]), self.network_state.bc_order)  # type: ignore[arg-type]
        if estimate:
            self.network_state.set_location(estimate)
            self._draw_map()

    # --------------------------------------------------------------- CSV
    def _choose_folder(self) -> None:
        start = self.folder_var.get().strip() or str(APP_DIR)
        folder = filedialog.askdirectory(initialdir=start, title="Select CSV folder")
        if folder:
            self.folder_var.set(folder)
            self._save_settings()

    def _toggle_recording(self) -> None:
        if self.csv_recorder.is_recording:
            self.csv_recorder.stop()
            self.record_btn.config(text="Start recording")
            self.record_status.config(text="Not recording")
            self.folder_entry.config(state="normal")
            self.filename_entry.config(state="normal")
            return

        folder = self.folder_var.get().strip()
        filename = self.filename_var.get().strip() or self._default_filename()
        if not folder:
            messagebox.showwarning("CSV", "Choose a folder first.")
            return
        if not filename.lower().endswith(".csv"):
            filename += ".csv"
            self.filename_var.set(filename)
        path = Path(folder).expanduser() / filename
        if path.exists() and path.stat().st_size > 0:
            if not messagebox.askyesno("Existing CSV", f"{path.name} already exists.\n\nAppend new packets to it?"):
                return
        try:
            self.csv_recorder.start(path)
        except Exception as exc:
            messagebox.showerror("CSV error", str(exc))
            return
        self._save_settings()
        self.record_btn.config(text="Stop recording")
        self.record_status.config(text=f"RECORDING -> {path}")
        self.folder_entry.config(state="disabled")
        self.filename_entry.config(state="disabled")

    # ------------------------------------------------------------ Filters
    def _clear_filters(self) -> None:
        self.env_var.set("Any ENV")
        self.bc_var.set("Any BC")
        self.reception_var.set("Any reception")
        self.pkt_var.set("")
        self.alarm_var.set(False)
        self.ack_var.set(False)
        self.retx_var.set(False)
        self.time_enabled_var.set(False)
        self._render_log()

    def _parse_time_field(self, value: str) -> Optional[datetime]:
        value = value.strip()
        if not value:
            return None
        for fmt in ("%Y-%m-%d %H:%M:%S", "%H:%M:%S"):
            try:
                dt = datetime.strptime(value, fmt)
                if fmt == "%H:%M:%S":
                    now = datetime.now()
                    dt = dt.replace(year=now.year, month=now.month, day=now.day)
                return dt
            except ValueError:
                pass
        return None

    def _packet_passes_filters(self, p: Packet) -> bool:
        env = self.env_var.get()
        if env != "Any ENV" and p.env_id != int(env.split()[-1]):
            return False
        bc = self.bc_var.get()
        if bc != "Any BC" and int(bc.split()[-1]) not in p.bc_sequence:
            return False
        rec = self.reception_var.get()
        if rec == "Direct" and p.reception != "DIRECT":
            return False
        if rec == "Forwarded" and p.reception != "FORWARDED":
            return False
        pkt = self.pkt_var.get().strip()
        if pkt:
            try:
                if p.pkt_id != int(pkt, 0):
                    return False
            except ValueError:
                return False
        if self.alarm_var.get() and not p.alarm:
            return False
        if self.ack_var.get() and not p.alarm_ack:
            return False
        if self.retx_var.get() and not p.retransmission:
            return False
        if self.time_enabled_var.get():
            f = self._parse_time_field(self.from_var.get())
            t = self._parse_time_field(self.to_var.get())
            if f is None or t is None or not (f <= p.timestamp <= t):
                return False
        return True

    def _render_log(self) -> None:
        if not hasattr(self, "log_text"):
            return
        blocks = [self._format_packet(p) for p in self.packets if self._packet_passes_filters(p)]
        self.log_text.config(state="normal")
        self.log_text.delete("1.0", "end")
        self.log_text.insert("end", "\n\n".join(blocks))
        self.log_text.config(state="disabled")
        if self.auto_scroll_var.get():
            self.log_text.see("end")
        self._update_filter_count()

    def _append_packet_to_log(self, packet: Packet) -> None:
        self.log_text.config(state="normal")
        if self.log_text.index("end-1c") != "1.0":
            self.log_text.insert("end", "\n\n")
        self.log_text.insert("end", self._format_packet(packet))
        self.log_text.config(state="disabled")
        if self.auto_scroll_var.get():
            self.log_text.see("end")
        self._update_filter_count()

    def _update_filter_count(self) -> None:
        shown = sum(1 for p in self.packets if self._packet_passes_filters(p))
        self.filter_count.config(text=f"{shown} / {len(self.packets)} packets")

    @staticmethod
    def _format_packet(p: Packet) -> str:
        stamp = p.timestamp.strftime("%H:%M:%S.%f")[:-3]
        payload_repr = repr(p.payload)
        if p.reception == "DIRECT":
            return "\n".join([
                f"[{stamp}] New ENV NODE PKT",
                f"MASK: {p.mask}",
                f"NODE_ID: {p.env_id}",
                f"PKT_ID: {p.pkt_id}",
                f"Payload: {payload_repr}",
                f"RSSI: {p.rssi}",
                f"link-RSSI: {p.link_rssi}",
            ])
        return "\n".join([
            f"[{stamp}] New BC NODE PKT",
            f"MASK: {p.mask} (ALARM: {int(p.alarm)} ALARM ACK: {int(p.alarm_ack)} RETX: {int(p.retransmission)})",
            f"NODE_ID: {p.env_id}",
            f"PKT_ID: {p.pkt_id}",
            f"Payload: {payload_repr}",
            f"RSSI: {p.rssi}",
            f"BC sequence: {bytes(p.bc_sequence)!r}",
            f"LINK RSSI: {p.link_rssi}",
        ])

    # ---------------------------------------------------------- Network map
    @staticmethod
    def _age_text(ts: Optional[datetime]) -> str:
        if ts is None:
            return "never"
        age = max(0, (datetime.now() - ts).total_seconds())
        if age < 1:
            return "<1 s ago"
        if age < 60:
            return f"{int(age)} s ago"
        if age < 3600:
            return f"{int(age // 60)} min ago"
        return f"{int(age // 3600)} h ago"

    def _draw_map(self) -> None:
        if not hasattr(self, "map_canvas"):
            return
        c = self.map_canvas
        c.delete("all")
        w = max(200, c.winfo_width())
        h = max(300, c.winfo_height())
        margin = 70
        base_y = int(h * 0.58)
        count = max(1, len(self.network_state.bc_order))
        spacing = max(80, (w - 2 * margin) / count)

        def xpos(pos: float) -> float:
            return margin + pos * spacing

        x0 = xpos(0)
        x1 = xpos(float(len(self.network_state.bc_order))) if self.network_state.bc_order else x0 + 100
        c.create_line(x0, base_y, x1, base_y, width=2)
        self.map_hit_regions = []

        chain = [("ROOT", "ROOT", 0.0)] + [("BC", bid, float(i)) for i, bid in enumerate(self.network_state.bc_order, 1)]
        for node_type, node_id, pos in chain:
            x = xpos(pos)
            ts = self.network_state.last_seen(node_type, node_id)
            r = 16

            # Shape + color identify node TYPE only.
            # ROOT = blue square, BC = green circle.
            if node_type == "ROOT":
                c.create_rectangle(
                    x-r, base_y-r, x+r, base_y+r,
                    fill=ROOT_COLOR, outline="#222222", width=2
                )
            else:
                c.create_oval(
                    x-r, base_y-r, x+r, base_y+r,
                    fill=BC_COLOR, outline="#222222", width=2
                )

            title = "ROOT" if node_type == "ROOT" else f"BC {node_id}"
            c.create_text(x, base_y+30, text=title, font=("Segoe UI", 10, "bold"))
            c.create_text(x, base_y+48, text=self._age_text(ts), font=("Segoe UI", 9))
            self.map_hit_regions.append((x-30, base_y-30, x+30, base_y+70, node_type, node_id))

        buckets: Dict[float, List[int]] = {}
        for env_id, loc in self.network_state.env_locations.items():
            key = round(loc.logical_position * 2) / 2
            buckets.setdefault(key, []).append(env_id)
        for pos, envs in sorted(buckets.items()):
            for index, env_id in enumerate(sorted(envs)):
                x = xpos(pos)
                direction = -1 if index % 2 == 0 else 1
                level = index // 2
                y = base_y + direction * (82 + level * 70)
                ts = self.network_state.last_seen("ENV", env_id)
                c.create_line(x, base_y, x, y, dash=(4, 3))
                r = 15

                # ENV = orange diamond in normal state.
                # During ALARM it blinks red/orange. The alarm state is cleared
                # only by a newer non-ALARM packet from the same ENV.
                alarm_active = self.network_state.env_alarm_active(env_id)
                blink_on = (int(time.monotonic() * 2) % 2) == 0
                env_fill = ENV_ALARM_COLOR if alarm_active and blink_on else ENV_COLOR
                env_outline = "#991B1B" if alarm_active else "#222222"
                env_width = 3 if alarm_active else 2
                c.create_polygon(
                    x, y-r,
                    x+r, y,
                    x, y+r,
                    x-r, y,
                    fill=env_fill, outline=env_outline, width=env_width
                )

                label_y = y-34 if direction < 0 else y+34
                c.create_text(x, label_y, text=f"ENV {env_id}", font=("Segoe UI", 10, "bold"))
                c.create_text(x, label_y + (16 if direction > 0 else -16), text=self._age_text(ts), font=("Segoe UI", 9))
                self.map_hit_regions.append((x-35, y-35, x+35, y+35, "ENV", env_id))

        # Compact legend. Colors are fixed by node type; age never changes them.
        legend_y = 14
        c.create_rectangle(12, legend_y-7, 24, legend_y+5, fill=ROOT_COLOR, outline="#222222")
        c.create_text(30, legend_y, anchor="w", text="ROOT", font=("Segoe UI", 9))
        c.create_oval(82, legend_y-7, 94, legend_y+5, fill=BC_COLOR, outline="#222222")
        c.create_text(100, legend_y, anchor="w", text="BC", font=("Segoe UI", 9))
        c.create_polygon(143, legend_y-7, 150, legend_y-1, 143, legend_y+5, 136, legend_y-1, fill=ENV_COLOR, outline="#222222")
        c.create_text(157, legend_y, anchor="w", text="ENV", font=("Segoe UI", 9))
        c.create_text(12, 34, anchor="w", text="Click ENV/BC nodes to filter the Live Log", font=("Segoe UI", 9))

    def _on_map_click(self, event) -> None:
        for x0, y0, x1, y1, node_type, node_id in self.map_hit_regions:
            if x0 <= event.x <= x1 and y0 <= event.y <= y1:
                if node_type == "ENV":
                    self.env_var.set(f"ENV {node_id}")
                    self._render_log()
                elif node_type == "BC":
                    self.bc_var.set(f"BC {node_id}")
                    self._render_log()
                break

    # ------------------------------------------------------------- Periodic
    def _periodic_refresh(self) -> None:
        self._draw_map()
        connected = bool(self.serial_worker and self.serial_worker.is_alive())
        serial_text = "CONNECTED" if connected else "DISCONNECTED"
        if self.csv_recorder.is_recording and self.csv_recorder.path:
            rec_text = f"RECORDING: {self.csv_recorder.path.name} | saved this run: {self.csv_recorder.rows_written}"
        else:
            rec_text = "NOT RECORDING"
        self.status_var.set(f"{serial_text} | packets received: {len(self.packets)} | {rec_text}")
        self.after(500, self._periodic_refresh)

    def _on_close(self) -> None:
        self._save_settings()
        self.csv_recorder.stop()
        if self.serial_worker and self.serial_worker.is_alive():
            self.serial_worker.stop()
        self.destroy()


def main() -> None:
    app = Dashboard()
    app.mainloop()


if __name__ == "__main__":
    main()
