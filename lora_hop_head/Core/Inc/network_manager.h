/*
 * network_manager.h
 *
 *  Created on: 19 ott 2026
 *      Author: LabMeas
 */

#ifndef INC_NETWORK_MANAGER_H_
#define INC_NETWORK_MANAGER_H_


#include "stdint.h"
#include "stdbool.h"
#include "lora.h"
#include "string.h"

/* ---------------- Node IDs ---------------- */

#define NODE_ID_UNASSIGNED     0xFF


/* ---------------- Node states ---------------- */

typedef enum {
	NODE_DISCOVERING,

	NODE_WAIT_DISC_CONFIRM,

	NODE_ASSIGNED_PENDING_LINK,

	/*
	 * MODE_SEL has been activated.
	 * DEPLOY_REQ is being retransmitted until
	 * DEPLOY_CONFIRM is received.
	 */
	NODE_WAIT_DEPLOY_CONFIRM,

	NODE_DEPLOYED
} node_net_state_t;


/* ---------------- Child association state ---------------- */

typedef enum {
	CHILD_CLOSED,
	CHILD_OPEN,
	CHILD_RESERVED
} child_accept_state_t;



/* ---------------- Link quality packets ---------------- */

// single PKT
typedef struct
{
	/*
	 * true:
	 * LINK_REPLY received for this probe.
	 *
	 * false:
	 * reply timeout.
	 */
	bool success;

	/*
	 * Valid only when success == true.
	 */
	int16_t rssi_worst;
	int8_t  snr_worst;

	uint16_t seq;

} link_sample_t;

// circular BUFF
typedef struct
{
	link_sample_t samples[LINK_WINDOW_SIZE];

	/*
	 * Index where the next attempt will be written.
	 */
	uint8_t write_idx;

	/*
	 * Number of valid attempts currently stored:
	 * 0 ... LINK_WINDOW_SIZE.
	 */
	uint8_t count;

} link_window_t;

// Link status during DISCOVERY mode
typedef enum
{
	/*
	 * The rolling window is not full yet.
	 */
	LINK_QUALITY_EVALUATING = 0,

	/*
	 * Reliable round-trip delivery and
	 * sufficient RSSI/SNR margin.
	 */
	LINK_QUALITY_GOOD,

	/*
	 * Link still works, but either packet
	 * success rate or radio margin is reduced.
	 */
	LINK_QUALITY_MARGINAL,

	/*
	 * Too many round-trip failures.
	 */
	LINK_QUALITY_BAD

} link_quality_state_t;

// link buffer metrics
typedef struct
{
	/*
	 * true only when the window contains
	 * LINK_WINDOW_SIZE attempts.
	 */
	bool ready;

	/*
	 * false only when no successful packets
	 * are available for RSSI/SNR calculation.
	 */
	bool percentiles_valid;

	uint8_t attempts;
	uint8_t successes;

	/*
	 * Percentage from 0 to 100.
	 */
	uint8_t success_rate_pct;

	int16_t rssi_pN;
	int8_t  snr_pN;

	/*
	 * Final classification derived from
	 * the metrics above.
	 */
	link_quality_state_t quality;

} link_metrics_t;

// full LINK context
typedef struct
{
	bool active;

	/*
	 * A LINK_PROBE was transmitted and its
	 * corresponding LINK_REPLY is still expected.
	 */
	bool waiting_reply;

	/*
	 * Sequence number used by the next LINK_PROBE.
	 */
	uint16_t next_seq;

	/*
	 * Sequence number of the outstanding probe.
	 */
	uint16_t pending_seq;


	/*
	 * Parent-side fields used to construct LINK_REPLY.
	 */
	uint16_t reply_seq;

	int16_t reply_rssi_child_to_parent;
	int8_t  reply_snr_child_to_parent;

	link_window_t window;
	link_metrics_t metrics;

} link_test_ctx_t;


/* ---------------- Network packet types ---------------- */

typedef enum
{
	NET_PKT_NONE 	 = 0x00,

	/* Discovery packets */
	DISC_REQ     	 = 0x01,
	DISC_OFFER  	 = 0x02,
	DISC_ACCEPT 	 = 0x03,
	DISC_CONFIRM	 = 0x04,

	/* Link-test packets */
	LINK_PROBE   	 = 0x10,
	LINK_REPLY   	 = 0x11,

	/* Deployment commit */
	DEPLOY_REQ     = 0x12,
	DEPLOY_CONFIRM = 0x13

} network_pkt_type_t;

/*
 * Action requested after processing a received
 * network-control packet.
 */
typedef enum {
	NET_RX_NO_ACTION,
	NET_RX_SCHEDULE_TX,
	NET_RX_BAD_PACKET
} network_rx_action_t;


/* --------------- Network PKTs priorities -------------- */
typedef enum {

    NET_TX_NONE,

    /*
     * Low-priority periodic traffic:
     * DISC_REQ while searching for a parent.
     */
    NET_TX_PERIODIC,

    /*
     * Retransmission:
     * currently DISC_ACCEPT waiting for CONFIRM.
     */
    NET_TX_RETX,

    /*
     * Immediate control response:
     * OFFER, first ACCEPT, CONFIRM.
     */
    NET_TX_PRIORITY

} network_tx_priority_t;


/* ---------------- Network handler ---------------- */

typedef struct {

	uint8_t node_id;

	uint8_t parent_id;

	uint8_t child_id;

	node_net_state_t state;

	child_accept_state_t child_state;

	bool child_association_confirmed;

	/*
	 * true after a valid DEPLOY_REQ has been
	 * received from the associated child.
	 */
	bool child_deployed;

	uint32_t discovery_token;

	uint32_t reserved_token;

	/*
	 * If != NET_PKT_NONE, a discovery packet
	 * must be transmitted.
	 */
	//discovery_pkt_type_t pending_disc_tx;
	network_pkt_type_t pending_net_tx;
	uint8_t net_tx_attempts;

	/*
	 * Network Link handler
	 */
	link_test_ctx_t link;

} network_ctx_t;


/* ---------------- Public functions ---------------- */

void network_init_unassigned(network_ctx_t *h_net);

void network_init_root(network_ctx_t *h_net);

bool network_build_discovery_req(
    const network_ctx_t *h_net,
    uint8_t *payload,
    uint8_t *payload_len
);

network_rx_action_t network_process_rx(
    network_ctx_t *h_net,
    const uint8_t *payload,
    uint8_t payload_len,
    int16_t rssi,
    int8_t snr
);

network_tx_priority_t network_get_tx_priority(
    const network_ctx_t *h_net
);

bool network_build_pending_tx(
    const network_ctx_t *h_net,
    uint8_t *payload,
    uint8_t *payload_len
);

void network_on_tx_done(
    network_ctx_t *h_net
);

void network_link_test_start(
    network_ctx_t *h_net
);

bool network_prepare_link_cycle(
    network_ctx_t *h_net
);

bool network_get_link_metrics(
    const network_ctx_t *h_net,
    link_metrics_t *metrics
);

bool network_request_deployment(
    network_ctx_t *h_net
);

#endif /* INC_NETWORK_MANAGER_H_ */
