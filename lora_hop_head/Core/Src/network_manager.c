/*
 * network_manager.c
 *
 *  Created on: 19 ott 2026
 *      Author: LabMeas
 */

#include "network_manager.h"
#include "sys_settings.h"



/* =========================== STATIC FUNCTIONs ==================================== */

static uint32_t network_get_token(
		const uint8_t *payload
);

static void network_put_token(
		uint8_t *payload,
		uint32_t token
);

static void link_window_push(
    link_test_ctx_t *h_link,
    bool success,
    int16_t rssi_worst,
    int8_t snr_worst,
    uint16_t seq
);

static void link_metrics_update(
    link_test_ctx_t *h_link
);

static void sort_int16_ascending(
    int16_t *values,
    uint8_t count
);

static void sort_int8_ascending(
    int8_t *values,
    uint8_t count
);

static void link_quality_update(
    link_metrics_t *metrics
);

/*
 * Helper to build token for BC identification during deployment state.
 *
 * @param  uint8_t*  payload  payload buffer (RAW)
 *
 * @return uint32_t  decoded token
 */
static uint32_t network_get_token(const uint8_t *payload)
{
    return
        ((uint32_t)payload[DISC_TOKEN_3_POS] << 24) |
        ((uint32_t)payload[DISC_TOKEN_2_POS] << 16) |
        ((uint32_t)payload[DISC_TOKEN_1_POS] << 8)  |
        ((uint32_t)payload[DISC_TOKEN_0_POS]);
}


/*
 * Helper to build token for BC identification during deployment state.
 *
 * @param  uint8_t*   payload  payload buffer (RAW)
 * @param  uint32_t   token    token number inserted in this payload
 *
 */
static void network_put_token(
    uint8_t *payload,
    uint32_t token
)
{
    payload[DISC_TOKEN_3_POS] =
        (uint8_t)((token >> 24) & 0xFF);

    payload[DISC_TOKEN_2_POS] =
        (uint8_t)((token >> 16) & 0xFF);

    payload[DISC_TOKEN_1_POS] =
        (uint8_t)((token >> 8) & 0xFF);

    payload[DISC_TOKEN_0_POS] =
        (uint8_t)(token & 0xFF);
}


/*
 * Push a PKT into the link-testing queue.
 * Number of PKTs limited to LINK_WINDOW_SIZE.
 *
 * @param  link_test_ctx_t *h_link     LINK context handler for this node (contains all the informations - queues/structures)
 * @param  bool            success     true if LINK_REPLY received before timeout, false otherwise
 * @param  int16_t         rssi_worst  worst RSSI measured in the link (child->parent; parent->child)
 * @param  int8_t          snr_worst   worst SNR measured in the link (child->parent; parent->child)
 * @param  uint16_t 			 seq         pkt_id corresponding to this link test request
 */
static void link_window_push(
    link_test_ctx_t *h_link,
    bool success,
    int16_t rssi_worst,
    int8_t snr_worst,
    uint16_t seq
)
{
	uint8_t idx =
			h_link->window.write_idx;

	h_link->window.samples[idx].success =
			success;

	h_link->window.samples[idx].rssi_worst =
			rssi_worst;

	h_link->window.samples[idx].snr_worst =
			snr_worst;

	h_link->window.samples[idx].seq =
			seq;


	/*
	 * Circular increment.
	 */
	h_link->window.write_idx =
			(uint8_t)(
					(idx + 1u) %
					LINK_WINDOW_SIZE
			);

	/*
	 * Count saturates at LINK_WINDOW_SIZE.
	 */
	if (h_link->window.count <
			LINK_WINDOW_SIZE)
	{
		h_link->window.count++;
	}

	/*
	 * Recalculate metrics after every new attempt.
	 */
	link_metrics_update(h_link);
}


/*
 * Insertion sort for a buffer of int16_t (RSSI in this APP).
 *
 * @param int16_t *values Buffer to be sorted
 * @param uint8_t count		Elements in the buffer
 *
 */
static void sort_int16_ascending(
    int16_t *values,
    uint8_t count
)
{
	for (uint8_t i = 1u; i < count; i++)
	{
		int16_t key =
				values[i];

		int16_t j =
				(int16_t)i - 1;

		while ((j >= 0) &&
					 (values[j] > key))
		{
			values[j + 1] =
					values[j];

			j--;
		}

		values[j + 1] =
				key;
	}
}


/*
 * Insertion sort for a buffer of int8_t (SNR in this APP).
 *
 * @param int8_t  *values Buffer to be sorted
 * @param uint8_t count		Elements in the buffer
 *
 */
static void sort_int8_ascending(
    int8_t *values,
    uint8_t count
)
{
	for (uint8_t i = 1u; i < count; i++)
	{
		int8_t key =
				values[i];

		int16_t j =
				(int16_t)i - 1;

		while ((j >= 0) &&
					 (values[j] > key))
		{
			values[j + 1] =
					values[j];

			j--;
		}

		values[j + 1] =
				key;
	}
}


/*
 * Update metrics for the acquired LINK TEST window.
 *
 * Computed metrics: - success rate (link probes with received reply) over the last N-pkts (length of the window, see sys_settings.h)
 *                   - percentile over RSSI (percentile threshold specified in sys_settings)
 *                   - percentile over SNR  (percentile threshold specified in sys_settings)
 *
 *                   NOTE: metrics must be considered valid only after the entire buffer has been filled.
 *                         (notified by h_link->metrics.ready field)
 *
 * @param link_test_ctx_t *h_link network-link context handler for this application
 */
static void link_metrics_update(
    link_test_ctx_t *h_link
)
{
	uint8_t attempts =
			h_link->window.count;

	uint8_t successes =
			0u;

	int16_t rssi_values[LINK_WINDOW_SIZE];
	int8_t  snr_values[LINK_WINDOW_SIZE];


	for (uint8_t i = 0u;
			 i < attempts;
			 i++)
	{
		const link_sample_t *sample =
				&h_link->window.samples[i];

		if (!sample->success)
		{
				continue;
		}

		rssi_values[successes] =
				sample->rssi_worst;

		snr_values[successes] =
				sample->snr_worst;

		successes++;
	}


	h_link->metrics.attempts =
			attempts;

	h_link->metrics.successes =
			successes;


	if (attempts > 0u)
	{
		h_link->metrics.success_rate_pct =
				(uint8_t)(
						((uint16_t)successes * 100u) /
						attempts
				);
	}
	else
	{
		h_link->metrics.success_rate_pct =
				0u;
	}


	/*
	 * Metrics become operational only after the
	 * entire N-attempt window has been filled.
	 */
	h_link->metrics.ready =
			(attempts == LINK_WINDOW_SIZE);


	h_link->metrics.percentiles_valid =
			(successes > 0u);


	if (successes == 0u)
	{
		h_link->metrics.rssi_pN =
				0;

		h_link->metrics.snr_pN =
				0;

		link_quality_update(
				&h_link->metrics
		);

		return;
	}


	sort_int16_ascending(
			rssi_values,
			successes
	);

	sort_int8_ascending(
			snr_values,
			successes
	);


	/*
	 * Nearest-rank percentile:
	 *
	 * example: rank = ceil(0.20 * N)
	 *
	 * Array index = rank - 1.
	 */
	uint8_t rank =
			(uint8_t)(
					((LINK_METRICS_PERCENTILE * successes) + 99u) /
					100u
			);

	uint8_t pN_idx =
			(uint8_t)(rank - 1u);


	h_link->metrics.rssi_pN =
			rssi_values[pN_idx];

	h_link->metrics.snr_pN =
			snr_values[pN_idx];

	link_quality_update(
			&h_link->metrics
	);
}


/*
 * Evaluate current LINK QUALITY STATUS
 */
static void link_quality_update(
    link_metrics_t *metrics
)
{
	/*
	 * Do not make a decision before the
	 * rolling window is completely filled.
	 */
	if (!metrics->ready)
	{
		metrics->quality =
				LINK_QUALITY_EVALUATING;

		return;
	}


	/*
	 * Too many actual round-trip failures.
	 *
	 * With a ten-packet window:
	 *
	 * 0 ... 6 successes -> BAD.
	 */
	if (metrics->success_rate_pct <=
			LINK_SUCCESS_RATE_BAD_MAX_PCT)
	{
		metrics->quality =
				LINK_QUALITY_BAD;

		return;
	}


	/*
	 * Intermediate packet-success rate.
	 *
	 * With a ten-packet window:
	 *
	 * 7 or 8 successes -> MARGINAL.
	 */
	if (metrics->success_rate_pct <
			LINK_SUCCESS_RATE_GOOD_MIN_PCT)
	{
		metrics->quality =
				LINK_QUALITY_MARGINAL;

		return;
	}


	/*
	 * At this point the success rate is satisfactory.
	 * The link can be GOOD only if the lower tail of
	 * both RSSI and SNR distributions is acceptable.
	 */
	if (!metrics->percentiles_valid)
	{
		/*
		 * Defensive fallback.
		 *
		 * In practice this condition cannot occur with
		 * high success_rate, but avoids using invalid
		 * RSSI/SNR values.
		 */
		metrics->quality =
				LINK_QUALITY_MARGINAL;

		return;
	}


	if ((metrics->rssi_pN <
			 LINK_RSSI_PERC_GOOD_MIN_DBM) ||

			(metrics->snr_pN <
			 LINK_SNR_PERC_GOOD_MIN_DB))
	{
		metrics->quality =
				LINK_QUALITY_MARGINAL;

		return;
	}

	metrics->quality =
			LINK_QUALITY_GOOD;
}

/* =========================== PUBLIC FUNCTIONS ==================================== */


/*
 * Initialize generic BC-node (not valid for head BC-node).
 *
 * @param network_ctx_t *h_net network handler structure for BC-node
 */
void network_init_unassigned(network_ctx_t *h_net)
{
    h_net->node_id = NODE_ID_UNASSIGNED;

    h_net->parent_id = NODE_ID_UNASSIGNED;

    h_net->state = NODE_DISCOVERING;

    h_net->child_state = CHILD_CLOSED;

    h_net->discovery_token = 0;

    h_net->reserved_token = 0;

    h_net->pending_net_tx = DISC_REQ;

    h_net->child_deployed = false;

}


/*
 * Initialize NODE 0 (head connected to serial port).
 *
 * @param network_ctx_t *h_net network handler structure for head BC-node
 */
void network_init_root(network_ctx_t *h_net)
{
    h_net->node_id = 0;

    h_net->parent_id = NODE_ID_UNASSIGNED;

    h_net->state = NODE_DEPLOYED;

    h_net->child_state = CHILD_OPEN;

    h_net->discovery_token = 0;

    h_net->reserved_token = 0;

    h_net->child_deployed = false;
}


/*
 * Build discovery PKT used to define the BC ID for this node, starting from
 * the devices already deployed in the sequence.
 *
 * @return bool: - false if sys is not in discovering mode
 *               - true  if the payload was correctly generated
 */
bool network_build_discovery_req(const network_ctx_t *h_net, uint8_t *payload, uint8_t *payload_len)
{
	if (h_net->state != NODE_DISCOVERING) return false;

	/* Application sync word */
	payload[DISC_SYNC_MSB_POS] =
			(uint8_t)((DISCOVERY_SYNC_WORD >> 8) & 0xFF);

	payload[DISC_SYNC_LSB_POS] =
			(uint8_t)(DISCOVERY_SYNC_WORD & 0xFF);

	/* Packet type */
	payload[DISC_TYPE_POS] = DISC_REQ;

	/* Discovery token */
	payload[DISC_TOKEN_3_POS] =
			(uint8_t)((h_net->discovery_token >> 24) & 0xFF);

	payload[DISC_TOKEN_2_POS] =
			(uint8_t)((h_net->discovery_token >> 16) & 0xFF);

	payload[DISC_TOKEN_1_POS] =
			(uint8_t)((h_net->discovery_token >> 8) & 0xFF);

	payload[DISC_TOKEN_0_POS] =
			(uint8_t)(h_net->discovery_token & 0xFF);

	*payload_len = DISC_REQ_SIZE;

	return true;
}


/*
 * Once the node enters LINK_TEST state, clear any pending request
 * and setup link structure for proper operation
 *
 * @param network_ctx_t* h_net network context handler
 */
void network_link_test_start(
    network_ctx_t *h_net
)
{
	if (h_net == NULL)
	{
		return;
	}

	/*
	 * Clear previous link-test data.
	 */
	memset(
			&h_net->link,
			0,
			sizeof(h_net->link)
	);

	/*
	 * Start LINK evaluation mode
	 */
	h_net->link.metrics.quality =
	    LINK_QUALITY_EVALUATING;

	h_net->link.active = true;

	/*
	 * First probe will use sequence number 0.
	 */
	h_net->link.next_seq = 0u;

	/*
	 * Prepare first LINK_PROBE.
	 */
	h_net->pending_net_tx =
			LINK_PROBE;

	h_net->net_tx_attempts = 0u;
}


/*
 * Prepares next probe request. To success the node must be in NODE_ASSIGNED_PENDING_LINK state,
 * with link active, and with no pending network TX.
 * This function is called immediately before the TX happen. In this way if a "waiting-reply" is
 * still pending at the moment of the TX, we consider a timeout and the link failed to forward this request.
 *
 * @param network_ctx_t *h_net  network context handler.
 *
 * @return bool  True if a network action is pending/submitted;
 *           		 False if no action is done (bad pkt for the current operating state of this node)
 *
 */
bool network_prepare_link_cycle(
    network_ctx_t *h_net
)
{
	if (h_net == NULL)
	{
		return false;
	}

	if ((h_net->state != NODE_ASSIGNED_PENDING_LINK) ||
			(!h_net->link.active))
	{
		return false;
	}

	/*
	 * Do not overwrite another network-control
	 * packet already waiting for TX.
	 */
	if (h_net->pending_net_tx != NET_PKT_NONE)
	{
		return true;
	}

	/*
	 * Previous probe is still unanswered:
	 * count it as a failed attempt.
	 */
	if (h_net->link.waiting_reply)
	{
		link_window_push(
				&h_net->link,
				false,
				0,
				0,
				h_net->link.pending_seq
		);

		h_net->link.waiting_reply =
				false;
	}

	/*
	 * Prepare the next probe.
	 */
	h_net->pending_net_tx =
			LINK_PROBE;

	h_net->net_tx_attempts =
			0u;

	return true;
}


/*
 * Get link metrics during LINK-TESTING (while depositing an intermediate sensor node).
 *
 * @param const network_ctx_t *h_net    network context handler for this node
 * @param link_metrics_t      *metrics  structure pointer which will contain the current metrics
 *
 * @return bool True if the buffer contains enough values to compute the desired metrics, False too few elements
 */
bool network_get_link_metrics(
    const network_ctx_t *h_net,
    link_metrics_t *metrics
)
{
	if ((h_net == NULL) ||
			(metrics == NULL))
	{
		return false;
	}

	*metrics =
			h_net->link.metrics;

	return h_net->link.metrics.ready;
}


/*
 * This function prepares the node for deployment mode, and is
 * intended for nodes in deployment state. After this call, the node
 * will inform the parent's node about the state change (deployment -> deployed).
 *
 * @param network_ctx_t *h_net network context handler
 *
 * @return bool True   if node is operating in the expected NODE_ASSIGNED_PENDING_LINK status
 *              False  if node is operating a state different from NODE_ASSIGNED_PENDING_LINK (Deployment invalid)
 */
bool network_request_deployment(
    network_ctx_t *h_net
)
{
	if (h_net == NULL)
	{
		return false;
	}

	/*
	 * Deployment can be requested only after
	 * discovery and during LINK TEST.
	 */
	if (h_net->state !=
			NODE_ASSIGNED_PENDING_LINK)
	{
		return false;
	}

	/*
	 * Stop generating new LINK_PROBE packets.
	 *
	 * Existing metrics are preserved, so the final
	 * link-quality result remains available.
	 */
	h_net->link.active =
			false;

	h_net->link.waiting_reply =
			false;

	/*
	 * Enter reliable deployment handshake.
	 */
	h_net->state =
			NODE_WAIT_DEPLOY_CONFIRM;

	h_net->pending_net_tx =
			DEPLOY_REQ;

	h_net->net_tx_attempts =
			0u;

	return true;
}


/*
 * Process RX PKTs during deployment state
 *
 * @param network_ctx_t* h_net        network context handler
 * @param uint8_t*       payload      RAW payload buff
 * @param uint8_t        payload_len  received payload len
 * @param int16_t        rssi         RSSI for this payload buff
 * @param int8_t         snr          SNR for this payload buff
 *
 * @return network_rx_action_t  next required action which should be handled by main application.
 */
network_rx_action_t network_process_rx(
    network_ctx_t *h_net,
    const uint8_t *payload,
    uint8_t payload_len,
    int16_t rssi,
    int8_t snr
)
{
	/*
	 * RSSI/SNR not used during discovery yet.
	 * They will be used by LINK_TEST.
	 */
	(void)rssi;
	(void)snr;

	/* Minimum discovery packet size */
	if (payload_len < DISC_REQ_SIZE)
	{
			return NET_RX_BAD_PACKET;
	}

	/* Check discovery sync word */
	uint16_t sync =
			((uint16_t)payload[DISC_SYNC_MSB_POS] << 8) |
			((uint16_t)payload[DISC_SYNC_LSB_POS]);

	if (sync != DISCOVERY_SYNC_WORD)
	{
			return NET_RX_BAD_PACKET;
	}


	network_pkt_type_t pkt_type =
	        (network_pkt_type_t)payload[DISC_TYPE_POS];

	uint32_t token = network_get_token(payload);

	switch (pkt_type)
	{

		/* =========================================================
		 * DISCOVERY REQUEST
		 * ========================================================= */
		case DISC_REQ:
		{
			if (payload_len != DISC_REQ_SIZE)
			{
				return NET_RX_BAD_PACKET;
			}

			/*
			 * Only a deployed node can accept a new child.
			 */
			if (h_net->state != NODE_DEPLOYED)
			{
				return NET_RX_NO_ACTION;
			}

			/*
			 * Case 1:
			 * this node is the current end of the chain.
			 */
			if (h_net->child_state == CHILD_OPEN)
			{
				/*
				 * Reserve this node for this discovery transaction.
				 */
				h_net->reserved_token = token;

				h_net->child_association_confirmed = false;

				h_net->child_deployed = false;

				h_net->child_state = CHILD_RESERVED;

				/*
				 * Prepare DISC_OFFER.
				 */
				h_net->pending_net_tx = DISC_OFFER;

				return NET_RX_SCHEDULE_TX;
			}


			/*
			 * Case 2:
			 * this node is already reserved, but the same
			 * requester is retransmitting DISC_REQ.
			 *
			 * Probably the previous OFFER was lost.
			 */
			if ((h_net->child_state == CHILD_RESERVED) &&
					(h_net->reserved_token == token))
			{
				/*
				 * OFFER was already accepted.
				 * Ignore old/repeated DISC_REQ.
				 *
				 * From this point on, reliability is handled
				 * by ACCEPT -> CONFIRM.
				 */
				if (h_net->child_association_confirmed)
				{
					return NET_RX_NO_ACTION;
				}

				h_net->pending_net_tx = DISC_OFFER;

				return NET_RX_SCHEDULE_TX;
			}


			/*
			 * CHILD_CLOSED or reservation belongs
			 * to another node.
			 */
			return NET_RX_NO_ACTION;
		}


		/* =========================================================
		 * DISCOVERY OFFER
		 * ========================================================= */
		case DISC_OFFER:
		{
			if (payload_len != DISC_OFFER_SIZE)
			{
				return NET_RX_BAD_PACKET;
			}

			/*
			 * Offers make sense only while we are searching
			 * for our parent.
			 */
			if (h_net->state != NODE_DISCOVERING)
			{
				return NET_RX_NO_ACTION;
			}


			/*
			 * Ignore OFFERs belonging to an older/different
			 * discovery transaction.
			 */
			if (token != h_net->discovery_token)
			{
				return NET_RX_NO_ACTION;
			}


			uint8_t parent_id =
					payload[DISC_PARENT_ID_POS];

			uint8_t offered_id =
					payload[DISC_NODE_ID_POS];


			/*
			 * Basic consistency check.
			 *
			 * In our linear topology:
			 *
			 * child ID = parent ID + 1
			 */
			if ((parent_id == NODE_ID_UNASSIGNED) ||
					(offered_id == NODE_ID_UNASSIGNED) ||
					(offered_id != (uint8_t)(parent_id + 1)))
			{
				return NET_RX_BAD_PACKET;
			}


			/*
			 * Association accepted locally.
			 */
			h_net->parent_id = parent_id;

			h_net->node_id = offered_id;

			h_net->state = NODE_WAIT_DISC_CONFIRM;


			/*
			 * Reply with DISC_ACCEPT.
			 */
			h_net->pending_net_tx = DISC_ACCEPT;

			return NET_RX_SCHEDULE_TX;
		}


		/* =========================================================
		 * DISCOVERY ACCEPT
		 * ========================================================= */
		case DISC_ACCEPT:
		{
			if (payload_len != DISC_ACCEPT_SIZE)
			{
				return NET_RX_BAD_PACKET;
			}


			/*
			 * Only the parent that reserved a child
			 * is interested in DISC_ACCEPT.
			 */
			if ((h_net->state != NODE_DEPLOYED) ||
					(h_net->child_state != CHILD_RESERVED))
			{
				return NET_RX_NO_ACTION;
			}


			/*
			 * Must belong to the same transaction.
			 */
			if (token != h_net->reserved_token)
			{
				return NET_RX_NO_ACTION;
			}


			uint8_t parent_id =
					payload[DISC_PARENT_ID_POS];

			uint8_t child_id =
					payload[DISC_NODE_ID_POS];


			/*
			 * This ACCEPT must be explicitly addressed
			 * to me.
			 */
			if (parent_id != h_net->node_id)
			{
				return NET_RX_NO_ACTION;
			}


			/*
			 * Child ID must match the ID we offered.
			 */
			if (child_id != (uint8_t)(h_net->node_id + 1))
			{
				return NET_RX_BAD_PACKET;
			}


			/*
			 * Association confirmed.
			 *
			 * Parent remains RESERVED because the new node
			 * has not been physically deployed yet.
			 */
			h_net->child_id = child_id;

			h_net->child_association_confirmed = true;

			h_net->pending_net_tx = DISC_CONFIRM;

			return NET_RX_SCHEDULE_TX;
		}

		/* =========================================================
		 * DISCOVERY CONFIRM
		 * ========================================================= */
		case DISC_CONFIRM:
		{
			if (payload_len != DISC_CONFIRM_SIZE)
			{
				return NET_RX_BAD_PACKET;
			}

			/*
			 * A CONFIRM is meaningful only after an OFFER
			 * has been accepted.
			 */
			if (h_net->state != NODE_WAIT_DISC_CONFIRM)
			{
				return NET_RX_NO_ACTION;
			}

			/*
			 * Must belong to my current discovery transaction.
			 */
			if (token != h_net->discovery_token)
			{
				return NET_RX_NO_ACTION;
			}

			uint8_t parent_id =
					payload[DISC_PARENT_ID_POS];

			uint8_t child_id =
					payload[DISC_NODE_ID_POS];

			/*
			 * CONFIRM must exactly match the previously
			 * accepted association.
			 */
			if ((parent_id != h_net->parent_id) ||
					(child_id != h_net->node_id))
			{
				return NET_RX_NO_ACTION;
			}

			/*
			 * Discovery completed.
			 * Next phase will be LINK TEST.
			 */
			h_net->state = NODE_ASSIGNED_PENDING_LINK;

			/*
			 * Start link test and prepare first probe.
			 */
			network_link_test_start(h_net);

			/*
			 * Ask main application to schedule the first probe.
			 */
			return NET_RX_SCHEDULE_TX;
		}


		/* =========================================================
		 * LINK PROBE
		 * ========================================================= */
		case LINK_PROBE:
		{
			if (payload_len != LINK_PROBE_SIZE)
			{
				return NET_RX_BAD_PACKET;
			}

			/*
			 * Only an already deployed node can act
			 * as parent.
			 */
			if (h_net->state != NODE_DEPLOYED)
			{
				return NET_RX_NO_ACTION;
			}

			/*
			 * Parent must have already completed the
			 * discovery association with this child.
			 */
			if ((h_net->child_state != CHILD_RESERVED) ||
					(!h_net->child_association_confirmed))
			{
				return NET_RX_NO_ACTION;
			}

			if (token != h_net->reserved_token)
			{
				return NET_RX_NO_ACTION;
			}

			uint8_t parent_id =
					payload[DISC_PARENT_ID_POS];

			uint8_t child_id =
					payload[DISC_NODE_ID_POS];

			if ((parent_id != h_net->node_id) ||
					(child_id != h_net->child_id))
			{
				return NET_RX_NO_ACTION;
			}

			uint16_t seq =
					((uint16_t)payload[LINK_SEQ_MSB_POS] << 8) |
					((uint16_t)payload[LINK_SEQ_LSB_POS]);

			/*
			 * rssi and snr describe the received
			 * child -> parent LINK_PROBE.
			 */
			h_net->link.reply_seq =
					seq;

			h_net->link.reply_rssi_child_to_parent =
					rssi;

			h_net->link.reply_snr_child_to_parent =
					snr;

			/*
			 * Prepare one-shot response.
			 */
			h_net->pending_net_tx =
					LINK_REPLY;

			h_net->net_tx_attempts = 0u;

			return NET_RX_SCHEDULE_TX;
		}


		/* =========================================================
		 * LINK REPLY
		 * ========================================================= */
		case LINK_REPLY:
		{
			if (payload_len != LINK_REPLY_SIZE)
			{
				return NET_RX_BAD_PACKET;
			}

			if ((h_net->state != NODE_ASSIGNED_PENDING_LINK) ||
					(!h_net->link.active))
			{
				return NET_RX_NO_ACTION;
			}

			if (token != h_net->discovery_token)
			{
				return NET_RX_NO_ACTION;
			}

			uint8_t parent_id =
					payload[DISC_PARENT_ID_POS];

			uint8_t child_id =
					payload[DISC_NODE_ID_POS];

			if ((parent_id != h_net->parent_id) ||
					(child_id != h_net->node_id))
			{
				return NET_RX_NO_ACTION;
			}

			uint16_t seq =
					((uint16_t)payload[LINK_SEQ_MSB_POS] << 8) |
					((uint16_t)payload[LINK_SEQ_LSB_POS]);

			/*
			 * Reject stale, duplicated or unrelated replies.
			 */
			if ((!h_net->link.waiting_reply) ||
					(seq != h_net->link.pending_seq))
			{
				return NET_RX_NO_ACTION;
			}

			uint16_t rssi_raw =
					((uint16_t)payload[LINK_RSSI_MSB_POS] << 8) |
					((uint16_t)payload[LINK_RSSI_LSB_POS]);

			int16_t rssi_child_to_parent =
					(int16_t)rssi_raw;

			int8_t snr_child_to_parent =
					(int8_t)payload[LINK_SNR_POS];

			/*
			 * Local rssi and snr describe
			 * parent -> child.
			 */
			int16_t rssi_worst =
					(rssi_child_to_parent < rssi) ?
					rssi_child_to_parent :
					rssi;

			int8_t snr_worst =
					(snr_child_to_parent < snr) ?
					snr_child_to_parent :
					snr;

			link_window_push(
					&h_net->link,
					true,
					rssi_worst,
					snr_worst,
					seq
			);

			/*
			 * Round trip completed.
			 */
			h_net->link.waiting_reply =
					false;

			return NET_RX_NO_ACTION;
		}


		/* =========================================================
		 * DEPLOYMENT REQUEST
		 * ========================================================= */
		case DEPLOY_REQ:
		{
			if (payload_len != DEPLOY_REQ_SIZE)
			{
				return NET_RX_BAD_PACKET;
			}

			/*
			 * Only an already deployed node can act
			 * as parent.
			 */
			if (h_net->state != NODE_DEPLOYED)
			{
				return NET_RX_NO_ACTION;
			}

			/*
			 * Request must belong to the association
			 * established during discovery.
			 */
			if (token != h_net->reserved_token)
			{
				return NET_RX_NO_ACTION;
			}

			uint8_t parent_id =
					payload[DISC_PARENT_ID_POS];

			uint8_t child_id =
					payload[DISC_NODE_ID_POS];

			if ((parent_id != h_net->node_id) ||
					(child_id != h_net->child_id))
			{
				return NET_RX_NO_ACTION;
			}

			/*
			 * First valid DEPLOY_REQ:
			 *
			 * the parent was RESERVED while the child
			 * performed LINK TEST.
			 */
			bool first_request =
					(h_net->child_state ==
							 CHILD_RESERVED) &&
					h_net->child_association_confirmed &&
					(!h_net->child_deployed);

			/*
			 * Duplicate DEPLOY_REQ:
			 *
			 * probably the previous DEPLOY_CONFIRM
			 * was lost.
			 */
			bool repeated_request =
					(h_net->child_state ==
							 CHILD_CLOSED) &&
					h_net->child_association_confirmed &&
					h_net->child_deployed;

			if ((!first_request) &&
					(!repeated_request))
			{
				return NET_RX_NO_ACTION;
			}

			if (first_request)
			{
				/*
				 * This parent is no longer allowed to
				 * accept a new child.
				 */
				h_net->child_state =
						CHILD_CLOSED;

				h_net->child_deployed =
						true;
			}

			/*
			 * Send or resend confirmation.
			 */
			h_net->pending_net_tx =
					DEPLOY_CONFIRM;

			h_net->net_tx_attempts =
					0u;

			return NET_RX_SCHEDULE_TX;
		}


		/* =========================================================
		 * DEPLOYMENT CONFIRM
		 * ========================================================= */
		case DEPLOY_CONFIRM:
		{
			if (payload_len != DEPLOY_CONFIRM_SIZE)
			{
				return NET_RX_BAD_PACKET;
			}

			if (h_net->state !=
					NODE_WAIT_DEPLOY_CONFIRM)
			{
				return NET_RX_NO_ACTION;
			}

			if (token != h_net->discovery_token)
			{
				return NET_RX_NO_ACTION;
			}

			uint8_t parent_id =
					payload[DISC_PARENT_ID_POS];

			uint8_t child_id =
					payload[DISC_NODE_ID_POS];

			/*
			 * CONFIRM must refer exactly to this
			 * parent-child association.
			 */
			if ((parent_id != h_net->parent_id) ||
					(child_id != h_net->node_id))
			{
				return NET_RX_NO_ACTION;
			}

			/*
			 * Deployment completed.
			 */
			h_net->state =
					NODE_DEPLOYED;

			/*
			 * This node is now the end of the chain
			 * and may accept the next child.
			 */
			h_net->child_state =
					CHILD_OPEN;

			h_net->child_id =
					NODE_ID_UNASSIGNED;

			h_net->child_association_confirmed =
					false;

			h_net->child_deployed =
					false;

			h_net->link.active =
					false;

			h_net->link.waiting_reply =
					false;

			/*
			 * Stop DEPLOY_REQ retransmissions.
			 */
			h_net->pending_net_tx =
					NET_PKT_NONE;

			h_net->net_tx_attempts =
					0u;

			return NET_RX_NO_ACTION;
		}


		/* =========================================================
		 * DEFAULT
		 * ========================================================= */
		default:
				return NET_RX_BAD_PACKET;
	}
}


/*
 * Build TX payload for a DISCOVERY PKT
 *
 * @param const network_ctx_t* h_net        network context handler
 * @param uint8_t*             payload      payload buffer to be modified
 * @param uint8_t*             payload_len  modified payload len (for full DISCOVERY PKT)
 *
 * return bool : true if everything ok
 *
 */
bool network_build_pending_tx(
    const network_ctx_t *h_net,
    uint8_t *payload,
    uint8_t *payload_len
)
{
	if (h_net->pending_net_tx == NET_PKT_NONE)
	{
		return false;
	}

	/* Common header */
	payload[DISC_SYNC_MSB_POS] =
			(uint8_t)((DISCOVERY_SYNC_WORD >> 8) & 0xFF);

	payload[DISC_SYNC_LSB_POS] =
			(uint8_t)(DISCOVERY_SYNC_WORD & 0xFF);

	payload[DISC_TYPE_POS] =
			(uint8_t)h_net->pending_net_tx;


	switch (h_net->pending_net_tx)
	{
		/* =====================================================
		 * DISCOVERY REQUEST
		 * ===================================================== */
		case DISC_REQ:
		{
			if (h_net->state != NODE_DISCOVERING)
			{
				return false;
			}

			network_put_token(
					payload,
					h_net->discovery_token
			);

			*payload_len = DISC_REQ_SIZE;

			return true;
		}


		/* =====================================================
		 * DISCOVERY OFFER
		 * ===================================================== */
		case DISC_OFFER:
		{
			if ((h_net->state != NODE_DEPLOYED) ||
					(h_net->child_state != CHILD_RESERVED))
			{
				return false;
			}

			network_put_token(
					payload,
					h_net->reserved_token
			);

			payload[DISC_PARENT_ID_POS] =
					h_net->node_id;

			payload[DISC_NODE_ID_POS] =
					(uint8_t)(h_net->node_id + 1);

			*payload_len = DISC_OFFER_SIZE;

			return true;
		}


		/* =====================================================
		 * DISCOVERY ACCEPT
		 * ===================================================== */
		case DISC_ACCEPT:
		{
			if (h_net->state != NODE_WAIT_DISC_CONFIRM)
			{
				return false;
			}

			network_put_token(
					payload,
					h_net->discovery_token
			);

			payload[DISC_PARENT_ID_POS] =
					h_net->parent_id;

			payload[DISC_NODE_ID_POS] =
					h_net->node_id;

			*payload_len = DISC_ACCEPT_SIZE;

			return true;
		}


		/* =====================================================
		 * DISCOVERY CONFIRM
		 * ===================================================== */
		case DISC_CONFIRM:
		{
			if ((h_net->state != NODE_DEPLOYED) ||
					(h_net->child_state != CHILD_RESERVED) ||
					(!h_net->child_association_confirmed))
			{
				return false;
			}

			network_put_token(
					payload,
					h_net->reserved_token
			);

			payload[DISC_PARENT_ID_POS] =
					h_net->node_id;

			payload[DISC_NODE_ID_POS] =
					h_net->child_id;

			*payload_len = DISC_CONFIRM_SIZE;

			return true;
		}



		/* =====================================================
		 * LINK PROBE
		 * ===================================================== */
		case LINK_PROBE:
		{
			/*
			 * Only nodes looking for link-test are valid
			 */
			if ((h_net->state != NODE_ASSIGNED_PENDING_LINK) ||
					(!h_net->link.active))
			{
				return false;
			}

			/*
			 * Same transaction token used during discovery.
			 */
			network_put_token(
					payload,
					h_net->discovery_token
			);

			payload[DISC_PARENT_ID_POS] =
					h_net->parent_id;

			payload[DISC_NODE_ID_POS] =
					h_net->node_id;

			payload[LINK_SEQ_MSB_POS] =
					(uint8_t)(
							(h_net->link.next_seq >> 8) &
							0xFFu
					);

			payload[LINK_SEQ_LSB_POS] =
					(uint8_t)(
							h_net->link.next_seq &
							0xFFu
					);

			*payload_len =
					LINK_PROBE_SIZE;

			return true;
		}


		/* =====================================================
		 * LINK REPLY
		 * ===================================================== */
		case LINK_REPLY:
		{
			if ((h_net->state != NODE_DEPLOYED) ||
					(h_net->child_state != CHILD_RESERVED) ||
					(!h_net->child_association_confirmed))
			{
				return false;
			}

			network_put_token(
					payload,
					h_net->reserved_token
			);

			payload[DISC_PARENT_ID_POS] =
					h_net->node_id;

			payload[DISC_NODE_ID_POS] =
					h_net->child_id;

			payload[LINK_SEQ_MSB_POS] =
					(uint8_t)(
							(h_net->link.reply_seq >> 8) &
							0xFFu
					);

			payload[LINK_SEQ_LSB_POS] =
					(uint8_t)(
							h_net->link.reply_seq &
							0xFFu
					);

			/*
			 * Serialize signed RSSI using its 16-bit
			 * two's-complement representation.
			 */
			uint16_t rssi_raw =
					(uint16_t)
					h_net->link.reply_rssi_child_to_parent;

			payload[LINK_RSSI_MSB_POS] =
					(uint8_t)(
							(rssi_raw >> 8) &
							0xFFu
					);

			payload[LINK_RSSI_LSB_POS] =
					(uint8_t)(
							rssi_raw &
							0xFFu
					);

			/*
			 * Serialize signed int8_t SNR.
			 */
			payload[LINK_SNR_POS] =
					(uint8_t)
					h_net->link.reply_snr_child_to_parent;

			*payload_len =
					LINK_REPLY_SIZE;

			return true;
		}


		/* =====================================================
		 * DEPLOYMENT REQUEST
		 * ===================================================== */
		case DEPLOY_REQ:
		{
			if (h_net->state !=
					NODE_WAIT_DEPLOY_CONFIRM)
			{
				return false;
			}

			network_put_token(
					payload,
					h_net->discovery_token
			);

			payload[DISC_PARENT_ID_POS] =
					h_net->parent_id;

			payload[DISC_NODE_ID_POS] =
					h_net->node_id;

			*payload_len =
					DEPLOY_REQ_SIZE;

			return true;
		}


		/* =====================================================
		 * DEPLOYMENT CONFIRM
		 * ===================================================== */
		case DEPLOY_CONFIRM:
		{
			if ((h_net->state != NODE_DEPLOYED) ||
					(h_net->child_state != CHILD_CLOSED) ||
					(!h_net->child_association_confirmed) ||
					(!h_net->child_deployed))
			{
				return false;
			}

			network_put_token(
					payload,
					h_net->reserved_token
			);

			payload[DISC_PARENT_ID_POS] =
					h_net->node_id;

			payload[DISC_NODE_ID_POS] =
					h_net->child_id;

			*payload_len =
					DEPLOY_CONFIRM_SIZE;

			return true;
		}


		/* =====================================================
		 * DEFAULT
		 * ===================================================== */
		default:
			return false;
	}
}


/*
 * Get priority for a discovery PKT
 *
 * @param  const network_ctx_t *h_net network context handler
 *
 * @return network_tx_priority_t : ENUM priority for discovery PKTs
 */
network_tx_priority_t network_get_tx_priority(
    const network_ctx_t *h_net
)
{
	switch (h_net->pending_net_tx)
	{
		case NET_PKT_NONE:
		{
			return NET_TX_NONE;
		}

		case DISC_REQ:
		{
			/*
			 * Periodic discovery advertisement.
			 * Lowest priority.
			 */
			return NET_TX_PERIODIC;
		}

		case DISC_OFFER:
		{
			return NET_TX_PRIORITY;
		}

		case DISC_ACCEPT:
		{
			/*
			 * ACCEPT has high priority.
			 */

			return NET_TX_PRIORITY;
		}


		case DISC_CONFIRM:
		{
			return NET_TX_PRIORITY;
		}

		case LINK_REPLY:
		{
			/*
			 * Immediate answer to a received probe.
			 */
			return NET_TX_PRIORITY;
		}

		case LINK_PROBE:
		{
			/*
			 * Periodic test traffic -> periodically reschedule link test requests.
			 */
			return NET_TX_PERIODIC;
		}

		case DEPLOY_REQ:
		{
			/*
			 * First request: high priority.
			 * Following requests: retransmissions.
			 */
			if (h_net->net_tx_attempts == 0u)
			{
				return NET_TX_PRIORITY;
			}

			return NET_TX_RETX;
		}

		case DEPLOY_CONFIRM:
		{
			return NET_TX_PRIORITY;
		}

		default:
		{
			return NET_TX_NONE;
		}
	}
}


/*
 * After TX completed, decide next TX priority until DISCOVERY PROCESS
 * has been fully completed.
 *
 * @param network_ctx_t *h_net network context handler.
 *
 */
void network_on_tx_done(
    network_ctx_t *h_net
)
{
	switch (h_net->pending_net_tx)
	{
		case DISC_REQ:
		{
			/*
			 * Keep DISC_REQ pending.
			 *
			 * As long as NODE_DISCOVERING is active,
			 * another periodic DISC_REQ will be required.
			 */
			break;
		}


		case DISC_OFFER:
		{
			/*
			 * OFFER is one-shot.
			 *
			 * If lost, a repeated DISC_REQ with the
			 * same token will schedule another OFFER.
			 */
			h_net->pending_net_tx = NET_PKT_NONE;

			break;
		}


		case DISC_ACCEPT:
		{
			/*
			 * IMPORTANT:
			 *
			 * Keep ACCEPT pending until DISC_CONFIRM
			 * is received.
			 */

			break;
		}


		case DISC_CONFIRM:
		{
			/*
			 * CONFIRM is one-shot.
			 *
			 * If lost, the child will send another ACCEPT,
			 * which will schedule another CONFIRM.
			 */
			h_net->pending_net_tx = NET_PKT_NONE;

			break;
		}


		case LINK_PROBE:
		{
			/*
			 * The transmitted sequence becomes the
			 * outstanding one.
			 */
			h_net->link.pending_seq =
					h_net->link.next_seq;

			/*
			 * Prepare a new sequence number for the
			 * following cycle.
			 */
			h_net->link.next_seq++;

			/*
			 * Wait for the matching LINK_REPLY.
			 */
			h_net->link.waiting_reply = true;

			/*
			 * No packet is immediately pending now.
			 *
			 * The next periodic TX event will prepare
			 * the following LINK_PROBE.
			 */
			h_net->pending_net_tx =
					NET_PKT_NONE;

			h_net->net_tx_attempts = 0u;

			break;
		}


		case LINK_REPLY:
		{
			/*
			 * One-shot packet.
			 *
			 * A lost reply will simply produce a failed
			 * attempt on the child side.
			 */
			h_net->pending_net_tx =
					NET_PKT_NONE;

			h_net->net_tx_attempts = 0u;

			break;
		}


		case DEPLOY_REQ:
		{
			/*
			 * Keep DEPLOY_REQ pending until the
			 * corresponding DEPLOY_CONFIRM arrives.
			 */
			if (h_net->net_tx_attempts < UINT8_MAX)
			{
				h_net->net_tx_attempts++;
			}

			break;
		}


		case DEPLOY_CONFIRM:
		{
			/*
			 * One-shot response.
			 *
			 * If lost, a duplicated DEPLOY_REQ will
			 * schedule another DEPLOY_CONFIRM.
			 */
			h_net->pending_net_tx =
					NET_PKT_NONE;

			h_net->net_tx_attempts =
					0u;

			break;
		}


		default:

			break;
	}
}





