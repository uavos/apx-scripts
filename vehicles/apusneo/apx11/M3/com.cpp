#include <apx.h>

/*------------------------------------------------------------------------
 * Sync packet on PORT_ID_TLM_SYNC: {node, msg, round}  (haps::SYNC_SIZE = 3)
 *
 *   msg == SYNC_MSG_LATCH (0xFF)  - marker: node latches a snapshot of all
 *                                   live data (start of round). Sent once per
 *                                   round as broadcast (node == SYNC_NODE_BCAST)
 *                                   so both IFC nodes freeze data at the same
 *                                   time instant.
 *   otherwise                     - poll: node sends the next message of its
 *                                   own tx sequence from the latched snapshot
 *                                   (msg_1 -> msg_2 -> msg_3|msg_5). The `msg`
 *                                   byte is informational; the node ignores it.
 *   round                         - round counter; node replies to a poll only
 *                                   if it matches the round of its snapshot.
 *
 * One round (7 slots, one packet per slot):
 * LATCH(bcast) | L:0 | R:3 | L:1 | R:4 | L:2 | R:5
 *------------------------------------------------------------------------*/

constexpr const port_id_t PORT_ID_TLM_SYNC{6};

constexpr const uint16_t TASK_MAIN_MS{10}; //100Hz

constexpr const uint8_t SYNC_NODE_BCAST{0xFF};
constexpr const uint8_t SYNC_MSG_LATCH{0xFF};
constexpr const uint8_t SYNC_SIZE{3};

constexpr const uint8_t NODE_IFC_L{3};
constexpr const uint8_t NODE_IFC_R{4};

constexpr const uint8_t NODE_CNT{2};
constexpr const uint8_t NODE_ID[NODE_CNT] = {NODE_IFC_L, NODE_IFC_R};

//poll sequence per round, paired with alternating nodes: L:0 R:3 L:1 R:4 L:2 R:5
//(node tx order is its own: msg_1, msg_2, msg_3|msg_5 - 3 polls per node)
constexpr const uint8_t MSG_CNT{6};
constexpr const uint8_t MSG_ID[MSG_CNT] = {0, 3, 1, 4, 2, 5};

//round: 1 latch slot + MSG_CNT poll slots
constexpr const uint8_t SLOT_CNT{1 + MSG_CNT};

uint8_t idx_slot{0};
uint8_t sync_round{0};

constexpr const uint16_t SCHEDULE_SYNC_TLM_TIMEOUT{100}; //ms

bool on_sync_telemetry{true};
uint16_t TLM_DIV{2};

uint32_t schedule_sync_tlm_timer{};

int main()
{
    uint32_t now = time_ms();
    schedule_sync_tlm_timer = now;

    schedule_periodic(task("on_main"), TASK_MAIN_MS);

    task("tlm_off"); // vmexec("tlm_off")
    task("tlm_l");   // vmexec("tlm_l")
    task("tlm_n");   // vmexec("tlm_n")
    task("tlm_f");   // vmexec("tlm_f")

    printf("COM Script ready...\n");

    return 0;
}

static void sync_send(uint8_t node, uint8_t msg)
{
    uint8_t data[SYNC_SIZE] = {node, msg, sync_round};
    send(PORT_ID_TLM_SYNC, data, SYNC_SIZE, true);
}

void telemetry_sync()
{
    if (idx_slot == 0) {
        //start of round: new round id, latch snapshot on all nodes
        sync_round++;
        sync_send(SYNC_NODE_BCAST, SYNC_MSG_LATCH);
    } else {
        //poll slots alternate nodes: L:0 R:3 L:1 R:4 L:2 R:5
        const uint8_t i = idx_slot - 1;
        const uint8_t node = NODE_ID[i % NODE_CNT];
        const uint8_t msg = MSG_ID[i];
        sync_send(node, msg);
    }

    if (++idx_slot >= SLOT_CNT) {
        idx_slot = 0;
    }
}

static void sync_restart()
{
    //next tick starts a fresh round with a latch marker
    idx_slot = 0;
    schedule_sync_tlm_timer = time_ms();
}

EXPORT void on_main()
{
    uint32_t now = time_ms();
    if (on_sync_telemetry && now - schedule_sync_tlm_timer > SCHEDULE_SYNC_TLM_TIMEOUT * TLM_DIV) {
        schedule_sync_tlm_timer = now;
        telemetry_sync();
    }
}

EXPORT void tlm_off()
{
    on_sync_telemetry = false;
    TLM_DIV = 2;
    sync_restart();
    printf("Telemetry sync off...\n");
}

EXPORT void tlm_l()
{
    on_sync_telemetry = true;
    TLM_DIV = 5;
    sync_restart();
    uint32_t round_ms = SCHEDULE_SYNC_TLM_TIMEOUT * TLM_DIV * SLOT_CNT;
    printf("Telemetry sync low mode, round:%u ms...\n", round_ms);
}

EXPORT void tlm_n()
{
    on_sync_telemetry = true;
    TLM_DIV = 2;
    sync_restart();
    uint32_t round_ms = SCHEDULE_SYNC_TLM_TIMEOUT * TLM_DIV * SLOT_CNT;
    printf("Telemetry sync normal mode, round:%u ms...\n", round_ms);
}

EXPORT void tlm_f()
{
    on_sync_telemetry = true;
    TLM_DIV = 1;
    sync_restart();
    uint32_t round_ms = SCHEDULE_SYNC_TLM_TIMEOUT * TLM_DIV * SLOT_CNT;
    printf("Telemetry sync fast mode, round:%u ms...\n", round_ms);
}
