#include <apx.h>

constexpr const port_id_t PORT_ID_TLM_SYNC{42};

constexpr const uint16_t TASK_DELAY_MS{10}; //100Hz

constexpr const uint8_t NODE_CNT{2};
constexpr const uint8_t MSG_CNT{6};

constexpr const uint8_t NODE_ID[NODE_CNT] = {3, 4};
constexpr const uint8_t MSG_ID[MSG_CNT] = {0, 3, 1, 4, 2, 5};

uint8_t idx_node{0};
uint8_t idx_msg{0};

constexpr const uint16_t SCHEDULE_SYNC_TLM_TIMEOUT{100};

bool on_sync_telemetry{true};
uint16_t TLM_DIV{2};

uint32_t schedule_sync_tlm_timer{};

int main()
{
    uint32_t now = time_ms();
    schedule_sync_tlm_timer = now;

    task("on_task", TASK_DELAY_MS); //100 Hz

    task("tlm_off"); //GCS with terminal command `vmexec("tlm_off")`
    task("tlm_l");   //GCS with terminal command `vmexec("tlm_l")`
    task("tlm_n");   //GCS with terminal command `vmexec("tlm_n")`
    task("tlm_f");   //GCS with terminal command `vmexec("tlm_f")`

    printf("COM Script ready...\n");

    return 0;
}

void telemetry_sync()
{
    const uint8_t nidx = NODE_ID[idx_node];
    const uint8_t midx = MSG_ID[idx_msg];

    uint8_t data[2] = {nidx, midx};

    if (++idx_node >= 2) {
        idx_node = 0;
    }

    if (++idx_msg >= 6) {
        idx_msg = 0;
    }

    send(PORT_ID_TLM_SYNC, data, 2, true);
}

EXPORT void on_task()
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
    printf("Telemetry sync off...\n");
}

EXPORT void tlm_l()
{
    on_sync_telemetry = true;
    TLM_DIV = 5;
    uint16_t timeout = SCHEDULE_SYNC_TLM_TIMEOUT * TLM_DIV;
    printf("Telemetry sync low mode:%u...\n", timeout);
}

EXPORT void tlm_n()
{
    on_sync_telemetry = true;
    TLM_DIV = 2;
    uint16_t timeout = SCHEDULE_SYNC_TLM_TIMEOUT * TLM_DIV;
    printf("Telemetry sync normal mode:%u...\n", timeout);
}

EXPORT void tlm_f()
{
    on_sync_telemetry = true;
    TLM_DIV = 1;
    uint16_t timeout = SCHEDULE_SYNC_TLM_TIMEOUT * TLM_DIV;
    printf("Telemetry sync fast mode:%u...\n", timeout);
}
