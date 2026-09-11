//v1.1 photo/camera shot trigger script
#include <apx.h>

constexpr const uint8_t TASK_MAIN_MS{10};
constexpr const uint16_t SHOT_INTERVAL_MS{1000};
constexpr const uint16_t SHOT_PULSE_MS{50};

using CAM_RELEASE = Mandala<mandala::ctr::env::cam::shot>;
using M_SHOTS_SENT = Mandala<mandala::est::env::usrw::w4>;

enum class State {
    wait_trigger,
    pulse_active,
};

uint32_t shots_sent{};

State m_state{State::wait_trigger};
uint32_t m_stateTime{};

int main()
{
    CAM_RELEASE();
    M_SHOTS_SENT();

    CAM_RELEASE::publish((uint32_t) mandala::cam_shot_off);

    m_stateTime = time_ms();

    schedule_periodic(task("on_main"), TASK_MAIN_MS);

    printf("nav script ready...\n");

    return 0;
}

EXPORT void on_main()
{
    uint32_t now = time_ms();

    switch (m_state) {
    case State::wait_trigger:
        if (now - m_stateTime >= SHOT_INTERVAL_MS) {
            CAM_RELEASE::publish((uint32_t) mandala::cam_shot_single);

            shots_sent++;
            M_SHOTS_SENT::publish((uint32_t) shots_sent);

            m_state = State::pulse_active;
            m_stateTime = now;
        }
        break;

    case State::pulse_active:
        if (now - m_stateTime >= SHOT_PULSE_MS) {
            CAM_RELEASE::publish((uint32_t) mandala::cam_shot_off);

            m_state = State::wait_trigger;
            m_stateTime = now;
        }
        break;
    }
}
