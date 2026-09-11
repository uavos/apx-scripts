//v1.1 photo/camera release counter script
#include <apx.h>

constexpr const uint8_t TASK_MAIN_MS{10};
constexpr const uint16_t CHECK_DELAY_MS{100};

//inputs
using CAM_RELEASE = Mandala<mandala::ctr::env::cam::shot>;
using MC_BIT0 = Mandala<mandala::est::env::usrb::b2>;
using MC_BIT1 = Mandala<mandala::est::env::usrb::b3>;

//output
using MC_RESET = Mandala<mandala::est::env::usrb::b4>;

//telemetry
using M_RELEASE_COUNTER = Mandala<mandala::est::env::usrw::w1>;
using M_MC_COUNTER = Mandala<mandala::est::env::usrw::w2>;
using M_ERROR_COUNTER = Mandala<mandala::est::env::usrw::w3>;
using M_COMMANDS_SENT = Mandala<mandala::est::env::usrw::w4>;
using M_MC_TOTAL = Mandala<mandala::est::env::usrw::w5>;

enum class State {
    idle,
    wait_check,
};

uint32_t RELEASE_COUNTER{};
uint32_t error_counter{};
uint8_t mc_counter{};
uint32_t mc_total{};

uint32_t m_camReleaseOld{mandala::cam_shot_off};

State m_state{State::idle};
uint32_t m_stateTime{};

int main()
{
    CAM_RELEASE();
    MC_BIT0();
    MC_BIT1();
    MC_RESET();

    M_RELEASE_COUNTER();
    M_MC_COUNTER();
    M_ERROR_COUNTER();
    M_COMMANDS_SENT();
    M_MC_TOTAL();

    M_RELEASE_COUNTER::publish((uint32_t) 0);
    M_ERROR_COUNTER::publish((uint32_t) 0);
    M_COMMANDS_SENT::publish((uint32_t) 0);
    M_MC_TOTAL::publish((uint32_t) 0);

    MC_RESET::publish(true);
    MC_RESET::publish(false);

    schedule_periodic(task("on_main"), TASK_MAIN_MS);

    printf("photo script ready...\n");

    return 0;
}

EXPORT void on_main()
{
    uint32_t now = time_ms();

    uint32_t camRelease = (uint32_t) CAM_RELEASE::value();
    if (camRelease == mandala::cam_shot_single && m_camReleaseOld == mandala::cam_shot_off) {
        RELEASE_COUNTER++;
        M_RELEASE_COUNTER::publish((uint32_t) RELEASE_COUNTER);

        if (m_state == State::idle) {
            m_state = State::wait_check;
            m_stateTime = now;
        }
    }
    m_camReleaseOld = camRelease;

    switch (m_state) {
    case State::idle:
        break;

    case State::wait_check:
        if (now - m_stateTime >= CHECK_DELAY_MS) {
            bool mcB2 = (bool) MC_BIT0::value();
            bool mcB3 = (bool) MC_BIT1::value();
            mc_counter = (uint8_t) ((mcB2 ? 1 : 0) | (mcB3 ? 2 : 0));
            M_MC_COUNTER::publish((uint32_t) mc_counter);

            mc_total += mc_counter;
            M_MC_TOTAL::publish((uint32_t) mc_total);

            if (mc_counter != 1) {
                error_counter += (mc_counter == 0) ? 1 : (mc_counter - 1);
                M_ERROR_COUNTER::publish((uint32_t) error_counter);
            }

            MC_RESET::publish(true);
            MC_RESET::publish(false);

            m_state = State::idle;
        }
        break;
    }
}
