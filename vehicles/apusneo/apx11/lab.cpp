#include <apx.h>

constexpr const port_id_t PORT_ID_AGL{33};

// Mandala interfaces for power and sensors
using m_power_agl = Mandala<mandala::ctr::env::pwr::agl>;
using m_agl_laser = Mandala<mandala::sns::nav::agl::laser>;

// Landing command and navigation mode/stage/action
using m_landing_command = Mandala<mandala::ctr::env::sw::sw1>;
using m_mode = Mandala<mandala::cmd::nav::proc::mode>;
using m_stage = Mandala<mandala::cmd::nav::proc::stage>;
using m_action = Mandala<mandala::cmd::nav::proc::action>;

// Switches and control registers for taxi/steering
using m_sw3 = Mandala<mandala::ctr::env::sw::sw3>;
using m_reg_str = Mandala<mandala::cmd::nav::reg::str>;
using m_reg_taxi = Mandala<mandala::cmd::nav::reg::taxi>;
using m_ctr_rud = Mandala<mandala::ctr::nav::str::rud>;

// System health and time tracking
using m_ltt = Mandala<mandala::est::env::sys::ltt>;
using m_health = Mandala<mandala::est::env::sys::health>;

using m_fts = Mandala<mandala::est::env::usrb::b3>;
using m_squawk = Mandala<mandala::est::env::usrw::w5>;

using m_pwr_satcom = Mandala<mandala::ctr::env::pwr::satcom>;

bool squawk_emergency = false; //true while 7700/7600 is being published

constexpr const uint8_t TASK_MAIN_MS{100};    //10Hz
constexpr const uint8_t TASK_LANDING_MS{200}; //5Hz

constexpr const uint32_t STAGE_FINAL{3};

bool landing{false};                  // landing command accepted, mode is monitored
uint32_t stage_expected{STAGE_FINAL}; // stage expected after our last action
bool stage_reset{false};              // reported stage may be stale after switch from other mode

int main()
{
    m_power_agl();
    m_agl_laser();

    m_landing_command();
    m_mode();
    m_stage();

    m_reg_str();
    m_reg_taxi();
    m_ctr_rud();

    m_ltt();
    m_health();
    m_fts();

    m_sw3("on_sw3");

    schedule_periodic(task("on_main"), TASK_MAIN_MS);
    schedule_periodic(task("on_landing"), TASK_LANDING_MS);

    receive(PORT_ID_AGL, "agl_handler");

    printf("IFC:Script ready...\n");

    return 0;
}

EXPORT void on_main()
{
    if ((uint32_t) m_ltt::value() < 10) {
        m_health::publish((uint32_t) mandala::sys_health_normal);
    }

    if ((bool) m_fts::value()) {
        m_squawk::publish(7700u); //7700 - Emergency (FTS activated)
        squawk_emergency = true;
    } else if ((uint32_t) m_health::value() == mandala::sys_health_warning) {
        m_squawk::publish(7600u); //7600 - lost link (system health warning)
        squawk_emergency = true;
    } else if (squawk_emergency) {
        m_squawk::publish(0u); //emergency cleared - reset squawk once
        squawk_emergency = false;
    }

    if ((uint32_t) m_health::value() == mandala::sys_health_warning
        && (uint32_t) m_mode::value() != mandala::proc_mode_TAXI) {
        m_pwr_satcom::publish((uint32_t) mandala::pwr_satcom_on);

        m_mode::publish((uint32_t) mandala::proc_mode_LANDING);
    }

    //agl
    send(PORT_ID_AGL, (const uint8_t *) "?LD\r\n", 5, false);
}

EXPORT void on_sw3()
{
    if (m_sw3::value() > 0.5f) {
        printf("sw3: taxi/steering ON");
    } else {
        m_reg_str::publish((uint32_t) mandala::reg_str_off);
        m_reg_taxi::publish((uint32_t) mandala::reg_taxi_off);
        m_ctr_rud::publish(0.f);
        printf("sw3: taxi/steering OFF, steering reset");
    }
}

EXPORT void on_landing()
{
    bool command = (uint32_t) m_landing_command::value() == 1;

    // start landing sequence on rising edge of the command
    if (command && !landing) {
        bool was_landing = (uint32_t) m_mode::value() == mandala::proc_mode_LANDING;
        m_mode::publish((uint32_t) mandala::proc_mode_LANDING);
        // entering landing mode resets stage to 0 (INIT) in landing proc
        stage_expected = was_landing ? (uint32_t) m_stage::value() : 0;
        stage_reset = !was_landing;
        landing = true;
        return; // first stage step on next cycle, after mode switch
    }

    if (!command) {
        landing = false;
        return;
    }

    // mode was changed from outside - drop the command, wait for next 1
    if ((uint32_t) m_mode::value() != mandala::proc_mode_LANDING) {
        m_landing_command::publish(0u);
        landing = false;
        return;
    }

    // stage sequence finished
    if (stage_expected >= STAGE_FINAL) {
        return;
    }

    uint32_t stage = (uint32_t) m_stage::value();

    // landing proc starts from 0 after mode switch, but does not publish stage if its
    // internal value was already 0 - so stale stage (e.g. set from outside) may still be
    // reported. Count on our own until landing proc reports a real stage.
    if (stage_reset) {
        if (stage >= STAGE_FINAL) {
            stage = stage_expected;
        } else {
            stage_reset = false;
        }
    }

    if (stage >= STAGE_FINAL) {
        stage_expected = STAGE_FINAL;
        return;
    }

    // stage was changed by different code (or our action was rejected) - just follow it
    if (stage != stage_expected) {
        stage_expected = stage;
        return;
    }

    // stage is owned by landing proc - request next stage instead of writing it
    stage_expected = stage + 1;
    m_action::publish((uint32_t) mandala::proc_action_next);
}

//ASCII:ld,0:-0.66
//HEX:6c 64 2c 30 3a 2d 30 2e 36 36 20 0d 0a
EXPORT void agl_handler(const uint8_t *data, size_t size)
{
    if ((uint32_t) m_power_agl::value() == mandala::pwr_agl_off) {
        return;
    }

    for (uint32_t i = 0; i < size; i++) {
        if (data[i] == ':') {
            int sign = 1;
            float result = 0.0f;
            float divisor = 10.0f;
            bool decimal = false;
            i++;
            if (data[i] == '-') {
                sign = -1;
                i++;
            }
            for (; i < size; i++) {
                if (data[i] >= '0' && data[i] <= '9') {
                    if (decimal) {
                        result += (data[i] - '0') / divisor;
                        divisor *= 10.0f;
                    } else {
                        result = result * 10.0f + (data[i] - '0');
                    }
                } else if (data[i] == '.') {
                    decimal = true;
                } else {
                    break;
                }
            }
            float agl = (float) sign * result;

            m_agl_laser::publish(agl);
        }
    }
}
