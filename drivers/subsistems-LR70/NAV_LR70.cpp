#include <apx.h>
//RT
using m_gyro_temp = Mandala<mandala::sns::nav::gyro::temp>;
using m_rt = Mandala<mandala::sns::env::aux::rt>;

//RPM
const float PART_ERR{0.04f};
const uint8_t K_DV{8};
const uint8_t TRY_NUM{25};

bool err1{};
bool err2{};
bool err3{};
bool stat_rpm1{};
bool stat_rpm2{};
bool stat_rpm3{};

uint16_t err1_cnt{};
uint16_t err2_cnt{};
uint16_t err3_cnt{};

float rpm1{};
float rpm2{};
float rpm3{};
float rpm1_prev{};
float rpm2_prev{};
float rpm3_prev{};

float rpm{};

float err_rpm1{};
float err_rpm2{};
float err_rpm3{};

uint8_t act_sns{1};

using m_rpm_front = Mandala<mandala::est::env::usrw::w9>;
using m_rpm_near = Mandala<mandala::est::env::usrw::w10>;
using m_rpm_ecu = Mandala<mandala::est::env::usrw::w11>;
using m_act_rpm = Mandala<mandala::est::env::usrc::c7>;

using m_sns_rpm = Mandala<mandala::sns::env::eng::rpm>;
using m_sw_ecu_rpm = Mandala<mandala::ctr::env::sw::sw4>;

//UVHPD
using m_mode = Mandala<mandala::cmd::nav::proc::mode>;
using m_on_hold = Mandala<mandala::est::env::usrb::b2>;

//RC mode
using m_rc_mode = Mandala<mandala::cmd::nav::rc::mode>;

using m_t2 = Mandala<mandala::ctr::env::tune::t2>; //prop

using m_rc_prop = Mandala<mandala::cmd::nav::rc::prop>;   //rc_prop
using m_rc_roll = Mandala<mandala::cmd::nav::rc::roll>;   //rc_roll
using m_rc_pitch = Mandala<mandala::cmd::nav::rc::pitch>; //rc_pitch
using m_rc_yaw = Mandala<mandala::cmd::nav::rc::yaw>;     //rc_yaw

enum {
    proc_mode_EMG = 0,
    proc_mode_RPV = 1,
    proc_mode_UAV = 2,
    proc_mode_WPT = 3,
    proc_mode_STBY = 4,
    proc_mode_TAXI = 5,
    proc_mode_TAKEOFF = 6,
    proc_mode_LANDING = 7,
};

int main()
{
    m_rpm_front();
    m_rpm_near();
    m_rpm_ecu();
    m_sw_ecu_rpm();

    m_mode();

    m_rc_mode();
    m_t2();

    m_gyro_temp("on_gyro_temp"); // subscribe `on changed` event
    task("on_rpm", 25);          // 40 Hz
    task("on_mode", 1000);       // 1 Hz

    task("on_rc_mode", 10); // 100 Hz

    return 0;
}

EXPORT void on_mode()
{
    if ((uint8_t) m_mode::value() == proc_mode_TAXI) {
        m_on_hold::publish(0u);
    } else {
        m_on_hold::publish(1u);
    }
}

EXPORT void on_gyro_temp()
{
    m_rt::publish(m_gyro_temp::value());
}

EXPORT void on_rpm()
{
    rpm1 = m_rpm_front::value();
    rpm2 = m_rpm_near::value();
    rpm3 = m_rpm_ecu::value() / K_DV;

    if (act_sns < 1 || act_sns > 3) {
        act_sns = 1;
    }

    rpm1_prev == rpm1 ? err1_cnt++ : err1_cnt = 0;
    rpm2_prev == rpm2 ? err2_cnt++ : err2_cnt = 0;
    rpm3_prev == rpm3 ? err3_cnt++ : err3_cnt = 0;

    err1 = err1_cnt >= TRY_NUM;
    err2 = err2_cnt >= TRY_NUM;
    err3 = err3_cnt >= TRY_NUM;

    rpm1_prev = rpm1;
    rpm2_prev = rpm2;
    rpm3_prev = rpm3;

    err_rpm1 = rpm1 * PART_ERR;
    err_rpm2 = rpm2 * PART_ERR;
    err_rpm3 = rpm3 * PART_ERR;

    if (err1 || err2 || err3) {
        if (!err3)
            act_sns = 3;
        if (!err2)
            act_sns = 2;
        if (!err1)
            act_sns = 1;
    } else {
        stat_rpm3 = ((rpm1 <= rpm2 + err_rpm2 && rpm1 >= rpm2 - err_rpm2)
                     && (rpm3 > rpm1 + err_rpm1 || rpm3 < rpm1 - err_rpm1));
        stat_rpm2 = ((rpm1 <= rpm3 + err_rpm3 && rpm1 >= rpm3 - err_rpm3)
                     && (rpm2 > rpm1 + err_rpm1 || rpm2 < rpm1 - err_rpm1));
        stat_rpm1 = ((rpm2 <= rpm3 + err_rpm3 && rpm2 >= rpm3 - err_rpm3)
                     && (rpm1 > rpm2 + err_rpm2 || rpm1 < rpm2 - err_rpm2));

        // L sensor failure
        if (stat_rpm1 && !stat_rpm2 && act_sns == 1)
            act_sns = 2; // R sensor active
        if (stat_rpm1 && !stat_rpm3 && act_sns == 1)
            act_sns = 3; // E sensor active

        // R sensor failure
        if (stat_rpm2 && !stat_rpm1 && act_sns == 2)
            act_sns = 1; // L sensor active
        if (stat_rpm2 && !stat_rpm3 && act_sns == 2)
            act_sns = 3; // E sensor active

        // E sensor failure
        if (stat_rpm3 && !stat_rpm1 && act_sns == 3)
            act_sns = 1; // L sensor active
        if (stat_rpm3 && !stat_rpm2 && act_sns == 3)
            act_sns = 2; // E sensor active
    }

    if ((bool) m_sw_ecu_rpm::value()) {
        act_sns = 3;
    }

    if (act_sns == 1)
        rpm = rpm1;
    if (act_sns == 2)
        rpm = rpm2;
    if (act_sns == 3)
        rpm = rpm3;

    m_act_rpm::publish((uint32_t) act_sns);
    m_sns_rpm::publish(rpm);
}

EXPORT void on_rc_mode()
{
    if ((uint32_t) m_rc_mode::value() == mandala::rc_mode_manual) {
        m_rc_prop::publish(m_t2::value() * (-1.f));
    }
}
