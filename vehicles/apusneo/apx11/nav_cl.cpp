#include <apx.h>

constexpr const char *txt_dev = "CL"; //CL

constexpr const uint16_t TASK_MAIN_MS{1000};
constexpr const uint16_t TASK_HEATER_MS{2000};

//----------------------Temperature-------------------------
constexpr const uint16_t START_HEATER{10};

using m_sw_manual = Mandala<mandala::ctr::env::sw::sw15>; //heater manual
using m_status_heater = Mandala<mandala::est::env::usrc::c15>;

using m_sw1 = Mandala<mandala::ctr::env::sw::sw1>;
using m_sw2 = Mandala<mandala::ctr::env::sw::sw2>; //heater for main nav off/on
using m_sw3 = Mandala<mandala::ctr::env::sw::sw3>;
using m_sw4 = Mandala<mandala::ctr::env::sw::sw4>;
using m_sw5 = Mandala<mandala::ctr::env::sw::sw5>;
using m_sw6 = Mandala<mandala::ctr::env::sw::sw6>;
using m_sw7 = Mandala<mandala::ctr::env::sw::sw7>;

using m_sns_temp = Mandala<mandala::sns::nav::gyro::temp>; //gyro temp

//get
using m_s1 = Mandala<mandala::sns::env::scr::s1>;

//set
using m_temp = Mandala<mandala::est::env::usrc::c7>;

using m_ltt = Mandala<mandala::est::env::sys::ltt>;
using m_health = Mandala<mandala::est::env::sys::health>;

using m_thr_cut = Mandala<mandala::cmd::nav::eng::cut>;
using m_mode = Mandala<mandala::cmd::nav::proc::mode>;

using m_pwr_satcom = Mandala<mandala::ctr::env::pwr::satcom>;

int main()
{
    //subscribe
    m_sw1();
    m_sw2();
    m_sw3();
    m_sw4();
    m_sw5();
    m_sw6();
    m_sw7();

    m_status_heater();

    m_s1();

    m_sns_temp();

    m_sw_manual();

    m_ltt();
    m_health();

    task("on_main", TASK_MAIN_MS);
    task("on_heater", TASK_HEATER_MS);

    printf("NAV:%s Script ready...\n", txt_dev);

    return 0;
}

EXPORT void on_main()
{
    m_temp::publish(m_s1::value() + 100);

    uint32_t status = ((uint32_t) m_sw1::value() << 0) | ((uint32_t) m_sw2::value() << 1)
                      | ((uint32_t) m_sw3::value() << 2) | ((uint32_t) m_sw4::value() << 3)
                      | ((uint32_t) m_sw5::value() << 4) | ((uint32_t) m_sw6::value() << 5)
                      | ((uint32_t) m_sw7::value() << 6);

    m_status_heater::publish((uint32_t) status);

    if ((uint32_t) m_ltt::value() < 10) {
        m_health::publish((uint32_t) mandala::sys_health_normal);
    }

    if ((uint32_t) m_health::value() == mandala::sys_health_warning) {
        m_pwr_satcom::publish((uint32_t) mandala::pwr_satcom_on);

        m_thr_cut::publish((uint32_t) mandala::eng_cut_on);
        m_mode::publish((uint32_t) mandala::proc_mode_LANDING);
    }
}

EXPORT void on_heater()
{
    if ((bool) m_sw_manual::value()) {
        return;
    }

    float RT = m_sns_temp::value();

    if (RT < START_HEATER) {
        m_sw2::publish(1u);
    }

    if (RT > START_HEATER * 1.5f) {
        m_sw2::publish(0u);
    }
}
