#include <apx.h>

constexpr const char *txt_dev = "CL"; //CL

constexpr const uint8_t TASK_MAIN_MS{100};
constexpr const uint16_t TASK_HEATER_MS{2000};

//----------------------Temperature-------------------------
constexpr const uint16_t START_HEATER{10};

using m_sw_manual = Mandala<mandala::ctr::env::sw::sw15>; //heater manual

using m_sw = Mandala<mandala::ctr::env::sw::sw2>;          //heater off/on
using m_sns_temp = Mandala<mandala::sns::nav::gyro::temp>; //gyro temp

//get
using m_s1 = Mandala<mandala::sns::env::scr::s1>;

//set
using m_temp = Mandala<mandala::est::env::usrc::c7>;

int main()
{
    m_s1();

    m_sns_temp();

    m_sw_manual();

    task("on_main", TASK_MAIN_MS);
    task("on_heater", TASK_HEATER_MS);

    printf("NAV:%s Script ready...\n", txt_dev);

    return 0;
}

EXPORT void on_main()
{
    m_temp::publish(m_s1::value() + 100);
}

EXPORT void on_heater()
{
    if (m_sw_manual::value()) {
        return;
    }

    float RT = m_sns_temp::value();

    if (RT < START_HEATER) {
        m_sw::publish(1u);
    }

    if (RT > START_HEATER * 1.5f) {
        m_sw::publish(0u);
    }
}
