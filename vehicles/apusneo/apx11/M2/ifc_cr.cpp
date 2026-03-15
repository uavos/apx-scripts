#include <apx.h>

constexpr const uint8_t TASK_MAIN_MS{200};

//get
using m_s1 = Mandala<mandala::sns::env::scr::s1>; //adc lv3 adsb_tx
using m_s2 = Mandala<mandala::sns::env::scr::s2>; //humid

//set
using m_temp_adsb_tx = Mandala<mandala::est::env::usrc::c5>;
using m_temp_ifc_cr = Mandala<mandala::est::env::usrc::c6>;

int main()
{
    m_s1();
    m_s2();

    schedule_periodic(task("on_main"), TASK_MAIN_MS);

    return 0;
}

EXPORT void on_main()
{
    m_temp_adsb_tx::publish(m_s1::value() + 100);
    m_temp_ifc_cr::publish(m_s2::value() + 100);
}
