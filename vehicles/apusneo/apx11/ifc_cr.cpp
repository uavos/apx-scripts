#include <apx.h>

constexpr const uint8_t TASK_MAIN_MS{100};

//get
using m_s1 = Mandala<mandala::sns::env::scr::s1>;

//set
using m_temp = Mandala<mandala::est::env::usrc::c8>;

int main()
{
    m_s1();

    schedule_periodic(task("on_main"), TASK_MAIN_MS);

    return 0;
}

EXPORT void on_main()
{
    m_temp::publish(m_s1::value() + 100);
}
