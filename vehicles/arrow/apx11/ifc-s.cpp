#include <apx.h>

using sw12 = Mandala<mandala::ctr::env::sw::sw12>;
using sw13 = Mandala<mandala::ctr::env::sw::sw13>;

using mode = Mandala<mandala::cmd::nav::proc::mode>;
using b9 = Mandala<mandala::est::env::usrb::b9>;

void enable_radio()
{
    sw12::publish(true);

    sleep(1000);

    sw13::publish(true);
}

void disable_radio()
{
    sw12::publish(false);
    sw13::publish(false);
}

EXPORT void rs_radio()
{
    disable_radio();
    sleep(3000);
    enable_radio();
}

EXPORT void on_mode()
{
    auto value = mode::value();
    if (value == (uint32_t) mandala::proc_mode_TAXI) {
        b9::publish(false);
    } else {
        b9::publish(true);
    }
}

int main()
{
    sleep(1000);
    enable_radio();

    task("rs_radio");
    mode("on_mode");

    return 0;
}
