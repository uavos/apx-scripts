// ERS launch on excessive roll
// Sets ERS launch and throttle cut when |roll| exceeds ERS_ROLL for longer than ERS_HOLD_MS, not in EMG mode

#include <apx.h>

using m_roll = Mandala<mandala::est::nav::att::roll>;
using m_att_valid = Mandala<mandala::est::nav::att::valid>;
using m_mode = Mandala<mandala::cmd::nav::proc::mode>;
using m_ers = Mandala<mandala::ctr::env::ers::launch>;
using m_eng_cut = Mandala<mandala::cmd::nav::eng::cut>;

static constexpr const uint32_t TASK_MS{20};      // [ms] check period
static constexpr const float ERS_ROLL{90.f};      // [deg] roll limit, both sides
static constexpr const uint32_t ERS_HOLD_MS{200}; // [ms] the roll must stay over the limit, 0 for instant

static uint32_t g_roll_ok_time{}; // last time the roll was within the limit
static bool g_done{};             // ERS launched, the script does nothing more

int main()
{
    m_roll(); // subscribe
    m_att_valid();
    m_mode();
    m_ers();
    m_eng_cut();

    g_roll_ok_time = time_ms();
    schedule_periodic(task("on_task"), TASK_MS);
    return 0;
}

EXPORT void on_task()
{
    if (g_done)
        return;

    const uint32_t now = time_ms();

    // EMG is the mode after power on, the attitude is not estimated yet
    if ((uint32_t) m_mode::value() == mandala::proc_mode_EMG || !(bool) m_att_valid::value()) {
        g_roll_ok_time = now;
        return;
    }

    const float roll = fabs(m_roll::value() * R2D);
    if (roll <= ERS_ROLL) {
        g_roll_ok_time = now;
        return;
    }

    if (now - g_roll_ok_time < ERS_HOLD_MS)
        return;

    g_done = true;
    m_eng_cut::publish(true);
    m_ers::publish(true);
    printf("VM:ERS on, roll %.0f\n", roll);
}
