// ERS launch on excessive roll or pitch

#include <apx.h>

using m_roll = Mandala<mandala::est::nav::att::roll>;
using m_pitch = Mandala<mandala::est::nav::att::pitch>;
using m_att_valid = Mandala<mandala::est::nav::att::valid>;
using m_mode = Mandala<mandala::cmd::nav::proc::mode>;
using m_ers = Mandala<mandala::ctr::env::ers::launch>;
using m_eng_cut = Mandala<mandala::cmd::nav::eng::cut>;

static constexpr const uint32_t TASK_MS{20};
static constexpr const float ERS_ROLL{85.f};
static constexpr const float ERS_PITCH{85.f};
static constexpr const uint32_t ERS_HOLD_MS{200};
static uint32_t g_ok_time{};
static bool g_done{};

int main()
{
    g_done = false;

    m_roll();
    m_pitch();
    m_att_valid();
    m_mode();
    m_ers();
    m_eng_cut();

    g_ok_time = time_ms();

    schedule_periodic(task("on_task"), TASK_MS);

    puts("VM: ERS initialized");

    return 0;
}

EXPORT void on_task()
{
    if (g_done)
        return;

    const uint32_t now = time_ms();
    const uint32_t mode = (uint32_t)m_mode::value();

    // Block prohibited modes
    if (mode == mandala::proc_mode_EMG
        || mode == mandala::proc_mode_TAKEOFF
        || mode == mandala::proc_mode_TAXI) {

        g_ok_time = now;
        return;
    }

    // Check attitude validity
    if (!(bool)m_att_valid::value()) {
        g_ok_time = now;
        return;
    }

    // Throttle cut must be OFF
    if ((bool)m_eng_cut::value()) {
        g_ok_time = now;
        return;
    }

    const float roll = m_roll::value() * R2D;
    const float pitch = m_pitch::value() * R2D;

    // Check roll and pitch
    if (fabs(roll) <= ERS_ROLL
        && fabs(pitch) <= ERS_PITCH) {

        g_ok_time = now;
        return;
    }

    if (now - g_ok_time < ERS_HOLD_MS)
        return;

    g_done = true;

    m_eng_cut::publish(true);
    m_ers::publish(true);

    puts("VM: ERS ON");
    printf("VM: Roll %.1f\n", roll);
    printf("VM: Pitch %.1f\n", pitch);
}
