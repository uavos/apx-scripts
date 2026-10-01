// Altitude dependent minimum throttle.
//
// On descent the autopilot (TECS) drops throttle down to shiva.eng.thr.min.
// At high altitude the turbine must not idle that low, so this script holds
// the throttle on an altitude dependent floor while there is excess energy
// (the aircraft is above the commanded altitude or faster than commanded).
//
// SIM / X-Plane variant: the floor is applied through throttle override
// (cmd.eng.ovr + cmd.rc.thr), so the result is visible on ctr.eng.thr.
//
// Debug outputs:
//   est.usr.u1 - current floor [%]
//   est.usr.u2 - 1 while the script holds the throttle

#include <apx.h>

//#define TEST_TABLE //compressed altitudes for short sim runs

static constexpr const uint8_t TABLE_SIZE{4};

#ifdef TEST_TABLE
const float TABLE_ALT[TABLE_SIZE]{0.f, 1000.f, 2000.f, 3000.f}; //[m]
#else
const float TABLE_ALT[TABLE_SIZE]{0.f, 3000.f, 5000.f, 8000.f}; //[m]
#endif
const float TABLE_THR[TABLE_SIZE]{0.1f, 0.15f, 0.2f, 0.25f}; //[0..1]

const uint16_t TASK_MS{100}; //msec

const float THR_EPS{0.01f}; //throttle below floor by this much counts as "below"

const float ALT_ON{60.f};  //[m] engage only when this much above commanded altitude
const float ALT_OFF{25.f}; //[m] release when closer than this to commanded altitude
const float SPD_ON{5.f};   //[m/s] ...or this much faster than commanded airspeed
const float SPD_OFF{2.f};  //[m/s]
const float SPD_LOW{3.f};  //[m/s] always release when this much slower than commanded

const uint32_t ENGAGE_MS{1000}; //throttle must stay below floor this long
const uint32_t REARM_MS{5000};  //pause after release before next engage

bool hold{false};
uint32_t time_below{0};
bool below{false};
uint32_t time_release{0};

using m_thr = Mandala<mandala::ctr::nav::eng::thr>;
using m_ovr = Mandala<mandala::cmd::nav::eng::ovr>;
using m_cut = Mandala<mandala::cmd::nav::eng::cut>;
using m_rc_thr = Mandala<mandala::cmd::nav::rc::thr>;

using m_pwr_eng = Mandala<mandala::ctr::env::pwr::eng>;
using m_mode = Mandala<mandala::cmd::nav::proc::mode>;

using m_altitude = Mandala<mandala::est::nav::pos::altitude>;
using m_airspeed = Mandala<mandala::est::nav::air::airspeed>;
using m_cmd_altitude = Mandala<mandala::cmd::nav::pos::altitude>;
using m_cmd_airspeed = Mandala<mandala::cmd::nav::pos::airspeed>;

using m_dbg_floor = Mandala<mandala::est::env::usr::u1>;
using m_dbg_hold = Mandala<mandala::est::env::usr::u2>;

int main()
{
    m_thr();
    m_ovr();
    m_cut();
    m_rc_thr();

    m_pwr_eng();
    m_mode();

    m_altitude();
    m_airspeed();
    m_cmd_altitude();
    m_cmd_airspeed();

    m_dbg_floor();
    m_dbg_hold();

    schedule_periodic(task("on_thr"), TASK_MS);

    printf("VM:thr floor script\n");

    return 0;
}

//linear interpolation over the table, clamped at both ends
float thr_floor(float altitude)
{
    if (altitude <= TABLE_ALT[0]) {
        return TABLE_THR[0];
    }

    for (uint8_t i = 1; i < TABLE_SIZE; i++) {
        if (altitude < TABLE_ALT[i]) {
            const float k = (altitude - TABLE_ALT[i - 1]) / (TABLE_ALT[i] - TABLE_ALT[i - 1]);
            return TABLE_THR[i - 1] + k * (TABLE_THR[i] - TABLE_THR[i - 1]);
        }
    }

    return TABLE_THR[TABLE_SIZE - 1];
}

void release(uint32_t now)
{
    m_ovr::publish(false);
    hold = false;
    below = false;
    time_release = now;
    m_dbg_hold::publish(0.f);
}

EXPORT void on_thr()
{
    const uint32_t now = time_ms();

    const float altitude = m_altitude::value();
    const float floor_thr = thr_floor(altitude);

    m_dbg_floor::publish(floor_thr * 100.f);

    //work only in automatic flight modes with the engine running and no throttle cut
    const uint32_t mode = (uint32_t) m_mode::value();
    const bool auto_mode = (mode == mandala::proc_mode_UAV) || (mode == mandala::proc_mode_WPT)
                           || (mode == mandala::proc_mode_STBY);
    const bool allowed = auto_mode && (bool) m_pwr_eng::value() && !(bool) m_cut::value();

    if (!allowed) {
        if (hold) {
            release(now);
        }
        below = false;
        return;
    }

    const float alt_err = altitude - m_cmd_altitude::value();            //>0: above target
    const float spd_err = m_airspeed::value() - m_cmd_airspeed::value(); //>0: faster than target

    if (hold) {
        const bool energy_ok = (alt_err < ALT_OFF)
                               && (spd_err < SPD_OFF); //descent/deceleration done
        const bool too_slow = spd_err < -SPD_LOW;      //give throttle back to TECS

        if (energy_ok || too_slow) {
            release(now);
            return;
        }

        m_rc_thr::publish(floor_thr); //follow the floor while altitude changes
        return;
    }

    //not holding

    if ((bool) m_ovr::value()) { //override is set by operator, do not interfere
        below = false;
        return;
    }

    const bool excess = (alt_err > ALT_ON) || (spd_err > SPD_ON);

    if (!excess || m_thr::value() >= floor_thr - THR_EPS) {
        below = false;
        return;
    }

    if (!below) {
        below = true;
        time_below = now;
        return;
    }

    if ((now - time_below < ENGAGE_MS) || (now - time_release < REARM_MS)) {
        return;
    }

    //engage: throttle value first, then override
    m_rc_thr::publish(floor_thr);
    m_ovr::publish(true);
    hold = true;
    m_dbg_hold::publish(1.f);
}
