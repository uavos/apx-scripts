//v1.0 photo/camera release counter script
#include <apx.h>

//bus mapping (indices are examples, remap as needed):
// est.usrb.b1 <- CAM_RELEASE        camera release signal, high pulse lasts ~50ms
// est.usrb.b2 <- MC_BIT0            counter microchip feedback, bit0
// est.usrb.b3 <- MC_BIT1            counter microchip feedback, bit1
// est.usr.u1  -> M_RELEASE_COUNTER  RELEASE_COUNTER telemetry
// est.usr.u2  -> M_MC_COUNTER       mc_counter telemetry
// est.usr.u3  -> M_ERROR_COUNTER    error_counter telemetry

//microchip counter free-runs 00,01,10,11,00,... and is never reset,
//RELEASE_COUNTER must stay unreset too so it stays in lockstep with it

//errors are counted by comparing, each poll, how many steps mc_counter advanced
//against how many releases we detected - this catches multi-step jumps too
//(e.g. mc_counter ticks twice between polls while no CAM_RELEASE edge was seen)

//CAM_RELEASE pulse is only ~50ms long, task must poll well below that to not miss the edge
constexpr const uint8_t TASK_MAIN_MS{10}; //100Hz

//inputs
using CAM_RELEASE = Mandala<mandala::est::env::usrb::b1>;
using MC_BIT0 = Mandala<mandala::est::env::usrb::b2>;
using MC_BIT1 = Mandala<mandala::est::env::usrb::b3>;

//telemetry
using M_RELEASE_COUNTER = Mandala<mandala::est::env::usr::u1>;
using M_MC_COUNTER = Mandala<mandala::est::env::usr::u2>;
using M_ERROR_COUNTER = Mandala<mandala::est::env::usr::u3>;

uint32_t RELEASE_COUNTER{};
uint32_t RELEASE_COUNTER_prev{};
uint32_t error_counter{};
uint8_t mc_counter{};
uint8_t mc_counter_prev{};

bool m_camReleaseOld{};

int main()
{
    CAM_RELEASE();
    MC_BIT0();
    MC_BIT1();

    M_RELEASE_COUNTER();
    M_MC_COUNTER();
    M_ERROR_COUNTER();

    task("on_main", TASK_MAIN_MS);

    printf("photo script ready...\n");

    return 0;
}

EXPORT void on_main()
{
    //counter microchip feedback, 2-bit: b2 low bit, b3 high bit
    bool mcB2 = (bool) MC_BIT0::value();
    bool mcB3 = (bool) MC_BIT1::value();
    mc_counter = (uint8_t) ((mcB2 ? 1 : 0) | (mcB3 ? 2 : 0));
    M_MC_COUNTER::publish((float) mc_counter);

    //count CAM_RELEASE 0->1 transitions
    bool camRelease = (bool) CAM_RELEASE::value();
    if (camRelease && !m_camReleaseOld) {
        RELEASE_COUNTER++;
        M_RELEASE_COUNTER::publish((float) RELEASE_COUNTER);
    }
    m_camReleaseOld = camRelease;

    //steps advanced since last poll: mc_counter wraps at 4, RELEASE_COUNTER doesn't
    uint8_t mcDelta = (uint8_t) ((mc_counter - mc_counter_prev + 4) % 4);
    uint32_t releaseDelta = RELEASE_COUNTER - RELEASE_COUNTER_prev;

    if (mcDelta != releaseDelta) {
        error_counter += (mcDelta > releaseDelta) ? (mcDelta - releaseDelta) : (releaseDelta - mcDelta);
        M_ERROR_COUNTER::publish((float) error_counter);
    }

    mc_counter_prev = mc_counter;
    RELEASE_COUNTER_prev = RELEASE_COUNTER;
}
