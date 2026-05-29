//Fuel control protocol
//UART: 19200 8 N 1

#include <apx.h>

//#define FUEL_SIM
#define SIM_SPEED 0.02f

static constexpr const port_id_t port_fuel_id{12};

const uint16_t TASK_MAIN_MS{100}; //msec
const uint16_t TASK_ERS_MS{100};  //msec
const uint16_t TASK_FUEL_MS{500}; //msec

//FUEL
const uint8_t ADR_FUEL_SENS1{85}; //75
const uint8_t ADR_FUEL_SENS2{86}; //76
const uint8_t ADR_FUEL_SENS3{87}; //77

const uint8_t MSG_FUEL_SIZE{9}; //FUEL

const float V_MAX1{16.7f}; //liters
const float V_MAX2{16.7f}; //liters
const float V_MAX3{16.7f}; //liters

const float CRITICAL_LOW{7.0f};            //%, reaching this level considered empty tank
const float TANK_2_KEEPFULL{95.f};         //%, if below, start pumping fuel in tank 2
const float TANK_2_POINT{45.f};            //%, starting next stage when reaching this point
const float TANK_12_RATIO{2.f};            //ratio of fuel to keep between tank 1 and 2
const float TANK_13_MAX_DIFF{20.f};        //%, max allowed difference between tank 1 and 3 levels
const float TANK_13_MIN_DIFF{10.0f};       //%, min required difference between tank 1 and 3 levels
const float TANK_13_MAX_DIFF_LEVELS{50.f}; //%, max diff between tank 1 and 3 at 50% average level
const float TANK_13_MIN_DIFF_LEVELS{20.f}; //%, min diff between tank 1 and 3 at 20% average level

const float k1 = (TANK_13_MAX_DIFF - 0.f) / (TANK_13_MAX_DIFF_LEVELS - 100.f); //0 is a start diff
const float b1 = 0.f - k1 * 100.f;

const float k2 = (TANK_13_MIN_DIFF - TANK_13_MAX_DIFF)
                 / (TANK_13_MIN_DIFF_LEVELS - TANK_13_MAX_DIFF_LEVELS);
const float b2 = TANK_13_MAX_DIFF - k2 * TANK_13_MAX_DIFF_LEVELS;

uint8_t snd_fuel_buf[MSG_FUEL_SIZE] = {};

uint8_t ADR_FUEL[3] = {ADR_FUEL_SENS1, ADR_FUEL_SENS2, ADR_FUEL_SENS3};
uint8_t idx_fuel_array = 0;

uint32_t startTimerPumpON;

uint8_t pump_stage{1}; //default stage

const uint8_t TIME_SA{5};        //sec
const uint8_t DELAY_PUMP{15};    //pump works for 15 sec before switching to another
const uint8_t TIME_TO_EMPTY{60}; //sec. Time to drain tank from critical to empty level

bool tank1_critical{false};
bool tank3_critical{false};
bool fuel_mc_old{false};
bool ignition_old{false};

struct _fuel
{
    _fuel(float v_max)
        : V_MAX(v_max)
        , liters(0.f)
        , percent(0.f)
        , time_ans(0)
    {}

    void set_percent(float p)
    {
        percent = p;
        liters = (p * V_MAX) / 100.f;
        time_ans = time_ms();
    }

    float V_MAX;   // max tank liters
    float liters;  // liters
    float percent; // percent
    uint32_t time_ans;
};

_fuel fuel[3] = {_fuel(V_MAX1), _fuel(V_MAX2), _fuel(V_MAX3)};

//Engine
using m_pwr_eng = Mandala<mandala::ctr::env::pwr::eng>;
using m_eng_cut = Mandala<mandala::cmd::nav::eng::cut>;

//fuel
using m_fuel1 = Mandala<mandala::est::env::usr::u5>;
using m_fuel2 = Mandala<mandala::est::env::usr::u6>;
using m_fuel3 = Mandala<mandala::est::env::usr::u7>;

using m_fuel_p = Mandala<mandala::sns::env::fuel::level>;
using m_fuel_l = Mandala<mandala::est::env::usr::u8>;

using m_pump1 = Mandala<mandala::ctr::env::sw::sw5>;
using m_pump2 = Mandala<mandala::ctr::env::sw::sw6>;
using m_pump3 = Mandala<mandala::ctr::env::sw::sw7>;

using m_warn1 = Mandala<mandala::est::env::usrb::b5>;
using m_warn2 = Mandala<mandala::est::env::usrb::b6>;
using m_warn3 = Mandala<mandala::est::env::usrb::b7>;

//using m_error1 = Mandala<mandala::est::env::usrc::c5>;
//using m_error2 = Mandala<mandala::est::env::usrc::c6>;
//using m_error3 = Mandala<mandala::est::env::usrc::c7>;

using m_fuel_mc = Mandala<mandala::ctr::env::sw::sw8>; //manual control

//datalink
using m_ltt = Mandala<mandala::est::env::sys::ltt>;
using m_health = Mandala<mandala::est::env::sys::health>;
using m_mode = Mandala<mandala::cmd::nav::proc::mode>;
using m_stage = Mandala<mandala::cmd::nav::proc::stage>;
using m_geo_safety = Mandala<mandala::est::nav::geo::safe>;

//ers
using m_ers1 = Mandala<mandala::ctr::env::ers::launch>;
using m_ers2 = Mandala<mandala::est::env::usrb::b1>;
using m_ers3 = Mandala<mandala::ctr::env::ers::rel>;

using m_air = Mandala<mandala::est::env::usrb::b8>;

using m_roll = Mandala<mandala::est::nav::att::roll>;
using m_pitch = Mandala<mandala::est::nav::att::pitch>;
using m_airspeed = Mandala<mandala::est::nav::air::airspeed>;
using m_altitude = Mandala<mandala::est::nav::pos::altitude>;
using m_vspeed = Mandala<mandala::est::nav::pos::vspeed>;

using m_ax = Mandala<mandala::est::nav::acc::x>;
using m_ay = Mandala<mandala::est::nav::acc::y>;
using m_az = Mandala<mandala::est::nav::acc::z>;

//emergency parachute deployment
static constexpr const float ERS_VDOWN{-30.f}; //[m/sec]
static constexpr const float ERS_ROLL{70.f};   //[dec]
static constexpr const float ERS_PITCH{50.f};  //[dec]

//parachute release
static constexpr const float GRAVITY_NORM{9.8f};   //[m/sec^2]
static constexpr const float RELEASE_VDOWN{-0.5f}; //[m/sec]
static constexpr const float G_MAX{5.5f};          //[G]

static constexpr const float AIR_ALT{500.f};  //[m]
static constexpr const float AIR_SPD{20.f};   //[m/sec]
static constexpr const float ERS2_ALT{200.f}; //[m]
static constexpr const float ERS3_ALT{10.f};  //[m]

uint16_t WAIT_TIME{3000}; //[msec]
bool g_onERS{false};      //flag that ERS1 was activated

uint32_t g_lastRollPitchTime{0};
uint32_t g_lastVspeedTime{0};
uint32_t g_LastVspeedReleaseTime{0};
uint32_t g_LastMaxRollInTakeoffModeTime{0};

bool g_checkAirLockout{false};
bool g_lowAltLockout{false};
bool g_AttAndVSpeedLockout{false};
bool g_onERS2Lockout{false};
bool g_onERS3Lockout{false};

bool g_targetMode{false};

int main()
{
    schedule_periodic(task("on_main"), TASK_MAIN_MS);
    schedule_periodic(task("on_ers"), TASK_ERS_MS);
    schedule_periodic(task("on_fuel"), TASK_FUEL_MS);

    receive(port_fuel_id, "on_fuel_serial");

    //fuel
    m_fuel_p();
    m_fuel_l();

    m_fuel1();
    m_fuel2();
    m_fuel3();

    m_pump1();
    m_pump2();
    m_pump3();

    m_warn1();
    m_warn2();
    m_warn3();

    m_fuel_mc();

    //datalink
    m_ltt();
    m_health();
    m_mode();
    m_stage();
    m_geo_safety();

    //ers
    m_ers1();
    m_ers2();
    m_ers3();

    m_pwr_eng();
    m_air();

    m_roll();
    m_pitch();
    m_airspeed();
    m_altitude();
    m_vspeed();
    m_ax();
    m_ay();
    m_az();

    uint32_t now = time_ms();

    g_lastRollPitchTime = now;
    g_lastVspeedTime = now;
    g_LastVspeedReleaseTime = now;

    m_ers1::publish(false);
    m_ers2::publish(false);
    m_ers3::publish(false);

    m_air::publish(false);

    m_ers1("on_launch"); // subscribe

    return 0;
}

EXPORT void on_main()
{
    //datalink
    if ((uint32_t) m_ltt::value() < 10) {
        m_health::publish((uint32_t) mandala::sys_health_normal);
    }

    if ((uint32_t) m_health::value() == mandala::sys_health_warning
        && (uint32_t) m_mode::value() != mandala::proc_mode_TAXI) {
        m_mode::publish((uint32_t) mandala::proc_mode_LANDING);
    }

    //geofence
    if ((uint32_t) m_geo_safety::value() == mandala::geo_safe_critical
        && (uint32_t) m_mode::value() != (uint32_t) mandala::proc_mode_LANDING) {
        m_mode::publish((uint32_t) mandala::proc_mode_LANDING);
        printf("VM:Geofence critical\n");
    }
}

bool checkRollAndPitch()
{
    const float roll = fabs(m_roll::value() * R2D);
    const float pitch = fabs(m_pitch::value() * R2D);
    const uint32_t now = time_ms();

    if (roll > ERS_ROLL || pitch > ERS_PITCH) {
        return (now - g_lastRollPitchTime > 2000);
    }
    g_lastRollPitchTime = now;

    return false;
}

bool checkVSpeed()
{
    const float vspeed = m_vspeed::value();
    const uint32_t now = time_ms();

    if (vspeed < ERS_VDOWN) {
        return (now - g_lastVspeedTime > 1000);
    }
    g_lastVspeedTime = now;

    return false;
}

bool checkTakeOFFSafety()
{
    const bool mode = (uint8_t) m_mode::value() == mandala::proc_mode_TAKEOFF;
    const bool roll = fabs(m_roll::value() * R2D) > ERS_ROLL;
    const bool vspeed = m_vspeed::value() < -1.f;
    const uint32_t now = time_ms();

    if (mode && roll && vspeed) {
        return (now - g_LastMaxRollInTakeoffModeTime > 1000);
    }
    g_LastMaxRollInTakeoffModeTime = now;

    return false;
}

bool checkVSpeedAndAltitudeRelease()
{
    const float vspeed = m_vspeed::value();
    const float altitude = m_altitude::value();
    const uint32_t now = time_ms();

    if ((vspeed > RELEASE_VDOWN) && (altitude < ERS3_ALT)) {
        return (now - g_LastVspeedReleaseTime > 500);
    }
    g_LastVspeedReleaseTime = now;

    return false;
}

bool checkGForceRelease()
{
    float altitude = (float) m_altitude::value();

    float Ax = (float) m_ax::value() / GRAVITY_NORM;
    float Ay = (float) m_ay::value() / GRAVITY_NORM;
    float Az = (float) m_az::value() / GRAVITY_NORM;

    float val = sqrt(Ax * Ax + Ay * Ay + Az * Az);

    if (altitude < ERS3_ALT && val > G_MAX) {
        printf("VM:G_VAL %.2f\n", val);
        return true;
    }

    return false;
}

EXPORT void on_ers()
{
    if ((uint32_t) m_mode::value() == mandala::proc_mode_TAXI) {
        return;
    }

    if ((uint32_t) m_mode::value() == mandala::proc_mode_UAV && (uint32_t) m_stage::value() == 2) {
        g_targetMode = true;
        return;
    }

    if (g_targetMode && m_vspeed::value() > 0) {
        g_targetMode = false;
    }

    if (g_targetMode) {
        return;
    }

    const float altitude = (float) m_altitude::value();
    const float airspeed = (float) m_airspeed::value();
    const bool ers1 = (bool) m_ers1::value();

    //check air state
    if (!g_checkAirLockout && altitude > AIR_ALT && airspeed > AIR_SPD) {
        g_checkAirLockout = true;

        g_lowAltLockout = false;
        g_AttAndVSpeedLockout = false;
        g_onERS2Lockout = false;
        g_onERS3Lockout = false;
        g_targetMode = false;

        m_air::publish(true);
        pump_stage = 1; //restart fuel algorithm after launch
        printf("VM:Start AIR\n");
    }

    //minimum safe altitude for parachute release
    if (g_checkAirLockout && !g_lowAltLockout && altitude < ERS2_ALT) {
        g_lowAltLockout = true;
        g_onERS = true;
        WAIT_TIME = 2500;
        m_ers1::publish(true);
        printf("VM:Low alt. ERS on\n");
    }

    const bool m_air_val = (bool) m_air::value() || g_checkAirLockout;
    const bool is_rp = checkRollAndPitch();
    const bool is_vspd = checkVSpeed();

    const bool ers = (m_air_val && !g_AttAndVSpeedLockout && (is_rp || is_vspd))
                     || checkTakeOFFSafety();

    if (ers) {
        g_AttAndVSpeedLockout = true;
        g_onERS = true;
        m_ers1::publish(true);
        if (altitude < ERS2_ALT) {
            WAIT_TIME = 100;
        }

        printf("VM:ERS1 on\n");
        printf("VM:rp %d\n", is_rp);
        printf("VM:vspd %d\n", is_vspd);
    }

    if (ers1 || g_onERS) {
        g_onERS = true;
        sleep(WAIT_TIME);
        WAIT_TIME = 0;

        //main parachute
        if (!g_onERS2Lockout && altitude < ERS2_ALT) {
            g_onERS2Lockout = true;
            m_ers2::publish(true);
            printf("VM:ERS2 on\n");
        }

        //release parachute
        const bool lvs_release = checkVSpeedAndAltitudeRelease();
        const bool gmax_release = checkGForceRelease();
        if (g_onERS2Lockout && !g_onERS3Lockout && (lvs_release || gmax_release)
            && (altitude < ERS3_ALT)) {
            g_onERS3Lockout = true;
            m_ers3::publish(true);
            printf("VM:ERS3 ok\n");
            printf("VM:LVS %u\n", lvs_release);
            printf("VM:GMAX %u\n", gmax_release);
        }
    }
}

EXPORT void on_launch()
{
    if ((bool) m_ers1::value()) {
        sleep(100);
        m_eng_cut::publish(true);
        m_pwr_eng::publish(false);
    }
}

EXPORT uint8_t calcCRC(const uint8_t *buf, size_t size)
{
    uint8_t crc = 0x00;

    for (uint8_t j = 0; j < size; j++) {
        uint8_t i = buf[j] ^ crc;
        crc = 0;
        if (i & 0x01)
            crc ^= 0x5e;
        if (i & 0x02)
            crc ^= 0xbc;
        if (i & 0x04)
            crc ^= 0x61;
        if (i & 0x08)
            crc ^= 0xc2;
        if (i & 0x10)
            crc ^= 0x9d;
        if (i & 0x20)
            crc ^= 0x23;
        if (i & 0x40)
            crc ^= 0x46;
        if (i & 0x80)
            crc ^= 0x8c;
    }
    return crc % 256;
}

void turn_off_all_pumps()
{
    m_pump1::publish(false);
    m_pump2::publish(false);
    m_pump3::publish(false);
}

void turn_on_all_pumps()
{
    m_pump1::publish(true);
    m_pump2::publish(true);
    m_pump3::publish(true);
}

void turn_off_pump_1()
{
    m_pump1::publish(false);
}

void turn_off_pump_3()
{
    m_pump3::publish(false);
}

void turn_on_pump_1()
{
    if (time_ms() > startTimerPumpON + DELAY_PUMP * 1000) {
        if ((bool) m_pump3::value()) { //if pump 3 is on, turn it off before turning on pump 1
            turn_off_pump_3();
            return;
        }
        if (!(bool) m_pump1::value()) { //if pump 1 is not on, turn it on and start timer
            m_pump1::publish(true);
            startTimerPumpON = time_ms();
        }
    }
}

void turn_on_pump_2()
{
    m_pump2::publish(true);
}

void turn_on_pump_3()
{
    if (time_ms() > startTimerPumpON + DELAY_PUMP * 1000) {
        if ((bool) m_pump1::value()) { //if pump 1 is on, turn it off before turning on pump 3
            turn_off_pump_1();
            return;
        }
        if (!(bool) m_pump3::value()) { //if pump 3 is not on, turn it on and start timer
            m_pump3::publish(true);
            startTimerPumpON = time_ms();
        }
    }
}

void pump_stage_1() //keep tank 2 full, pumping from 1 and 3 according to required difference
{                   // prioritize keeping more fuel in tank 1
    if (fuel[1].percent < TANK_2_KEEPFULL) {
        float avrg_t1t3 = (fuel[0].percent + fuel[2].percent) / 2.f;

        float req_diff = 0.0f;
        bool valid = true;

        if (avrg_t1t3 > TANK_13_MAX_DIFF_LEVELS) {
            req_diff = k1 * avrg_t1t3 + b1;
        } else if (avrg_t1t3 > TANK_13_MIN_DIFF_LEVELS) {
            req_diff = k2 * avrg_t1t3 + b2;
        } else {
            valid = false;
            pump_stage = 2; //if both 1 and 3 tanks are near 15%
            printf("fuel stage: 2");
            turn_off_pump_1(); //turn off pump 1
        }

        if (valid) {
            //keep required difference between tank 1 and 3
            if (fuel[0].percent - fuel[2].percent > req_diff) {
                turn_on_pump_1();
            } else {
                turn_on_pump_3();
            }
        }
    } else {
        turn_off_pump_1(); //does this prevent pumps from working for 15 sec???
        turn_off_pump_3();
    }
}

void pump_stage_2() //leaving tank 1 with around 15%, pump from tank 3 until empty
{
    if (!tank3_critical) {
        if (fuel[1].percent < TANK_2_KEEPFULL) { //if tank 2 is below 95%, pump fuel
            turn_on_pump_3();
        } else { //if tank 2 is full, turn off pumps and wait until fuel drops below 95% again
            turn_off_pump_1();
            turn_off_pump_3();
        }

        //if tank 3 near empty - start pump 3 for 1 minute to drain it for sure
        if (fuel[2].percent < CRITICAL_LOW) {
            tank3_critical = true;
            //force restart pump3 and it's timer immediately
            m_pump1::publish(false);
            m_pump3::publish(true);
            startTimerPumpON = time_ms();
        }
    } else if (time_ms() > startTimerPumpON + TIME_TO_EMPTY * 1000) {
        pump_stage = 3;
        printf("fuel stage: 3");
        turn_off_pump_3();
    }
}

void pump_stage_3() //leaving tank 1 with around 25%, pump from tank 2 until ~45%
{                   //pump 2 works when engine works, so no need to turn it on specifically
    if (fuel[1].percent < TANK_2_POINT) {
        pump_stage = 4;
        printf("fuel stage: 4");
    }
}

void pump_stage_4() // pump from tank 1 and tank 2 with specific ratio
{
    if (!tank1_critical) {
        if (fuel[1].percent > fuel[0].percent * TANK_12_RATIO) {
            turn_off_pump_1();
        } else {
            turn_on_pump_1();
        }

        if (fuel[0].percent < CRITICAL_LOW) {
            tank1_critical = true;
            //force restart pump1 and it's timer immediately
            m_pump3::publish(false);
            m_pump1::publish(true);
            startTimerPumpON = time_ms();
        }
    } else if (time_ms() > startTimerPumpON + TIME_TO_EMPTY * 1000) {
        turn_off_pump_1();
    }
}

void fuel_auto_control()
{
    turn_on_pump_2(); //keep pump2 on while engine is on

    switch (pump_stage) {
    case 1:
        pump_stage_1();
        break;
    case 2:
        pump_stage_2();
        break;
    case 3:
        pump_stage_3();
        break;
    case 4:
        pump_stage_4();
        break;
    }
}

void check_answer_time()
{
    uint32_t now = time_ms();

    m_warn1::publish((fuel[0].time_ans + TIME_SA * 1000 < now) ? true : false);
    m_warn2::publish((fuel[1].time_ans + TIME_SA * 1000 < now) ? true : false);
    m_warn3::publish((fuel[2].time_ans + TIME_SA * 1000 < now) ? true : false);
}

EXPORT void on_fuel()
{
    check_answer_time();

    m_fuel1::publish(fuel[0].liters);
    m_fuel2::publish(fuel[1].liters);
    m_fuel3::publish(fuel[2].liters);

    m_fuel_l::publish(fuel[0].liters + fuel[1].liters + fuel[2].liters);
    m_fuel_p::publish((fuel[0].percent + fuel[1].percent + fuel[2].percent) / 3.f);

    const bool ignition = (bool) m_pwr_eng::value();
    const bool fuel_mc = (bool) m_fuel_mc::value();
    const bool ers = (bool) m_ers1::value();

    if (ers) {
        turn_off_all_pumps();
    } else {
        if (ignition != ignition_old) {
            ignition_old = ignition;
            if (!ignition) {
                turn_off_all_pumps();
            }
        }

        if (fuel_mc != fuel_mc_old) {
            fuel_mc_old = fuel_mc;
            if (fuel_mc) {
                printf("Manual fuel ON");
            } else {
                turn_off_all_pumps();
                printf("Manual fuel OFF");
            }
        }

        if (!fuel_mc && ignition) {
            fuel_auto_control();
        }
    }

    //request fuel sensors
    snd_fuel_buf[0] = 0x31;
    snd_fuel_buf[2] = 0x06;

    snd_fuel_buf[1] = ADR_FUEL[idx_fuel_array++];
    if (idx_fuel_array == 3) {
        idx_fuel_array = 0;
    }

    snd_fuel_buf[3] = calcCRC(snd_fuel_buf, 3);
    send(port_fuel_id, snd_fuel_buf, 4, true);
}

EXPORT void on_fuel_serial(const uint8_t *data, size_t size)
{
    //typePack(0)[0x3E] adr(1) fmt(2) data(3) crc(8)
    if (size != MSG_FUEL_SIZE) {
        return;
    }

    if ((data[0] != 0x3E) || (data[2] != 0x06)) {
        return;
    }

    if (data[8] != calcCRC(data, 8)) {
        return;
    }

    //uint32_t error = data[3];
    //if (error < 150) //error codes start from 150
    //    error = 0;

    float fuel_percent = (float) (data[4] | (data[5] << 8)) / 10.f;

    if (data[1] == ADR_FUEL[0]) {
        fuel[0].set_percent(fuel_percent);
        //m_error1::publish(error);
    } else if (data[1] == ADR_FUEL[1]) {
        fuel[1].set_percent(fuel_percent);
        //m_error2::publish(error);
    } else if (data[1] == ADR_FUEL[2]) {
        fuel[2].set_percent(fuel_percent);
        //m_error3::publish(error);
    }
}
