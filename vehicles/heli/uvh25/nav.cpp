#include <apx.h>

//======================================================================================
// CONSTANTS & CONFIGURATION
//======================================================================================

// Communication Ports
const uint8_t PORT_ID_CAN{11};
const uint8_t PORT_ID_ESC{12};

// Protocol Package Sizes
const uint8_t PACK_SIZE_ESC{10};

constexpr const uint16_t TASK_MAIN_PERIOD{100}; //ms
constexpr const uint16_t TASK_ERS_PERIOD{100};  //ms

//======================================================================================
// ERS CONSTANTS
//======================================================================================

#define R_SQUIB_MIN 1.0f //Ohm
#define R_SQUIB_MAX 8.5f //Ohm
#define R_WIRES 0.68f    //Ohm, measured by shorting pyro with diag on

#define MULT_PIRO_U_DIAG 1.128f
#define MULT_SQUIB_U_DIAG 1.1f
#define MULT_PIRO_U_ARM 1.128f
#define MULT_SQUIB_U_ARM 1.1f
#define DIAG_CNT_THRESHOLD 10 //1s for 100ms period, ADC should be set for 10Hz!
#define CHRG_CNT_THRESHOLD 30 //3s for 100ms period, ADC should be set for 10Hz!
#define MIN_VOLT_CHARGED 8.0f
#define MAX_VOLT_DIAG 3.3f
#define MAX_VOLT_DIAG_FIRE 0.3f

//======================================================================================
// CAN DEFINES AND IDS
//======================================================================================

// VESC Tail
#define VESC_TAIL_ID 0x24 // VESC ID 36
#define STATUS_MSG_1 0x09
#define STATUS_MSG_2 0x0E
#define STATUS_MSG_3 0x0F
#define STATUS_MSG_4 0x10
#define STATUS_MSG_5 0x1B
#define ERPM_DIVIDER 14.f //amount of magnets divided by 2

// UVHPU Power Management Unit
#define UVHPU_ID 0x80
#define UVHPU_PACK1 UVHPU_ID + 1
#define UVHPU_PACK2 UVHPU_ID + 2
#define UVHPU_PACK3 UVHPU_ID + 3
#define UVHPU_PACK4 UVHPU_ID + 4
#define UVHPU_PACK5 UVHPU_ID + 5
#define UVHPU_PACK6 UVHPU_ID + 6
#define UVHPU_PACK7 UVHPU_ID + 7

//MCELL
#define MCELL_ID 0x0100
#define MCELL_PACK1 MCELL_ID + 1
#define MCELL_PACK2 MCELL_ID + 2
#define MCELL_PACK3 MCELL_ID + 3
#define MCELL_PACK4 MCELL_ID + 4
#define MCELL_PACK5 MCELL_ID + 5
#define MCELL_PACK6 MCELL_ID + 6
#define MCELL_PACK7 MCELL_ID + 7

// Altitude (AGL) Sensor
#define AGL_CAN_ID 0x00090002

//======================================================================================
// ERS STATE ENUM
//======================================================================================

enum class ERS_State {
    ERROR = 0,
    DISARMED_INIT = 1,
    DISARMED_LOOP = 2,
    ARM_INIT = 3,
    ARM_LOOP = 4,
    FIRED = 5
};

// vesc
using m_vesc_tail_rpm = Mandala<mandala::est::env::usrf::f1>;        //+
using m_vesc_tail_current = Mandala<mandala::est::env::usrf::f2>;    //+
using m_vesc_tail_duty = Mandala<mandala::est::env::usrf::f3>;       //+
using m_vesc_tail_temp_fet = Mandala<mandala::est::env::usrf::f4>;   //+
using m_vesc_tail_temp_motor = Mandala<mandala::est::env::usrf::f5>; //+
using m_vesc_tail_curr_in = Mandala<mandala::est::env::usrf::f6>;    //+

// uvhpu
using m_pu_vbat = Mandala<mandala::sns::env::bat::voltage>; //+
using m_pu_vsys = Mandala<mandala::sns::env::pwr::vsys>;    //+
using m_pu_ibat = Mandala<mandala::sns::env::bat::current>; //+
using m_pu_tbat = Mandala<mandala::sns::env::bat::temp>;    //+
using m_pu_pbat = Mandala<mandala::est::env::usr::u1>;      //+
using m_pu_cbat = Mandala<mandala::est::env::usr::u2>;      //+
using m_pu_ebat = Mandala<mandala::est::env::usr::u3>;      //+
using m_pu_status = Mandala<mandala::est::env::usrc::c1>;   //+
using m_pu_hold = Mandala<mandala::est::env::usrc::c2>;     //+

// mcell
using m_mcell_vbat = Mandala<mandala::est::env::usr::u4>;    //+
using m_mcell_tpcb = Mandala<mandala::est::env::usr::u5>;    //+
using m_mcell_tbat = Mandala<mandala::est::env::usr::u6>;    //+
using m_mcell_max = Mandala<mandala::est::env::usr::u7>;     //+
using m_mcell_min = Mandala<mandala::est::env::usr::u8>;     //+
using m_mcell_status = Mandala<mandala::est::env::usrc::c3>; //+

// esc
using m_eng_temp = Mandala<mandala::sns::env::eng::temp>;       //+
using m_eng_volt = Mandala<mandala::sns::env::eng::voltage>;    //+
using m_eng_current = Mandala<mandala::sns::env::eng::current>; //+
using m_eng_rpm = Mandala<mandala::sns::env::eng::rpm>;         //+

// agl
using m_agl = Mandala<mandala::sns::nav::agl::radio>;

//ERS
using m_ers_status = Mandala<mandala::sns::env::ers::status>; //+
using m_ers_block = Mandala<mandala::sns::env::ers::block>;   //+
using m_ers_launch = Mandala<mandala::ctr::env::ers::launch>; //+

using m_ers_fire = Mandala<mandala::est::env::usrb::b1>;   //+
using m_ers_charge = Mandala<mandala::est::env::usrb::b2>; //+
using m_ers_diag = Mandala<mandala::est::env::usrb::b3>;   //+
using m_ers_led = Mandala<mandala::est::env::usrb::b4>;    //+

// using mandala variables
using m_mode = Mandala<mandala::cmd::nav::proc::mode>;
using m_rotor_rpm = Mandala<mandala::sns::env::prop::rpm>;
using m_pyro_volt = Mandala<mandala::est::env::usrc::c4>;
using m_squib_volt = Mandala<mandala::est::env::usrf::f7>;

// trim rudder
using m_reg_yaw = Mandala<mandala::cmd::nav::reg::yaw>;
using m_trim_rudder_l = Mandala<mandala::est::env::usrf::f9>;

// ESC VCP Data Structure
struct ESC_VCP_Data
{
    uint8_t temp;
    float voltage;
    float current;
    uint16_t consumption;
    uint16_t rpm;
};

// VESC CAN Data Structure
struct VESC_CAN_Data
{
    int32_t rpm;                 // STATUS_MSG_1
    float current;               // STATUS_MSG_1
    float duty;                  // STATUS_MSG_1
    uint32_t apm_hours;          // STATUS_MSG_2
    uint32_t apm_hours_charged;  // STATUS_MSG_2
    uint32_t watt_hours;         // STATUS_MSG_3
    uint32_t watt_hours_charged; // STATUS_MSG_3
    float temp_fet;              // STATUS_MSG_4
    float temp_mot;              // STATUS_MSG_4
    float curr_in;               // STATUS_MSG_4
    float pid_pos_now;           // STATUS_MSG_4
    float voltage;               // STATUS_MSG_5
    uint32_t tacho;              // STATUS_MSG_5
};

// MCELL
#pragma pack(1)
struct MCELL
{
    float v_bat;
    float t_bat;
    float t_pcb;
    uint8_t status;
    int16_t cell[12] = {};
    float cell_volt(uint8_t cell_idx) { return cell[cell_idx] / 1000.f; };
};

// UVHPU
struct UVHPU
{
    struct
    {
        float vbat;
        float ibat;
        float imon;
    } MSG1;
    struct
    {
        float vout;
        float tbat;
        float pbat;
        float status;
    } MSG2;
    struct
    {
        float cbat;
        float ebat;
    } MSG3;
    struct
    {
        float res_bar;
        float v_res;
    } MSG4;
    struct
    {
        float ibat_filt;
        float vbat_filt;
    } MSG5;
    struct
    {
        float cbat_res;
        float ebat_res;
    } MSG6;
    struct
    {
        int16_t life_cycles;
        float cbat_mod;
    } MSG7;
};
#pragma pack()

//======================================================================================
// GLOBAL VARIABLES
//======================================================================================

// ESC Handler Data
ESC_VCP_Data esc_data;
uint8_t esc_tbuf[PACK_SIZE_ESC]{};
// anti-stuck rpm logic variables
float rpm_prev = 0.0f;
uint8_t same_counter = 0;
#define SAME_LIMIT 30       //3 sec at 100ms interval
#define MIN_RPM_CHECK 500.f //reset stuck rpm only below this value

// VESC Tail Data
VESC_CAN_Data tail_data{};

// UVHPU Data
UVHPU _uvhpu{};

// MCELL data
MCELL _mcel{};

// ERS State
ERS_State ers_state = ERS_State::DISARMED_INIT;
bool fire_check_done = false;
uint8_t diag_counter = DIAG_CNT_THRESHOLD;
uint8_t fire_check_counter = DIAG_CNT_THRESHOLD;
uint8_t charge_counter = CHRG_CNT_THRESHOLD; //need more time to charge

constexpr const uint16_t SERVO_TASK_MS{20};
constexpr const float F = 3.f;
constexpr const float Ampl = 1.f;
constexpr const float w = 2.f * PI * F;
constexpr const float T = 1.f / F;
float test_servo = 0.f;

using m_sin_test = Mandala<mandala::est::env::usrf::f8>;

int main()
{
    schedule_periodic(task("on_main"), TASK_MAIN_PERIOD);
    schedule_periodic(task("on_ers"), TASK_ERS_PERIOD);

    //schedule_periodic(task("on_test_servo"), SERVO_TASK_MS);

    task("mcell"); //GCS with terminal command `vmexec("mcell")`
    task("uvhpu"); // GCS with terminal command `vmexec("uvhpu")`

    m_rotor_rpm();
    m_mode();

    m_squib_volt();
    m_pyro_volt();
    m_ers_fire();
    m_ers_block();
    m_ers_launch();

    m_reg_yaw();

    m_ers_block::publish(true);                                     //disarm on start
    m_ers_status::publish((uint32_t) mandala::ers_status_disarmed); //set status disarmed

    receive(PORT_ID_ESC, "esc_handler");
    receive(PORT_ID_CAN, "can_handler");
}

EXPORT void on_test_servo()
{
    float ctrl = Ampl * sin(w * test_servo);

    test_servo += SERVO_TASK_MS / 1000.f;
    if (test_servo >= T) {
        test_servo = 0.f;
    }

    m_sin_test::publish(ctrl);
}

uint8_t update_crc8(uint8_t data, uint8_t crc)
{
    data ^= crc;

    for (uint8_t i = 0; i < 8; i++) {
        data = uint8_t((data & 0x80) ? 0x07 ^ (data << 1) : (data << 1));
    }
    return data;
}

uint8_t get_crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc{0};
    for (uint8_t i = 0; i < len; i++) {
        crc = update_crc8(data[i], crc);
    }
    return crc & 0xFF;
}

void serializeInt(uint8_t *data, uint8_t index, int32_t value)
{
    for (uint8_t i = 0; i < 4; i++) {
        uint8_t shift = 8 * (4 - i - 1);
        data[index + i] = (value >> shift) & 0xFF;
    }
}

int16_t unpackInt16(const uint8_t *data, uint8_t index)
{
    return (int16_t) (data[index] | (data[index + 1] << 8));
}

void processMCELLPackage(const uint32_t &can_id, const uint8_t *data)
{
    switch (can_id) {
    case MCELL_PACK1: {
        _mcel.v_bat = (float) unpackInt16(data, 0) / 100.f;
        _mcel.t_bat = (float) unpackInt16(data, 2) / 100.f;
        _mcel.t_pcb = (float) unpackInt16(data, 4) / 100.f;
        _mcel.status = data[7];

        m_mcell_vbat::publish(_mcel.v_bat);
        m_mcell_tbat::publish(_mcel.t_bat);
        m_mcell_status::publish((uint32_t) _mcel.status);
        m_mcell_tpcb::publish(_mcel.t_pcb);
        break;
    }
    case MCELL_PACK2: {
        memcpy(_mcel.cell, data, 8);
        break;
    }
    case MCELL_PACK3: {
        memcpy(_mcel.cell + 4, data, 8);
        break;
    }
    case MCELL_PACK4: {
        memcpy(_mcel.cell + 8, data, 8);
        break;
    }
    case MCELL_PACK5: {
        memcpy(_mcel.cell + 12, data, 8);
        break;
    }
    case MCELL_PACK6: {
        memcpy(_mcel.cell + 16, data, 8);
        break;
    }
    case MCELL_PACK7: {
        memcpy(_mcel.cell + 20, data, 8);
        break;
    }
    }
}

void processVESCPackage(const uint32_t &msg_id, const uint8_t *data, VESC_CAN_Data *vesc_data)
{
    switch (msg_id) {
    case STATUS_MSG_1: {
        vesc_data->rpm = int32_t((data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]);
        vesc_data->current = float(int16_t((data[4] << 8) | data[5])) / 10.f;
        vesc_data->duty = float(int16_t((data[6] << 8) | data[7])) / 10.f;
        break;
    }
    case STATUS_MSG_2: {
        vesc_data->apm_hours = uint32_t((data[0] << 24) | (data[1] << 16) | (data[2] << 8)
                                        | data[3]);
        vesc_data->apm_hours_charged = uint32_t((data[4] << 24) | (data[5] << 16) | (data[6] << 8)
                                                | data[7]);
        break;
    }
    case STATUS_MSG_3: {
        vesc_data->watt_hours = uint32_t((data[0] << 24) | (data[1] << 16) | (data[2] << 8)
                                         | data[3]);
        vesc_data->watt_hours_charged = uint32_t((data[4] << 24) | (data[5] << 16) | (data[6] << 8)
                                                 | data[7]);
        break;
    }
    case STATUS_MSG_4: {
        vesc_data->temp_fet = float(int16_t((data[0] << 8) | data[1])) / 10.f;
        vesc_data->temp_mot = float(int16_t((data[2] << 8) | data[3])) / 10.f;
        vesc_data->curr_in = float(int16_t((data[4] << 8) | data[5])) / 10.f;
        vesc_data->pid_pos_now = float(int16_t((data[6] << 8) | data[7])) / 50.f;
        break;
    }
    case STATUS_MSG_5: {
        vesc_data->voltage = float(int16_t((data[4] << 8) | data[5])) / 10.f;
        vesc_data->tacho = uint32_t((data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]);
        break;
    }
    }
}

void processUVHPUackage(const uint32_t &can_id, const uint8_t *data)
{
    switch (can_id) {
    case UVHPU_PACK1: {
        _uvhpu.MSG1.vbat = (float) unpackInt16(data, 0) / 100.f;
        memcpy(&_uvhpu.MSG1.ibat, data + 2, 4);
        _uvhpu.MSG1.imon = (float) unpackInt16(data, 6) / 100.f;
        m_pu_vbat::publish(_uvhpu.MSG1.vbat);
        m_pu_vsys::publish(_uvhpu.MSG1.vbat);
        m_pu_ibat::publish(_uvhpu.MSG1.ibat);
        break;
    }
    case UVHPU_PACK2: {
        _uvhpu.MSG2.vout = (float) unpackInt16(data, 0) / 100.f;
        _uvhpu.MSG2.tbat = (float) unpackInt16(data, 2) / 100.f;
        _uvhpu.MSG2.pbat = (float) unpackInt16(data, 4);
        _uvhpu.MSG2.status = data[7];

        m_pu_status::publish(_uvhpu.MSG2.status);
        m_pu_pbat::publish(_uvhpu.MSG2.pbat);
        m_pu_tbat::publish(_uvhpu.MSG2.tbat);
        break;
    }
    case UVHPU_PACK3: {
        memcpy(&_uvhpu.MSG3.cbat, data, 8);
        m_pu_cbat::publish(_uvhpu.MSG3.cbat);
        m_pu_ebat::publish(_uvhpu.MSG3.ebat);
        break;
    }
    case UVHPU_PACK4: {
        memcpy(&_uvhpu.MSG4.res_bar, data, 8);
        break;
    }
    case UVHPU_PACK5: {
        memcpy(&_uvhpu.MSG5.ibat_filt, data, 8);
        break;
    }
    case UVHPU_PACK6: {
        memcpy(&_uvhpu.MSG6.cbat_res, data, 8);
        break;
    }
    case UVHPU_PACK7: {
        _uvhpu.MSG7.life_cycles = unpackInt16(data, 0);
        memcpy(&_uvhpu.MSG7.cbat_mod, data + 2, 4);
        break;
    }
    }
}

EXPORT void on_main()
{
    if (m_reg_yaw::value() >= (uint32_t) mandala::reg_yaw_fixed) {
        m_trim_rudder_l::publish(0.3f);
    } else {
        m_trim_rudder_l::publish(0.f);
    }

    // Save ESC data to mandala
    m_eng_temp::publish((uint32_t) esc_data.temp);
    m_eng_volt::publish((float) esc_data.voltage);
    m_eng_current::publish((float) esc_data.current);
    m_eng_rpm::publish((uint32_t) esc_data.rpm);

    // Calculate min and max cell voltages from MCELL data
    float vcl_max = -1.f;
    float vcl_min = 1000.f;

    for (uint8_t i = 0; i < 12; i++) {
        float cell_voltage = _mcel.cell_volt(i);
        if (cell_voltage > 0) { // Only consider valid readings
            if (cell_voltage > vcl_max) {
                vcl_max = cell_voltage;
            }
            if (cell_voltage < vcl_min) {
                vcl_min = cell_voltage;
            }
        }
    }

    // Publish min/max values if valid readings exist
    if (vcl_max > 0 && vcl_min < 1000.f) {
        m_mcell_max::publish(vcl_max);
        m_mcell_min::publish(vcl_min);
    }

    //RPM anti-stuck logic: if RPM is the same for a long time and less than 500, set it to 0
    float rpm_main = m_rotor_rpm::value();
    if (rpm_main == rpm_prev) {
        same_counter++;
        if (same_counter >= SAME_LIMIT && rpm_main < MIN_RPM_CHECK) {
            m_rotor_rpm::publish(0.f);
            same_counter = 0;
        }
    } else {
        rpm_prev = rpm_main;
        same_counter = 0;
    }

    //pu hold
    if (m_mode::value() == (uint32_t) mandala::proc_mode_TAXI) { //only in taxi mode
        m_pu_hold::publish(false);
    } else {
        m_pu_hold::publish(true);
    }
}

EXPORT void on_ers()
{
    /*float squib_R = (m_squib_U::value() * MULT_SQUIB_U_DIAG) / (200.0f)
                        / (m_pyro_U::value() * MULT_PIRO_U_DIAG / 5600.0f)
                    - R_WIRES;
    printf("resist %.2f", squib_R);*/

    switch (ers_state) {
    case ERS_State::DISARMED_INIT: {
        m_ers_led::publish(false);                                      //turn LED off
        m_ers_charge::publish(false);                                   //discharge capacitor
        m_ers_fire::publish(false);                                     //ensure fire is off
        m_ers_status::publish((uint32_t) mandala::ers_status_disarmed); //set status disarmed

        m_ers_diag::publish(true); //turn on diag voltage
        ers_state = ERS_State::DISARMED_LOOP;
        break;
    }
    case ERS_State::DISARMED_LOOP: {
        if (fire_check_done == false) { //fire check done once
            if (m_ers_fire::value() == false) {
                m_ers_fire::publish(true);
                return;
                //time to settle
            }

            float pyro_U = m_pyro_volt::value() * MULT_PIRO_U_DIAG;

            if (pyro_U > MAX_VOLT_DIAG_FIRE) {
                fire_check_counter--;
                if (fire_check_counter == 0) {
                    printf("VM:pyro voltage did not drop after fire check: %.2f", pyro_U);
                    printf("VM:ERS ERROR");
                    ers_state = ERS_State::ERROR;
                }
                return;
            } else {
                fire_check_counter = DIAG_CNT_THRESHOLD;
            }
            //everything ok, disarm fire and move on
            m_ers_fire::publish(false);
            //time to settle
            fire_check_done = true;
            return;
        }

        float squib_U = m_squib_volt::value() * MULT_SQUIB_U_DIAG;
        float pyro_U = m_pyro_volt::value() * MULT_PIRO_U_DIAG;
        float squib_R = (squib_U / 200.0f) / (pyro_U / 5600.0f) - R_WIRES;

        if (squib_R < R_SQUIB_MIN || squib_R > R_SQUIB_MAX || pyro_U > MAX_VOLT_DIAG) {
            diag_counter--;
            if (diag_counter == 0) {
                printf("VM:ERS ERROR");
                if (pyro_U > MAX_VOLT_DIAG)
                    printf("VM:pyro voltage too high: %.2f", pyro_U);
                else
                    printf("VM:squib resistance out of range: %.2f", squib_R);
                ers_state = ERS_State::ERROR;
            }
            return;
        } else {
            diag_counter = DIAG_CNT_THRESHOLD;
        }

        //all checks passed, wait for arm command
        if (m_ers_block::value() == false) {
            ers_state = ERS_State::ARM_INIT;
            printf("VM:ARMED");
        }
        break;
    }
    case ERS_State::ARM_INIT: {
        m_ers_diag::publish(false);                               //ensure diag voltage is off
        m_ers_led::publish(true);                                 //turn LED on
        m_ers_charge::publish(true);                              //charge capacitor
        m_ers_status::publish((uint32_t) mandala::ers_status_ok); //set status ok

        ers_state = ERS_State::ARM_LOOP;
        break;
    }

    case ERS_State::ARM_LOOP: {
        if (m_ers_block::value() == true) {
            ers_state = ERS_State::DISARMED_INIT;
            printf("VM:DISARMED");
            m_ers_charge::publish(false); //discharge capacitor
            return;
        }

        float squib_U = m_squib_volt::value() * MULT_SQUIB_U_ARM; //recalibrate after charging
        float pyro_U = m_pyro_volt::value() * MULT_PIRO_U_ARM;
        float squib_R = (squib_U / 200.0f) / (pyro_U / 5600.0f) - R_WIRES;

        if (pyro_U < MIN_VOLT_CHARGED) {
            charge_counter--;
            if (charge_counter == 0) {
                printf("VM:pyro voltage too low: %.2f", pyro_U);
                printf("VM:ERS ERROR");
            }
        } else {
            charge_counter = CHRG_CNT_THRESHOLD;
        }

        if (squib_R < R_SQUIB_MIN || squib_R > R_SQUIB_MAX) { //already stabilized at this point
            printf("VM:squib resistance out of range: %.2f", squib_R);
        }

        if (m_ers_launch::value() == true) {
            m_ers_fire::publish(true);
            ers_state = ERS_State::FIRED;
            printf("VM:ERS FIRED");
        }

        break;
    }

    case ERS_State::FIRED: {
        m_ers_led::publish(false); //turn arm LED off

        if (m_ers_block::value() == true) { //reset ERS state if needed by blocking ers
            ers_state = ERS_State::DISARMED_INIT;
            printf("DISARMED");
            m_ers_charge::publish(false); //discharge capacitor
        }
        break;
    }
    case ERS_State::ERROR: {
        m_ers_fire::publish(false);                                    //ensure fire is off
        m_ers_diag::publish(false);                                    //turn off diag voltage
        m_ers_led::publish(false);                                     //turn arm LED off
        m_ers_charge::publish(false);                                  //discharge capacitor
        m_ers_status::publish((uint32_t) mandala::ers_status_failure); //set status failure
        break;
    }
    }
}

EXPORT void esc_handler(const uint8_t *data, size_t size)
{
    if (size != PACK_SIZE_ESC) {
        return;
    }

    memcpy(esc_tbuf, data, size);

    if (get_crc8(esc_tbuf, PACK_SIZE_ESC - 1) != esc_tbuf[PACK_SIZE_ESC - 1]) {
        return;
    }

    esc_data.temp = data[0];
    esc_data.voltage = float((esc_tbuf[1] << 8) | (esc_tbuf[2])) / 100.f;
    esc_data.current = float((esc_tbuf[3] << 8) | (esc_tbuf[4])) / 100.f;
    esc_data.consumption = uint16_t((esc_tbuf[5] << 8) | (esc_tbuf[6]));
    esc_data.rpm = uint16_t((esc_tbuf[7] << 8) | (esc_tbuf[8])) * 100 / 7u;
}

EXPORT void can_handler(const uint8_t *data, size_t size)
{
    if (size < 4) {
        return;
    }

    uint32_t can_id = (uint32_t) (data[0] + (data[1] << 8) + (data[2] << 16) + (data[3] << 24));
    can_id &= 0x7FFFFFFF; // 32nd bit is ext/std flag

    uint8_t can_data[8] = {};
    for (uint8_t i = 0; i < 8; i++) {
        can_data[i] = data[4 + i]; // 4 is data position
    }

    // Process AGL (Altitude) message
    if (can_id == AGL_CAN_ID) {
        uint16_t raw = (uint16_t) ((data[4] << 8) + data[5]);
        float altitude = float(raw) / 100.0f;
        if (altitude > 0.1f && altitude < 40.0f)
            m_agl::publish(altitude);
        return;
    }

    // Process VESC Tail Motor (CAN ID 0xFF)
    switch (can_id & 0xFF) {
    case VESC_TAIL_ID: {
        uint16_t msg_id = (can_id >> 8) & 0xFF;
        processVESCPackage(msg_id, can_data, &tail_data);

        m_vesc_tail_rpm::publish((float) tail_data.rpm / ERPM_DIVIDER);
        m_vesc_tail_current::publish(tail_data.current);
        m_vesc_tail_duty::publish(tail_data.duty);
        m_vesc_tail_temp_fet::publish(tail_data.temp_fet);
        m_vesc_tail_temp_motor::publish(tail_data.temp_mot);
        m_vesc_tail_curr_in::publish(tail_data.curr_in);
        break;
    }
    }

    // Process UVHPU Power Management (CAN ID 0xFFFF)
    switch (can_id & 0xFFFF) {
    case MCELL_PACK1:
    case MCELL_PACK2:
    case MCELL_PACK3:
    case MCELL_PACK4:
    case MCELL_PACK5:
    case MCELL_PACK6:
    case MCELL_PACK7: {
        processMCELLPackage(can_id, can_data);
        break;
    }
    case UVHPU_PACK1:
    case UVHPU_PACK2:
    case UVHPU_PACK3:
    case UVHPU_PACK4:
    case UVHPU_PACK5:
    case UVHPU_PACK6:
    case UVHPU_PACK7: {
        processUVHPUackage(can_id, can_data);
        break;
    }
    }
}

EXPORT void uvhpu()
{
    printf("vbat: %.2f", _uvhpu.MSG1.vbat);
    printf("ibat: %.2f", _uvhpu.MSG1.ibat);
    printf("imon: %.2f", _uvhpu.MSG1.imon);

    printf("vout: %.2f", _uvhpu.MSG2.vout);
    printf("tbat: %.2f", _uvhpu.MSG2.tbat);
    printf("pbat: %.2f", _uvhpu.MSG2.pbat);
    printf("status: %u", _uvhpu.MSG2.status);

    printf("cbat: %.2f", _uvhpu.MSG3.cbat);
    printf("ebat: %.2f", _uvhpu.MSG3.ebat);

    printf("res_bar: %.2f", _uvhpu.MSG4.res_bar);
    printf("v_res: %.2f", _uvhpu.MSG4.v_res);

    printf("ibat_filt: %.2f", _uvhpu.MSG5.ibat_filt);
    printf("vbat_filt: %.2f", _uvhpu.MSG5.vbat_filt);

    printf("cbat_res: %.2f", _uvhpu.MSG6.cbat_res);
    printf("ebat_res: %.2f", _uvhpu.MSG6.ebat_res);

    printf("life_cycles: %u", _uvhpu.MSG7.life_cycles);
    printf("cbat_mod: %.2f", _uvhpu.MSG7.cbat_mod);
}

EXPORT void mcell()
{
    printf("v_bat: %.2f", _mcel.v_bat);
    printf("t_bat: %.2f", _mcel.t_bat);
    printf("t_pcb: %.2f", _mcel.t_pcb);
    printf("state: %u", _mcel.status);

    printf("C[1]: %.2f", _mcel.cell_volt(0));
    printf("C[2]: %.2f", _mcel.cell_volt(1));
    printf("C[3]: %.2f", _mcel.cell_volt(2));
    printf("C[4]: %.2f", _mcel.cell_volt(3));

    printf("C[5]: %.2f", _mcel.cell_volt(4));
    printf("C[6]: %.2f", _mcel.cell_volt(5));
    printf("C[7]: %.2f", _mcel.cell_volt(6));
    printf("C[8]: %.2f", _mcel.cell_volt(7));

    printf("C[9] %.2f", _mcel.cell_volt(8));
    printf("C[10] %.2f", _mcel.cell_volt(9));
    printf("C[11] %.2f", _mcel.cell_volt(10));
    printf("C[12] %.2f", _mcel.cell_volt(11));
}
