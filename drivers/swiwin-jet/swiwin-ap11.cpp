//Swiwin turbine control protocol version 1.0
//UART: 9600 8 N 2

#include <apx.h>

static constexpr const port_id_t port_id{12};

const uint8_t MSG_SIZE = 7;

uint16_t frame_length;

uint8_t m_rev_buf[8] = {};
uint8_t m_snd_buf[8] = {};

typedef enum {
    eSW_DISABLE = 0,
    eSW_STOP = 1,
    eSW_READY = 2,
    eSW_START = 3,
} eSWState;

enum {
    eUPDATE_RATE_20Hz = 0,
    eUPDATE_RATE_50Hz = 1,
    eUPDATE_RATE_100Hz = 2, //2 & 3
};

enum {
    eCMD_EXHAUST_AIR = 1,       // Ver:1
    eCMD_TEST_GLOWPLUG = 2,     // Ver:1
    eCMD_TEST_FUEL_VALVE = 3,   // Ver:1
    eCMD_TEST_IGNI_VALVE = 4,   // Ver:1
    eCMD_TEST_PUMP = 5,         // Ver:1
    eCMD_TEST_STARTER = 6,      // Ver:1
    eCMD_SEND_RATE_20Hz = 7,    // Ver:1
    eCMD_SEND_RATE_50Hz = 8,    // Ver:1
    eCMD_SEND_RATE_100Hz = 9,   // Ver:1
    eCMD_RESET_FLOW = 10,       // Ver:2
    eCMD_CALI_THRUST_ZERO = 11, // Ver:2
    eCMD_TEST_Max
};

enum {
    eCMD_POWER = 1,
    eCMD_SHORT = 2,
    eCMD_UNLOCK = 3,
    eCMD_IGN_PUMP = 4,
    eCMD_RPM_ACC = 5,
};

int32_t m_engine_rpm = 0;

int32_t m_engine_temp = 0;
int32_t m_switch = 0;
uint32_t m_engine_status = 0;
uint32_t m_error_code = 0;

uint32_t m_last_error_code = 0;
uint32_t m_last_engine_status = 0;

int32_t m_rc_cur_voltage = 0;
int32_t m_pwr_cur_voltage = 0;
int32_t m_pump_cur_voltage = 0;

int32_t m_throttle = 0;
int32_t m_pressure = 0;

int32_t m_current = 0;
int32_t m_thrust = 0;

int32_t m_pump_ign_voltage = 0;
int32_t m_rpm_acc = 0;

int32_t m_engine_max_rpm = 0;
int32_t m_pump_max_voltage = 0;
int32_t m_version = 0;
int32_t m_update_rate = 0;

int32_t m_flow_rate = 0;
int32_t m_flow_total = 0;

int32_t m_engine_idle_rpm = 0;

int32_t m_ctrl_switch = eSW_DISABLE; //0 ~ 3,     1:Off  2:Ready  3:Start
int32_t m_ctrl_throttle = 0;         //0 ~ 100%
uint8_t m_set_ign_pump = 0;          //Set Pump Voltage of Igniton: 0.10 ~ 5.00v
uint8_t m_set_rpm_acc = 0;           //Set RPM ACC: 10 ~ 70
uint8_t m_cmd_param = 0;             //Command

int32_t m_set_update_rate = eUPDATE_RATE_20Hz;

using m_rpm = Mandala<mandala::sns::env::eng::rpm>;
using m_egt = Mandala<mandala::sns::env::eng::egt>;
using m_eng_voltage = Mandala<mandala::sns::env::eng::voltage>;
using m_vsrv = Mandala<mandala::sns::env::pwr::vsrv>;
using m_pump_volt = Mandala<mandala::est::env::usr::u2>; //pump voltage
using m_thr_fb = Mandala<mandala::est::env::usr::u1>;    //fb throttle

//---------------------------------------------------------------
float new_throttle;
bool start_eng = false;
using m_pwr_ign = Mandala<mandala::ctr::env::pwr::eng>;
using m_eng_mode = Mandala<mandala::cmd::nav::eng::mode>;

using m_thr = Mandala<mandala::ctr::nav::eng::thr>;
//---------------------------------------------------------------

//---------------------------------------------------------------
static const unsigned char crc_array[] = {
    0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
    0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
    0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
    0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
    0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
    0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
    0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
    0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
    0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
    0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
    0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
    0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
    0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
    0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
    0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
    0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
};

/***************************************************************
** CRC8
***************************************************************/
unsigned char calc_crc8(unsigned char *buf, unsigned char crc_len)
{
    unsigned char i, crc8 = 0;
    for (i = 0; i < crc_len; i++) {
        crc8 = crc_array[crc8 ^ buf[i]];
    }
    return crc8;
}
//---------------------------------------------------------------
float LimitSignal(float signal, const float &min, const float &max)
{
    if (signal < min)
        return min;

    if (signal > max)
        return max;

    return signal;
}
//---------------------------------------------------------------

/***************************************************************
** Set State for RC Switch
** Input: eSW_STOP, eSW_READY, eSW_START
***************************************************************/
void set_switch(eSWState sw)
{
    m_ctrl_switch = sw;
}

/***************************************************************
** Set Valve for RC Throttle
** Input: 0 ~ 1000
***************************************************************/
void set_throttle(int32_t val)
{
    if (val > 1000) {
        val = 1000;
    }
    m_ctrl_throttle = val;
}

const char *EcuStrError(uint32_t id)
{
    switch (id) {
    case 0: { return "null..."; }
    case 1: { return "timout..."; }
    case 2: { return "low_vbat..."; }
    case 3: { return "glowplug..."; }
    case 4: { return "pump..."; }
    case 5: { return "starter..."; }
    case 6: { return "rpm_low..."; }
    case 7: { return "rpm_instability..."; }
    case 8: { return "temp_high..."; }
    case 9: { return "temp_low..."; }
    case 10: { return "temp_sensor..."; }
    case 11: { return "valve_s..."; }
    case 12: { return "valve_m..."; }
    case 13: { return "lost_radio..."; }
    case 14: { return "starter_ctl_htmp..."; }
    case 15: { return "pump_ctl_htmp..."; }
    case 16: { return "clutch_error..."; }
    case 17: { return "current_overload..."; }
    case 18: { return "engine_offline..."; }
    }
    return "unknown";
}

const char *EcuStrStatus(uint32_t id)
{
    switch (id) {
    case 0: { return "stop..."; }
    case 1: { return "ready..."; }
    case 2: { return "ign_stick_down..."; }
    case 3: { return "ign_run..."; }
    case 4: { return "preheat..."; }
    case 5: { return "fuelramp..."; }
    case 6: { return "run_learn_stick_max..."; }
    case 7: { return "run_learn_stick_min..."; }
    case 8: { return "run_learn_rc..."; }
    case 9: { return "run_stick_min..."; }
    case 10: { return "run_pump_limit..."; }
    case 11: { return "running..."; }
    case 12: { return "cooling..."; }
    case 13: { return "restart..."; }
    case 14: { return "test_glowplug..."; }
    case 15: { return "test_valve_main..."; }
    case 16: { return "test_valve_ignite..."; }
    case 17: { return "test_oilpump..."; }
    case 18: { return "test_starter..."; }
    case 19: { return "exhaust_air..."; }
    }
    return "unknown";
}

//---------------------------------------------------------------
void state_comm(void)
{
    int32_t rpm;
    //Engine RPM
    rpm = (((m_rev_buf[1] & 0xFF)) << 0) | (((m_rev_buf[2] & 0xFF)) << 8);
    m_engine_rpm = rpm * 10;
}

void state_id_state1()
{
    int32_t tmp = 0;
    int32_t err_code = 0;

    err_code = ((m_rev_buf[3] & 0xE0) >> 5) | ((m_rev_buf[4] & 0x03) << 3);

    m_engine_status = (m_rev_buf[3] & 0x1F);
    m_error_code = (err_code & 0x1F);

    //Engine Temperature
    tmp = (((m_rev_buf[4] & 0x1C)) << 6) | (((m_rev_buf[5] & 0xFF)) << 0);
    m_engine_temp = tmp - 50;

    m_switch = ((m_rev_buf[4] & 0x60) >> 5);
}

void state_id_state2()
{
    m_rc_cur_voltage = m_rev_buf[3];
    m_pwr_cur_voltage = m_rev_buf[4];
    m_pump_cur_voltage = m_rev_buf[5];
}

void state_id_state3()
{
    int32_t press;

    m_throttle = m_rev_buf[3];
    if (m_throttle > 100) {
        m_throttle = 100;
    }

    press = ((m_rev_buf[4]) << 0) | ((m_rev_buf[5]) << 8);
    m_pressure = press * 2;
}

void state_id_state4()
{
    int32_t temp = 0;

    temp = ((m_rev_buf[3]) << 0) | ((m_rev_buf[4]) << 8);
    m_current = temp & 0x1FF;

    temp = ((m_rev_buf[5]) << 0) | (((m_rev_buf[4] & 0xFE)) << 7);
    m_thrust = temp & 0x7FFF;
}

void state_id_state5()
{
    m_pump_ign_voltage = (m_rev_buf[3]) * 2; //0.10 ~  5.00v
    m_rpm_acc = m_rev_buf[4];                //10 ~ 70

    if (m_rpm_acc < 10)
        m_rpm_acc = 10;
    if (m_rpm_acc > 70)
        m_rpm_acc = 70;
}

void state_id_state6()
{
    m_engine_max_rpm = (m_rev_buf[3]) * 1000;
    m_pump_max_voltage = m_rev_buf[4];
    m_version = (m_rev_buf[5] >> 2) & 0x3F;
    m_update_rate = (m_rev_buf[5] >> 0) & 0x03;
}

void state_id_state7()
{
    int32_t temp = 0;

    temp = ((m_rev_buf[3]) << 0) | ((m_rev_buf[4]) << 8);
    m_flow_rate = temp & 0x3FF;

    temp = ((m_rev_buf[4]) >> 2) | ((m_rev_buf[5]) << 6);
    m_flow_total = temp & 0x3FFF;
}

void state_id_state8()
{
    m_engine_idle_rpm = (m_rev_buf[3]) * 1000;
}
//---------------------------------------------------------------
void print_and_save_param()
{
    if (m_error_code != m_last_error_code) {
        printf("sw_error: %s\n", EcuStrError(m_error_code));
        m_last_error_code = m_error_code;
    }

    if (m_engine_status != m_last_engine_status) {
        printf("sw_status: %s\n", EcuStrStatus(m_engine_status));
        m_last_engine_status = m_engine_status;
    }

    m_rpm::publish((uint32_t) m_engine_rpm / 10);
    m_egt::publish((float) m_engine_temp);
    m_eng_voltage::publish((float) m_pwr_cur_voltage / 10.f);
    m_vsrv::publish((float)m_rc_cur_voltage / 10.f);
    m_pump_volt::publish((float) m_pump_cur_voltage / 10.f);
    m_thr_fb::publish((uint32_t) m_throttle);

}
//---------------------------------------------------------------
int main()
{
    frame_length = 0;

    task("on_task", 300);
    receive(port_id, "on_serial");

    m_pwr_ign(); //subscribe
    m_thr();
    m_eng_mode();

    return 0;
}
//---------------------------------------------------------------
EXPORT void on_serial(const uint8_t *data, size_t size)
{
    frame_length = 0;

    if (size != MSG_SIZE) { //MSG_SIZE
        return;
    }

    memcpy(&m_rev_buf, data, MSG_SIZE);

    if (calc_crc8(m_rev_buf, MSG_SIZE - 1) == m_rev_buf[MSG_SIZE - 1]) {
        state_comm();
        switch (m_rev_buf[0] & 0x0F) {
        case 1: { state_id_state1(); break; };
        case 2: { state_id_state2(); break; };
        case 3: { state_id_state3(); break; };
        case 4: { state_id_state4(); break; };
        case 5: { state_id_state5(); break; };
        case 6: { state_id_state6(); break; };
        case 7: { state_id_state7(); break; };
        case 8: { state_id_state8(); break; };
        }
        print_and_save_param();
    }
}

//---------------------------------------------------------------
uint8_t get_send_buf(uint8_t **ppbuf)
{
    if (ppbuf)
        *ppbuf = m_snd_buf;

    m_snd_buf[0] = 0xFF; //Header
    m_snd_buf[4] = 0xFF;

    if (m_set_ign_pump) {
        //Unlock
        m_snd_buf[1] = eCMD_UNLOCK << 4;
        m_snd_buf[2] = 0x00;
        m_snd_buf[3] = calc_crc8(&m_snd_buf[1], 2);
        //Set Param: Ignition Pump Voltage
        m_snd_buf[5] = eCMD_IGN_PUMP << 4;
        m_snd_buf[6] = m_set_ign_pump;
        m_snd_buf[7] = calc_crc8(&m_snd_buf[5], 2);
        m_set_ign_pump = 0;
        return 8;
    } else if (m_set_rpm_acc) {
        //Unlock
        m_snd_buf[1] = eCMD_UNLOCK << 4;
        m_snd_buf[2] = 0x00;
        m_snd_buf[3] = calc_crc8(&m_snd_buf[1], 2);
        //Set Param: RPM ACC
        m_snd_buf[5] = eCMD_RPM_ACC << 4;
        m_snd_buf[6] = m_set_rpm_acc;
        m_snd_buf[7] = calc_crc8(&m_snd_buf[5], 2);
        m_set_rpm_acc = 0;
        return 8;
    } else {
        //Send Control(Switch & Throttle)
        m_snd_buf[1] = eCMD_POWER << 4;
        m_snd_buf[1] |= (m_ctrl_switch & 0x03) << 2;
        m_snd_buf[1] |= (unsigned char) ((m_ctrl_throttle & 0x300) >> 8);
        m_snd_buf[2] = (unsigned char) ((m_ctrl_throttle & 0x0FF) >> 0);
        m_snd_buf[3] = calc_crc8(&m_snd_buf[1], 2);

        if (m_update_rate != m_set_update_rate && m_version >= 1) {
            switch (m_set_update_rate) {
            case eUPDATE_RATE_20Hz:
                m_snd_buf[6] = eCMD_SEND_RATE_20Hz;
                break;
            case eUPDATE_RATE_50Hz:
                m_snd_buf[6] = eCMD_SEND_RATE_50Hz;
                break;
            case eUPDATE_RATE_100Hz:
            default:
                m_snd_buf[6] = eCMD_SEND_RATE_100Hz;
                break;
            }
            m_snd_buf[5] = eCMD_SHORT << 4;
            m_snd_buf[7] = calc_crc8(&m_snd_buf[5], 2);
            m_cmd_param = 0;
            return 8;
        }
        //Send Short Command
        else if (m_cmd_param) {
            m_snd_buf[5] = eCMD_SHORT << 4;
            m_snd_buf[6] = m_cmd_param;
            m_snd_buf[7] = calc_crc8(&m_snd_buf[5], 2);
            m_cmd_param = 0;
            return 8;
        }
        return 4;
    }
}
//---------------------------------------------------------------
EXPORT void on_task()
{

    bool on_power_ignition = (bool)m_pwr_ign::value();

    //start
    if(on_power_ignition && (uint32_t)m_eng_mode::value() == mandala::eng_mode_start) {
        set_switch(eSWState::eSW_START);
        m_eng_mode::publish((uint32_t) mandala::eng_mode_auto);
        start_eng = true;
        //printf("state: %s", "START");
    }

    //ready
    if (on_power_ignition && m_switch != eSWState::eSW_START && !start_eng) {
        set_switch(eSWState::eSW_READY);
        //printf("state: %s", "READY");
    }

    //stop and ctr_throttle
    if (on_power_ignition) {
        set_throttle((int32_t) (m_thr::value() * 1000));
    } else {
        set_switch(eSWState::eSW_STOP);
        start_eng = false;
        //printf("state: %s", "STOP");
    }

    //cooling
    if (m_engine_temp > 100 && !start_eng) {
        set_switch(eSWState::eSW_READY);
        m_pwr_ign::publish((uint32_t) 1.0);
    }


    uint8_t *cmd;
    uint8_t len = get_send_buf(&cmd);
    if (len) {
        send(port_id, cmd, len, true);
    }
}
