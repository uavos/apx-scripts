#include <apx.h>

static constexpr const port_id_t port_gill_id{21}; //U4    19200
static constexpr const port_id_t port_fan_id{22};  //U5    115200
static constexpr const port_id_t port_agl_id{23};  //U6    115200
static constexpr const port_id_t port_aux_id{24};  //CAN   1Mb

const uint8_t PACK_SIZE_GILL = 31;
const uint8_t PACK_SIZE_FAN = 10;
const uint8_t PACK_SIZE_AGL = 6;
const uint8_t PACK_SIZE_CAN = 12;

//FAN
//-------------------------------------------------------------------------------------
struct FAN_VCP_Data
{
    uint8_t temp;
    float voltage;
    float current;
    uint16_t consumption;
    uint16_t rpm;
};
uint8_t fan_tbuf[PACK_SIZE_FAN] = {};

FAN_VCP_Data fan_data{};
//-------------------------------------------------------------------------------------

//VESC
//-------------------------------------------------------------------------------------
#define VESC_TAIL_ID 0x24 // VESC ID 36
#define VESC_GEN_ID 0x25  // VESC ID 37
#define STATUS_MSG_1 0x09
#define STATUS_MSG_2 0x0E
#define STATUS_MSG_3 0x0F
#define STATUS_MSG_4 0x10
#define STATUS_MSG_5 0x1B
#define CAN_PACKET_SET_CURRENT 1
#define CAN_PACKET_SET_CURRENT_BRAKE 2
#define CAN_PACKET_SET_RPM 3

struct VESC_CAN_Data
{
    int32_t rpm;                 //MSG1
    float current;               //MSG1
    float duty;                  //MSG1
    uint32_t apm_hours;          //MSG2
    uint32_t apm_hours_charged;  //MSG1
    uint32_t watt_hours;         //MSG3
    uint32_t watt_hours_charged; //MSG3
    float temp_fet;              //MSG4
    float temp_mot;              //MSG4
    float curr_in;               //MSG4
    float pid_pos_now;           //MSG4
    float voltage;               //MSG5
    uint32_t tacho;              //MSG5
};

VESC_CAN_Data tail_data{};
VESC_CAN_Data gen_data{};
//-------------------------------------------------------------------------------------

//ECU
//-------------------------------------------------------------------------------------
#define ECU_COMM_ID 0x80
#define ECU_CTRL_ID 0x060A
#define ECU_TCP_ID 0x0614
#define ECU_MAS_ID 0x061E
#define ECU_FSP_ID 0x0632
#define ECU_STATUS_ID 0x063C
//-------------------------------------------------------------------------------------

//MCELL
//-------------------------------------------------------------------------------------
#define MCELL_ID 0x0100
#define MCELL_PACK1 MCELL_ID + 1
#define MCELL_PACK2 MCELL_ID + 2
#define MCELL_PACK3 MCELL_ID + 3
#define MCELL_PACK4 MCELL_ID + 4

struct CELL
{
    int16_t c1;
    int16_t c2;
    int16_t c3;
    int16_t c4;
};

struct MCELL
{
    struct
    {
        float v_bat{0.f};
        float t_bat{0.f};
        float t_pcb{0.f};
        uint8_t state{0};
    } MSG1;
    CELL MSG2;
    CELL MSG3;
    CELL MSG4;
    CELL MSG5;
};

MCELL _mcel = {};

//-------------------------------------------------------------------------------------

//UVHPU
//-------------------------------------------------------------------------------------
#define UVHPU_ID 0x80
#define UVHPU_PACK1 UVHPU_ID + 1
#define UVHPU_PACK2 UVHPU_ID + 2
#define UVHPU_PACK3 UVHPU_ID + 3
#define UVHPU_PACK4 UVHPU_ID + 4
#define UVHPU_PACK5 UVHPU_ID + 5
//-------------------------------------------------------------------------------------

//Gill
using m_gill_fuel = Mandala<mandala::sns::env::fuel::level>;

//Fan
using m_fan_temp = Mandala<mandala::est::env::usrf::f3>;        // +
using m_fan_voltage = Mandala<mandala::est::env::usrf::f4>;     // +
using m_fan_current = Mandala<mandala::est::env::usrf::f5>;     // +
using m_fan_consumption = Mandala<mandala::est::env::usrw::w6>; // +
using m_fan_rpm = Mandala<mandala::est::env::usrw::w7>;         // +

//uLanding
using m_agl = Mandala<mandala::est::env::usrf::f1>; // +
using m_snr = Mandala<mandala::est::env::usrf::f2>; // +

//Vesc tail
using m_vesc_tail_rpm = Mandala<mandala::est::env::usrf::f6>;         // +
using m_vesc_tail_current = Mandala<mandala::est::env::usrf::f7>;     // +
using m_vesc_tail_duty = Mandala<mandala::est::env::usrf::f8>;        // +
using m_vesc_tail_temp_fet = Mandala<mandala::est::env::usrf::f9>;    // +
using m_vesc_tail_temp_motor = Mandala<mandala::est::env::usrf::f10>; // +
using m_vesc_tail_curr_in = Mandala<mandala::est::env::usrf::f11>;    // +

//Vesc gen
using m_vesc_gen_rpm = Mandala<mandala::sns::env::gen::rpm>;
using m_vesc_gen_curr_in = Mandala<mandala::sns::env::gen::current>;
using m_vesc_gen_motor_temp = Mandala<mandala::sns::env::gen::temp>;
using m_vesc_gen_fet_temp = Mandala<mandala::est::env::usrf::f12>;

//Ecu
using m_ecu_speed = Mandala<mandala::est::env::usrw::w3>;              // +
using m_ecu_tps = Mandala<mandala::est::env::usr::u11>;                // +
using m_ecu_cylinder_head_temp1 = Mandala<mandala::est::env::usr::u4>; // +
using m_ecu_cylinder_head_temp2 = Mandala<mandala::est::env::usr::u5>; // +
using m_ecu_egt1 = Mandala<mandala::est::env::usrw::w4>;               // +
using m_ecu_egt2 = Mandala<mandala::est::env::usrw::w5>;               // +
using m_ecu_inlet_air_temp = Mandala<mandala::est::env::usr::u6>;      // +
using m_ecu_baromeric_pressure = Mandala<mandala::est::env::usr::u7>;  // +
using m_ecu_coolant_temp = Mandala<mandala::est::env::usr::u8>;        // +
using m_ecu_fuel_pressure = Mandala<mandala::est::env::usr::u9>;       // +
using m_ecu_fuel_consumotion = Mandala<mandala::est::env::usr::u10>;   // +

using m_ecu_st_sns = Mandala<mandala::est::env::usrw::w11>; // +
using m_ecu_st_act = Mandala<mandala::est::env::usrw::w12>; // +
using m_ecu_st_eng = Mandala<mandala::est::env::usrw::w13>; // +
using m_ecu_st_fl = Mandala<mandala::est::env::usrw::w14>;  // +
using m_ecu_st_pump = Mandala<mandala::est::env::usrb::b7>; // +

//Start ENG
using m_pwr_ign = Mandala<mandala::ctr::env::pwr::eng>;
using m_eng_mode = Mandala<mandala::cmd::nav::eng::mode>;
bool start_eng{};
uint32_t start_time_eng{};

//-------------------------------------------------------------------------------------
uint8_t update_crc8(uint8_t data, uint8_t crc)
{
    data ^= crc;
    for (uint8_t i = 0; i < 8; i++)
        data = uint8_t((data & 0x80) ? 0x07 ^ (data << 1) : (data << 1));
    return data;
}

uint8_t get_crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++)
        crc = update_crc8(data[i], crc);
    return crc & 0xFF;
}

int32_t decodeValue(const uint8_t *data, int8_t index, int8_t len)
{
    int32_t value = 0;
    uint32_t mul = 1;

    for (int8_t i = len - 1; i >= 0; i--) {
        uint8_t byte = data[index + i];
        if (byte >= 48 && byte <= 57)
            value += (byte - 48) * mul;
        else if (byte >= 65 && byte <= 70)
            value += (byte - 55) * mul;
        else
            return -1;
        mul *= 16;
    }

    return value;
}

void serializeInt(uint8_t *data, uint8_t index, int32_t value)
{
    for (uint8_t i = 0; i < 4; i++) {
        uint8_t shift = 8 * (4 - i - 1);
        data[index + i] = (value >> shift) & 0xFF;
    }
}

float deserializeFloat2B(uint8_t b0, uint8_t b1)
{
    float sign = (b0 & 0x80 || b1 & 0x80) ? -1.f : 1.f;
    float i = b0 & 0x80 ? 0x100 - b0 : b0;
    float f = b1 & 0x80 ? 0x100 - b1 : b1;

    return sign * (i + f * 0.01f);
}
//-------------------------------------------------------------------------------------

void ECUDemandRequest(uint8_t);
void ECUEnableDisable(bool);
void setRPM(const uint8_t &, const int32_t &);
void setCurrent(const uint8_t &, const float &);

int main()
{
    ECUDemandRequest(0);     // 0% throttle
    ECUEnableDisable(false); // Ignition OFF

    receive(port_gill_id, "on_serial_gill"); //1 Hz
    receive(port_fan_id, "on_serial_fan");   //30 Hz
    receive(port_agl_id, "on_serial_agl");   //100 Hz
    receive(port_aux_id, "on_can_aux");      //ECU 60Hz | TAIL 20Hz | GEN 20Hz | MCELL 30Hz | UVHPU 50Hz

    task("on_request_gill", 1000);
    task("on_start_eng", 100);

    m_pwr_ign(); //subscribe
    m_eng_mode();

    start_time_eng = time_ms();

    printf("IFC Script ready...");

    return 0;
}

EXPORT void on_start_eng()
{
    bool on_power_ignition = (bool) m_pwr_ign::value();
    if ((uint32_t) m_eng_mode::value() == mandala::eng_mode_start && !on_power_ignition) {
        m_eng_mode::publish((uint32_t) mandala::eng_mode_auto);
        start_eng = false;
        return;
    }

    if (on_power_ignition && !start_eng && (uint32_t) m_eng_mode::value() == mandala::eng_mode_start) {
        start_eng = true;
        start_time_eng = time_ms();
        //printf("IFC ENG start...");
    }

    if (on_power_ignition && start_eng && (time_ms() - start_time_eng) < 3000u) {
        setRPM(VESC_GEN_ID, -40000);
        //printf("IFC ENG spin...");
    } else if (start_eng) {
        m_eng_mode::publish((uint32_t) mandala::eng_mode_auto);
        start_eng = false;
        //printf("IFC ENG off...");
    }
}

uint32_t count_fr{0};
EXPORT void on_request_gill()
{
    //printf("Aux CAN Hz %u", count_fr);
    //count_fr = 0;

    const uint8_t tbuf[] = {0x41, 0x0d};
    send(port_gill_id, tbuf, sizeof(tbuf), false);
}

EXPORT void on_serial_gill(const uint8_t *data, size_t size)
{
    /*
    Data from GILL
    A,0000,0037,00C9,00AC,
    41 0d 0a 0d 02 41 2c 30 30 30 30 2c 30 30 33 37 2c 30 30 43 39 2c 30 30 41 43 2c 03 0a 0d 3e
    */

    if (size != PACK_SIZE_GILL) {
        return;
    }

    const uint8_t start_fl = 12; // Starting byte fuel lavel
    uint8_t data_full[4] = {};

    memcpy(data_full, data + start_fl, 4);

    int32_t full = decodeValue(data_full, 0, 4);
    if (full > 0) {
        float fuel = (float) full * 0.0059642f - 0.333996f;
        //printf("GILL %.2f", fuel);
        m_gill_fuel::publish(fuel);
    }
}

EXPORT void on_serial_fan(const uint8_t *data, size_t size)
{
    if (size != PACK_SIZE_FAN) {
        //printf("FAN: size err...");
        return;
    }

    memcpy(fan_tbuf, data, size);

    if (get_crc8(fan_tbuf, PACK_SIZE_FAN - 1) != fan_tbuf[PACK_SIZE_FAN - 1]) {
        //printf("FAN: CRC err...");
        return;
    }

    fan_data.temp = data[0];
    fan_data.voltage = float((fan_tbuf[1] << 8) | (fan_tbuf[2])) / 100.f;
    fan_data.current = float((fan_tbuf[3] << 8) | (fan_tbuf[4])) / 100.f;
    fan_data.consumption = uint16_t((fan_tbuf[5] << 8) | (fan_tbuf[6]));
    fan_data.rpm = uint16_t((fan_tbuf[7] << 8) | (fan_tbuf[8])) * 100u;

    m_fan_temp::publish((float) fan_data.temp);
    m_fan_voltage::publish((float) fan_data.voltage);
    m_fan_current::publish((float) fan_data.current);
    m_fan_consumption::publish((uint32_t) fan_data.consumption);
    m_fan_rpm::publish((uint32_t) fan_data.rpm);
}

EXPORT void on_serial_agl(const uint8_t *data, size_t size)
{
    if (size != PACK_SIZE_AGL) {
        return;
    }

    if (data[0] != 0xFE) {
        return;
    }

    const uint8_t crc = (data[1] + data[2] + data[3] + data[4]) & 0xFF;
    if (crc != data[5]) {
        return;
    }

    float agl = float((data[3] << 8) | data[2]) / 100.f;
    uint8_t snr = data[4];

    m_agl::publish(agl);
    m_snr::publish((uint32_t) snr);
}

void setCurrent(const uint8_t &VECS_CAN_ID, const float &val)
{
    uint8_t msg[4 + 4] = {}; // ext id + DATA
    int32_t current = int32_t(val * 1000);

    msg[0] = VECS_CAN_ID;
    msg[1] = CAN_PACKET_SET_CURRENT;
    msg[3] |= 0x80; // IDE (bit 7) 1=ext,0=std;
    serializeInt(msg, 4, current);
    send(port_aux_id, msg, 8, false);
}

void setRPM(const uint8_t &VECS_CAN_ID, const int32_t &val)
{
    uint8_t msg[4 + 4] = {}; // ext id + DATA

    msg[0] = VECS_CAN_ID;
    msg[1] = CAN_PACKET_SET_RPM;
    msg[3] |= 0x80; // IDE (bit 7) 1=ext,0=std
    serializeInt(msg, 4, val);
    send(port_aux_id, msg, 8, false);
}

void processVESCPackage(const uint32_t &msg_id, const uint8_t *data, VESC_CAN_Data *vesc_data)
{
    switch (msg_id) {
    case STATUS_MSG_1: {
        vesc_data->rpm = int32_t((data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]) / 14;
        vesc_data->current = float(int16_t((data[4] << 8) | data[5])) / 10.f;
        vesc_data->duty = float(int16_t((data[6] << 8) | data[7])) / 10.f;
        break;
    }
    case STATUS_MSG_2: {
        vesc_data->apm_hours = uint32_t((data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]);
        vesc_data->apm_hours_charged = uint32_t((data[4] << 24) | (data[5] << 16) | (data[6] << 8) | data[7]);
        break;
    }
    case STATUS_MSG_3: {
        vesc_data->watt_hours = uint32_t((data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]);
        vesc_data->watt_hours_charged = uint32_t((data[4] << 24) | (data[5] << 16) | (data[6] << 8) | data[7]);
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

void ECUEnableDisable(bool cmd_enable)
{
    uint8_t msg[4 + 8] = {}; // ext id + DATA

    msg[0] = ECU_COMM_ID;
    msg[3] |= 0x80; // IDE (bit 7) 1=ext,0=std
    msg[4] = 0x01;
    msg[11] = cmd_enable;
    send(port_aux_id, msg, 12, false);
}

void ECUDemandRequest(uint8_t cmd_rpm)
{
    uint8_t msg[4 + 8] = {}; // ext id + DATA

    msg[0] = ECU_COMM_ID;
    msg[3] |= 0x80; // IDE (bit 7) 1=ext,0=std
    msg[4] = 0x02;
    msg[10] = cmd_rpm >> 8;
    msg[11] = cmd_rpm;
    send(port_aux_id, msg, 12, false);
}

void processECUPackage(const uint32_t &can_id, const uint8_t *data)
{
    switch (can_id) {
    case ECU_CTRL_ID: {
        uint16_t speed = uint16_t((data[6] << 8) | data[7]);
        //uint16_t demand = uint16_t((data[4] << 8) | data[5]);
        uint16_t tps = uint16_t((data[2] << 8) | data[3]);
        //uint16_t throttle = uint16_t((data[0] << 8) | data[1]);

        m_ecu_tps::publish((float) tps / 10.f);
        m_ecu_speed::publish((uint32_t) speed);
        break;
    }
    case ECU_TCP_ID: {
        uint16_t cylinder_head_temp1 = uint16_t((data[6] << 8) | data[7]);
        uint16_t cylinder_head_temp2 = uint16_t((data[4] << 8) | data[5]);
        uint16_t egt1 = uint16_t((data[2] << 8) | data[3]);
        uint16_t egt2 = uint16_t((data[0] << 8) | data[1]);

        m_ecu_cylinder_head_temp1::publish((float) cylinder_head_temp1);
        m_ecu_cylinder_head_temp2::publish((float) cylinder_head_temp2);
        m_ecu_egt1::publish((uint32_t) egt1);
        m_ecu_egt2::publish((uint32_t) egt2);
        break;
    }
    case ECU_MAS_ID: {
        uint16_t inlet_air_temp = uint16_t((data[6] << 8) | data[7]);
        uint16_t barometric_pressure = uint16_t((data[4] << 8) | data[5]);
        //uint16_t power = uint16_t((data[2] << 8) | data[3]);
        uint16_t coolant_temp = uint16_t((data[0] << 8) | data[1]);

        m_ecu_inlet_air_temp::publish((float) inlet_air_temp / 10.f);
        m_ecu_baromeric_pressure::publish((float) barometric_pressure / 100.f);
        m_ecu_coolant_temp::publish((uint32_t) coolant_temp);
        break;
    }
    case ECU_FSP_ID: {
        //uint16_t pump_speed = uint16_t((data[6] << 8) | data[7]);
        uint16_t fuel_pressure = uint16_t((data[4] << 8) | data[5]);
        uint16_t fuel_consumotion = uint16_t((data[3] << 8) | data[2]);

        m_ecu_fuel_pressure::publish((float) fuel_pressure / 100.f);
        m_ecu_fuel_consumotion::publish((uint32_t) fuel_consumotion);
        break;
    }
    case ECU_STATUS_ID: {
        uint16_t st_sns = uint16_t((data[6] << 8) | data[7]);
        uint16_t st_act = uint16_t((data[4] << 8) | data[5]);
        uint16_t st_eng = uint16_t((data[2] << 8) | data[3]);
        uint16_t st_fl = uint16_t((data[0] << 8) | data[1]);

        m_ecu_st_sns::publish((uint32_t) st_sns);
        m_ecu_st_act::publish((uint32_t) st_act);
        m_ecu_st_eng::publish((uint32_t) st_eng);
        m_ecu_st_fl::publish((uint32_t) st_fl);

        uint8_t status_pump = 0;
        if (st_fl & 0x02) {
            status_pump = 1;
        } else if (st_fl & 0x04) {
            status_pump = 2;
        } else if (st_fl & 0x06) {
            status_pump = 3;
        }

        m_ecu_st_pump::publish((uint32_t) status_pump);
        break;
    }
    }
}

void processMCELLPackage(const uint32_t &can_id, const uint8_t *data)
{
    switch (can_id) {
    case MCELL_PACK1: {
        _mcel.MSG1.v_bat = deserializeFloat2B(data[0], data[1]);
        _mcel.MSG1.t_bat = deserializeFloat2B(data[2], data[3]);
        _mcel.MSG1.t_pcb = deserializeFloat2B(data[4], data[5]);
        _mcel.MSG1.state = data[6];
        /*
        printf("v_bat %.2f", _mcel.MSG1.v_bat);
        printf("t_bat %.2f", _mcel.MSG1.t_bat);
        printf("t_pcb %.2f", _mcel.MSG1.t_pcp);
        printf("state %u", _mcel.MSG1.state);
        */
        break;
    }
    case MCELL_PACK2: {
        memcpy(&_mcel.MSG2.c1, data, 8);
        /*
        printf("C[0] %.2f", _mcel.MSG2.c1 / 1000.f);
        printf("C[1] %.2f", _mcel.MSG2.c2 / 1000.f);
        printf("C[2] %.2f", _mcel.MSG2.c3 / 1000.f);
        printf("C[3] %.2f", _mcel.MSG2.c4 / 1000.f);
        */
        break;
    }
    case MCELL_PACK3: {
        memcpy(&_mcel.MSG3.c1, data, 8);
        /*
        printf("C[4] %.2f", _mcel.MSG3.c1 / 1000.f);
        printf("C[5] %.2f", _mcel.MSG3.c2 / 1000.f);
        printf("C[6] %.2f", _mcel.MSG3.c3 / 1000.f);
        printf("C[7] %.2f", _mcel.MSG3.c4 / 1000.f);
        */
        break;
    }
    case MCELL_PACK4: {
        memcpy(&_mcel.MSG4.c1, data, 8);
        /*
        printf("C[8] %.2f", _mcel.MSG4.c1 / 1000.f);
        printf("C[9] %.2f", _mcel.MSG4.c2 / 1000.f);
        printf("C[10] %.2f", _mcel.MSG4.c3 / 1000.f);
        printf("C[11] %.2f", _mcel.MSG4.c4 / 1000.f);
        */
        break;
    }
    }
}

void processUVHPUackage(const uint32_t &can_id, const uint8_t *data) {}

EXPORT void on_can_aux(const uint8_t *data, size_t size)
{
    if (size != PACK_SIZE_CAN) {
        return;
    }

    uint32_t can_id = (uint32_t) (data[0] + (data[1] << 8) + (data[2] << 16) + (data[3] << 24));
    can_id &= 0x7FFFFFFF; // 32nd bit is ext/std flag

    uint8_t can_data[8] = {};
    for (uint8_t i = 0; i < 8; i++) {
        can_data[i] = data[4 + i]; // 4 is data position
    }

    //CAN ID 0xFF
    switch (can_id & 0xFF) {
    case VESC_TAIL_ID: {
        //printf("vesc tail %u", can_id);
        uint16_t msg_id = (can_id >> 8) & 0xFF;
        processVESCPackage(msg_id, can_data, &tail_data);

        m_vesc_tail_rpm::publish((float) tail_data.rpm);
        m_vesc_tail_current::publish(tail_data.current);
        m_vesc_tail_duty::publish(tail_data.duty);
        m_vesc_tail_temp_fet::publish(tail_data.temp_fet);
        m_vesc_tail_temp_motor::publish(tail_data.temp_mot);
        m_vesc_tail_curr_in::publish(tail_data.curr_in);

        break;
    }
    case VESC_GEN_ID: {
        //printf("vesc gen %u", can_id);
        uint16_t msg_id = (can_id >> 8) & 0xFF;
        processVESCPackage(msg_id, can_data, &gen_data);

        m_vesc_gen_rpm::publish((float) gen_data.rpm);
        m_vesc_gen_fet_temp::publish(gen_data.temp_fet);
        m_vesc_gen_motor_temp::publish(gen_data.temp_mot);
        m_vesc_gen_curr_in::publish(gen_data.curr_in);

        break;
    }
    }

    //CAN ID 0XFFFF
    switch (can_id & 0xFFFF) {
    case ECU_CTRL_ID:
    case ECU_TCP_ID:
    case ECU_MAS_ID:
    case ECU_FSP_ID:
    case ECU_STATUS_ID: {
        //printf("ecu %u", can_id);
        processECUPackage(can_id, can_data);
        break;
    }
    case MCELL_PACK1:
    case MCELL_PACK2:
    case MCELL_PACK3:
    case MCELL_PACK4: {
        //printf("mcell %u", can_id);
        processMCELLPackage(can_id, can_data);
        break;
    }
    case UVHPU_PACK1:
    case UVHPU_PACK2:
    case UVHPU_PACK3:
    case UVHPU_PACK4:
    case UVHPU_PACK5: {
        //printf("uvhpu %u", can_id);
        processUVHPUackage(can_id, can_data);
        break;
    }
    }
}
