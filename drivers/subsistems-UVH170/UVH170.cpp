#include <apx.h>

const port_id_t PORT_ID{1};
const port_id_t PORT_ID_FUEL{3};

const uint8_t PACK_SIZE_CAN{12};
const uint8_t MSG_FUEL_SIZE{9}; //FUEL

//Vesc tail
using m_vesc_tail_rpm = Mandala<mandala::est::env::usr::u5>;
using m_vesc_tail_current = Mandala<mandala::est::env::usr::u6>;
using m_vesc_tail_duty = Mandala<mandala::est::env::usr::u7>;
using m_vesc_tail_temp_fet = Mandala<mandala::est::env::usr::u8>;
using m_vesc_tail_temp_motor = Mandala<mandala::est::env::usr::u9>;
using m_vesc_tail_curr_in = Mandala<mandala::est::env::usr::u11>;

//Vesc gen
using m_vesc_gen_rpm = Mandala<mandala::sns::env::gen::rpm>;
using m_vesc_gen_curr_in = Mandala<mandala::sns::env::gen::current>;
using m_vesc_gen_motor_temp = Mandala<mandala::sns::env::gen::temp>;
using m_vesc_gen_fet_temp = Mandala<mandala::est::env::usr::u12>;

//Uvhpu
using m_uvhpu_vbat = Mandala<mandala::est::env::usr::u13>;
using m_uvhpu_vbat_filt = Mandala<mandala::est::env::usr::u14>;
using m_uvhpu_pbat = Mandala<mandala::est::env::usr::u15>;
using m_uvhpu_status = Mandala<mandala::est::env::usrc::c2>;
using m_uvhpu_ibat_filt = Mandala<mandala::est::env::usrf::f1>;
using m_uvhpu_cbat = Mandala<mandala::est::env::usrf::f10>;
using m_uvhpu_ebat = Mandala<mandala::est::env::usrf::f11>;
using m_uvhpu_ibat = Mandala<mandala::est::env::usrf::f12>;

//Engine
using m_pwr_ign = Mandala<mandala::ctr::env::pwr::eng>;
using m_sw_starter = Mandala<mandala::ctr::nav::eng::starter>;
using m_eng_ctr = Mandala<mandala::ctr::nav::eng::thr>;
using m_rpm = Mandala<mandala::est::env::usr::u4>;

// AGL
using m_agl = Mandala<mandala::est::env::usr::u1>;

//fuel pressure ADC
using m_fps_adc = Mandala<mandala::est::env::usr::u3>;
using m_fps = Mandala<mandala::sns::env::fuel::ps>;

// DUT
using m_fuel_p = Mandala<mandala::sns::env::fuel::level>;
using m_fuel_l = Mandala<mandala::est::env::usrf::f2>;
using m_warn = Mandala<mandala::est::env::usrc::c1>;

const uint16_t TASK_FUEL_MS{500}; //msec

uint8_t snd_fuel_buf[MSG_FUEL_SIZE] = {};
uint8_t ADR_FUEL = 170;
const float V_MAX1 = 16.7f;
const uint8_t TIME_SA{5}; //sec
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

_fuel fuel = _fuel(V_MAX1);

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
    uint32_t apm_hours_charged;  //MSG2
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

//UVHPU
//-------------------------------------------------------------------------------------
#define UVHPU_ID 0x80
#define UVHPU_PACK1 UVHPU_ID + 1
#define UVHPU_PACK2 UVHPU_ID + 2
#define UVHPU_PACK3 UVHPU_ID + 3
#define UVHPU_PACK4 UVHPU_ID + 4
#define UVHPU_PACK5 UVHPU_ID + 5
#define UVHPU_PACK6 UVHPU_ID + 6
#define UVHPU_PACK7 UVHPU_ID + 7

#pragma pack(1)
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

UVHPU _uvhpu{};

// AGL
//-------------------------------------------------------------------------------------
#define AGL_CAN_ID 0x00090002

//-------------   STARTER CONFIG   ---------------------------------------------------------
const uint8_t STARTER_RPM_TIME = 30;    //3 sec at 100ms interval
const uint8_t STARTER_CURRENT_TIME = 5; //0.5 sec at 100ms interval
const int32_t STARTER_RPM = 20000;      // ~1000 rpm for starter
const float STARTER_CURRENT = 20.f;     // 20A for starter
const float STARTER_THROTTLE = 0.01f;   // 1% throttle during starter

void setRPM(const uint8_t &, const int32_t &);
void setCurrent(const uint8_t &, const float &);
uint8_t currentStarterCnt = STARTER_CURRENT_TIME;
uint8_t rpmStarterCnt = STARTER_RPM_TIME;

float rpm_prev = 0.0f;
bool starter_active = false;
static uint32_t same_counter = 0;
const uint32_t SAME_LIMIT = 30; //3 sec at 100ms interval

int main()
{
    schedule_periodic(task("on_main"), 100);
    schedule_periodic(task("on_fuel"), TASK_FUEL_MS);

    task("uvhpu"); //GCS with terminal command `vmexec("uvhpu")`

    m_pwr_ign(); //subscribe
    m_sw_starter();
    m_eng_ctr();
    m_fps_adc();
    m_rpm();

    receive(PORT_ID, "on_serial");
    receive(PORT_ID_FUEL, "on_fuel_serial");
}

EXPORT void on_main()
{
    //power ignition logic
    bool on_power_ignition = (bool) m_pwr_ign::value();
    if (on_power_ignition && (uint32_t) m_sw_starter::value()) {
        starter_active = true;
    }

    if (starter_active) {
        m_eng_ctr::publish(STARTER_THROTTLE); //throttle to 1% during starter
        if (currentStarterCnt > 0) {          //current phase of starter
            setCurrent(VESC_GEN_ID, -STARTER_CURRENT);
            currentStarterCnt--;
        }
        if (currentStarterCnt == 0) { //rpm phase of starter
            if (rpmStarterCnt > 0) {
                setRPM(VESC_GEN_ID, STARTER_RPM);
                rpmStarterCnt--;
            } else {
                starter_active = false;
                currentStarterCnt = STARTER_CURRENT_TIME;
                rpmStarterCnt = STARTER_RPM_TIME;
                setRPM(VESC_GEN_ID, 0);
            }
        }
    }

    // calculate fuel pressure from ADC value
    float fuel_pressure = (m_fps_adc::value() - 0.2f) / 0.6429f; //bar
    m_fps::publish(fuel_pressure);

    //RPM anti-stuck logic: if RPM is the same for a long time and less than 500, set it to 0
    float rpm_main = m_rpm::value();
    if (rpm_main == rpm_prev) {
        same_counter++;
        if (same_counter >= SAME_LIMIT && rpm_main < 500.f) {
            m_rpm::publish(0.0f);
            same_counter = 0; // сбрасываем, чтобы не зациклиться
        }
    } else {
        rpm_prev = rpm_main;
        same_counter = 0;
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

void check_answer_time()
{
    uint32_t now = time_ms();

    m_warn::publish((fuel.time_ans + TIME_SA * 1000 < now) ? true : false);
}

EXPORT void on_fuel()
{
    check_answer_time();

    m_fuel_l::publish(fuel.liters);
    m_fuel_p::publish(fuel.percent);

    //request fuel sensors
    snd_fuel_buf[0] = 0x31;
    snd_fuel_buf[2] = 0x06;

    snd_fuel_buf[1] = ADR_FUEL;

    snd_fuel_buf[3] = calcCRC(snd_fuel_buf, 3);
    send(PORT_ID_FUEL, snd_fuel_buf, 4, true);
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

void processUVHPUackage(const uint32_t &can_id, const uint8_t *data)
{
    switch (can_id) {
    case UVHPU_PACK1: {
        _uvhpu.MSG1.vbat = (float) unpackInt16(data, 0) / 100.f;
        memcpy(&_uvhpu.MSG1.ibat, data + 2, 4);
        _uvhpu.MSG1.imon = (float) unpackInt16(data, 6) / 100.f;

        m_uvhpu_vbat::publish(_uvhpu.MSG1.vbat);
        m_uvhpu_ibat::publish(_uvhpu.MSG1.ibat);
        break;
    }
    case UVHPU_PACK2: {
        _uvhpu.MSG2.vout = (float) unpackInt16(data, 0) / 100.f;
        _uvhpu.MSG2.tbat = (float) unpackInt16(data, 2) / 100.f;
        _uvhpu.MSG2.pbat = (float) unpackInt16(data, 4);
        _uvhpu.MSG2.status = data[7];

        m_uvhpu_status::publish(_uvhpu.MSG2.status);
        m_uvhpu_pbat::publish(_uvhpu.MSG2.pbat);
        break;
    }
    case UVHPU_PACK3: {
        memcpy(&_uvhpu.MSG3.cbat, data, 8);
        m_uvhpu_cbat::publish(_uvhpu.MSG3.cbat);
        m_uvhpu_ebat::publish(_uvhpu.MSG3.ebat);
        break;
    }
    case UVHPU_PACK4: {
        memcpy(&_uvhpu.MSG4.res_bar, data, 8);
        break;
    }
    case UVHPU_PACK5: {
        memcpy(&_uvhpu.MSG5.ibat_filt, data, 8);
        m_uvhpu_ibat_filt::publish(_uvhpu.MSG5.ibat_filt);
        m_uvhpu_vbat_filt::publish(_uvhpu.MSG5.vbat_filt);
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

void setCurrent(const uint8_t &VECS_CAN_ID, const float &val)
{
    uint8_t msg[4 + 4] = {}; // ext id + DATA
    int32_t current = int32_t(val * 1000);

    msg[0] = VECS_CAN_ID;
    msg[1] = CAN_PACKET_SET_CURRENT;
    msg[3] |= 0x80; // IDE (bit 7) 1=ext,0=std;
    serializeInt(msg, 4, current);
    send(PORT_ID, msg, 8, false);
}

void setRPM(const uint8_t &VECS_CAN_ID, const int32_t &val)
{
    uint8_t msg[4 + 4] = {}; // ext id + DATA

    msg[0] = VECS_CAN_ID;
    msg[1] = CAN_PACKET_SET_RPM;
    msg[3] |= 0x80; // IDE (bit 7) 1=ext,0=std
    serializeInt(msg, 4, val);
    send(PORT_ID, msg, 8, false);
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

EXPORT void on_serial(const uint8_t *data, size_t size)
{
    //printf("%d", size);
    if (size != PACK_SIZE_CAN) {
        //return;
    }

    uint32_t can_id = (uint32_t) (data[0] + (data[1] << 8) + (data[2] << 16) + (data[3] << 24));
    can_id &= 0x7FFFFFFF; // 32nd bit is ext/std flag
    //printf("can_id:%x\n", can_id);

    uint8_t can_data[8] = {};
    for (uint8_t i = 0; i < 8; i++) {
        can_data[i] = data[4 + i]; // 4 is data position
    }

    if (can_id == AGL_CAN_ID) {
        //printf("agl");
        uint16_t raw = (uint16_t) ((data[4] << 8) + data[5]);
        float altitude = float(raw) / 100.0f;
        if (altitude > 0.1f && altitude < 40.0f)
            m_agl::publish(altitude);
        return;
    }

    //CAN ID 0xFF
    switch (can_id & 0xFF) {
    case VESC_TAIL_ID: {
        //printf("vesc tail %u", can_id);
        uint16_t msg_id = (can_id >> 8) & 0xFF;
        processVESCPackage(msg_id, can_data, &tail_data);

        m_vesc_tail_rpm::publish((float) tail_data.rpm / 11);
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

        m_vesc_gen_rpm::publish((float) abs(gen_data.rpm) / 21);
        m_vesc_gen_fet_temp::publish(gen_data.temp_fet);
        m_vesc_gen_motor_temp::publish(gen_data.temp_mot);
        m_vesc_gen_curr_in::publish(fabs(gen_data.curr_in));

        break;
    }
    }

    //CAN ID 0XFFFF
    switch (can_id & 0xFFFF) {
    case UVHPU_PACK1:
    case UVHPU_PACK2:
    case UVHPU_PACK3:
    case UVHPU_PACK4:
    case UVHPU_PACK5:
    case UVHPU_PACK6:
    case UVHPU_PACK7: {
        //printf("uvhpu %x", can_id);
        processUVHPUackage(can_id, can_data);
        break;
    }
    }
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

    float fuel_percent = (float) (data[4] | (data[5] << 8)) / 10.f;

    //printf("fuel: %.2f", fuel_percent);

    if (data[1] == ADR_FUEL) {
        fuel.set_percent(fuel_percent);
    }
}
