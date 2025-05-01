#include <apx.h>

#define NODE_LEFT TRUE //ifc-l
//#define NODE_RIGHT TRUE //ifc-r

#if defined NODE_LEFT
constexpr const char *txt_dev = "L";           //L
constexpr const port_id_t PORT_ID_CAN_AUX{20}; //L=20 //CAN   1Mb
#endif

#if defined NODE_RIGHT
constexpr const char *txt_dev = "R";           //R
constexpr const port_id_t PORT_ID_CAN_AUX{70}; //R=70 //CAN   1Mb
#endif

constexpr const uint8_t NODE_WING_L_ID{1};
constexpr const uint8_t NODE_WING_R_ID{2};

constexpr const uint8_t TASK_MAIN_MS{50};          //20Hz
constexpr const uint8_t TASK_SAVE_MANDALA_MS{100}; //100Hz

//--------------------------ports--------------------------
constexpr const port_id_t PORT_ID_AGL{5};
constexpr const port_id_t PORT_ID_GCU_R{41};
constexpr const port_id_t PORT_ID_WING{43};
//constexpr const port_id_t PORT_ID_MHX{44};
//---------------------------------------------------------

//--------------------------size---------------------------
constexpr const uint8_t CNT_DEV{2};
constexpr const uint8_t PACK_CAN_SIZE{12};
constexpr const uint8_t PACK_WING_SIZE{13};
constexpr const uint8_t MSG7_ID{6}; //for nav-l and nav-r wing
//---------------------------------------------------------

//--------------------------pu-----------------------------
constexpr const uint16_t PU_ID{128};
constexpr const uint16_t PU_SHIFT{20};
constexpr const uint16_t PU_PACK1{PU_ID + 1};
constexpr const uint16_t PU_PACK2{PU_ID + 2};
constexpr const uint16_t PU_PACK3{PU_ID + 3};
constexpr const uint16_t PU_PACK4{PU_ID + 4};
constexpr const uint16_t PU_PACK5{PU_ID + 5};
constexpr const uint16_t PU_PACK6{PU_ID + 6};
constexpr const uint16_t PU_PACK7{PU_ID + 7};

constexpr const uint16_t PU_CMD_HEATER{PU_ID + 10};
//constexpr const uint16_t PU_CMD_PUMP{PU_ID + 11};
constexpr const uint16_t PU_CMD_PON{PU_ID + 12};
//constexpr const uint16_t PU_CMD_RB{PU_ID + 13};

//data
#pragma pack(1)
struct PU
{
    struct
    {
        float vbat;
        float ibat;
        float imon;
    } msg1;
    struct
    {
        float vout;
        float tbat;
        float pbat;
        uint8_t status;
    } msg2;
    struct
    {
        float cbat;
        float ebat;
    } msg3;
    struct
    {
        float res_bar;
        float v_res;
    } msg4;
    struct
    {
        float ibat_flt;
        float vbat_flt;
    } msg5;
    struct
    {
        float cbat_res;
        float ebat_res;
    } msg6;
    struct
    {
        int16_t life_cycles;
        float cbat_mod;
        float h_pwr;
    } msg7;
};
#pragma pack()

PU _pu[CNT_DEV] = {};

//---------------------------------------------------------

//--------------------------mc-----------------------------
constexpr const uint16_t MC_ID{256};
constexpr const uint16_t MC_SHIFT{20};
constexpr const uint16_t MC_PACK1{MC_ID + 1};
constexpr const uint16_t MC_PACK2{MC_ID + 2};
constexpr const uint16_t MC_PACK3{MC_ID + 3};

#pragma pack(1)
struct MC
{
    float vbat;
    float tbat;
    float tpcb;
    uint8_t status;
    uint16_t cell[8] = {};
    float cell_volt(const uint8_t &cell_idx) { return cell[cell_idx] / 1000.f; };
};
#pragma pack()

MC _mc[CNT_DEV] = {};
//---------------------------------------------------------

//--------------------------sp-----------------------------
constexpr const uint16_t SP_ID{512};
constexpr const uint16_t SP_SHIFT{20};
constexpr const uint16_t SP_PACK1{SP_ID + 1};
constexpr const uint16_t SP_PACK2{SP_ID + 2};
constexpr const uint16_t SP_POWER_ON{SP_ID + 6};
constexpr const uint16_t SP_SAVE_K{SP_ID + 7};

constexpr const uint16_t SP_CMD_POWER_ON{SP_ID + 10};
constexpr const uint16_t SP_CMD_K{SP_ID + 11};

#pragma pack(1)
struct SP
{
    struct
    {
        float vin;
        float vout;
        float temp;
        uint8_t status;
    } msg1;

    struct
    {
        float cin;
        float cout;
    } msg2;
};
#pragma pack()

SP _sp[CNT_DEV] = {};
bool sp_state{false};
//---------------------------------------------------------

uint8_t size_error_wing = 0;
//----------------------------------------------------------

//-------------------------Wing-----------------------------
#pragma pack(1)
struct WING_DATA
{
    uint16_t voltage[2];
    int8_t volz_temp[2];
    int16_t volz_pos[2];
    int8_t gyro_temp;
};
#pragma pack()
#pragma pack()

WING_DATA _wing_l = {};
WING_DATA _wing_r = {};
//----------------------------------------------------------

//----------------------Temperature-------------------------
constexpr float VREF{3.f};
constexpr float RPUP_PT1000{3160.f};

enum class TSType {
    PT1000,
    NTC3988,
    NTC3570,
};

constexpr uint8_t SIZE_PT1000{23};
constexpr float RES_PT1000[SIZE_PT1000] = {723.3453f,  763.2784f,  803.0628f,  842.7065f,  882.2166f,  921.5990f,
                                           960.8588f,  1000.f,     1039.0252f, 1077.9350f, 1116.7292f, 1155.4080f,
                                           1193.9713f, 1232.4190f, 1270.7513f, 1308.9680f, 1347.0693f, 1385.0550f,
                                           1422.9253f, 1460.6800f, 1498.3193f, 1535.8430f, 1573.2513f};

constexpr float TEMP_PT1000[SIZE_PT1000] = {-70.f, -60.f, -50.f, -40.f, -30.f, -20.f, -10.f, 0.f,
                                            10.f,  20.f,  30.f,  40.f,  50.f,  60.f,  70.f,  80.f,
                                            90.f,  100.f, 110.f, 120.f, 130.f, 140.f, 150.f};
//----------------------------------------------------------

//----------------------saveToMandala----------------------
//pu
using m_pu_vbat = Mandala<mandala::est::env::usr::u3>;
using m_pu_temp = Mandala<mandala::est::env::usrw::w3>;
using m_pu_pwr = Mandala<mandala::est::env::usr::u4>;
using m_pu_status = Mandala<mandala::est::env::usrc::c7>;
using m_pu_cbat_flt = Mandala<mandala::est::env::usrf::f5>;
using m_pu_vbat_flt = Mandala<mandala::est::env::usrf::f6>;
using m_pu_h_pwr = Mandala<mandala::est::env::usrc::c10>;

//mc
using m_mc_vbat = Mandala<mandala::est::env::usrf::f7>;
using m_mc_tbat = Mandala<mandala::est::env::usrw::w4>;
using m_mc_tpcb = Mandala<mandala::est::env::usrw::w5>;
using m_mc_status = Mandala<mandala::est::env::usrc::c8>;

//sp
using m_sp_vin = Mandala<mandala::est::env::usrf::f8>;
using m_sp_vout = Mandala<mandala::est::env::usrf::f9>;
using m_sp_temp = Mandala<mandala::est::env::usr::u1>;
using m_sp_status = Mandala<mandala::est::env::usrc::c9>;
using m_sp_cin = Mandala<mandala::est::env::usr::u2>;
using m_sp_cout = Mandala<mandala::est::env::usr::u9>;

//nav
using m_navl_temp = Mandala<mandala::est::env::usr::u10>;
using m_navr_temp = Mandala<mandala::est::env::usr::u11>;

//adc
using m_pt1000 = Mandala<mandala::est::env::usrf::f14>; //adc pt1000

//cmd
using m_pu_cmd_hpwr = Mandala<mandala::est::env::usrc::c11>;    //pu
using m_sp_pwr_ign = Mandala<mandala::ctr::env::pwr::eng>;      //sp
using m_sp_cmd_k = Mandala<mandala::ctr::env::tune::t15>;       //sp
using m_adc_pt1000_raw = Mandala<mandala::est::env::usrx::x14>; //adc pt1000

int main()
{
    m_sp_pwr_ign();
    m_adc_pt1000_raw();

    //task("pu_on");  //GCS with terminal command `vmexec("pu_on")`
    //task("pu_off"); //GCS with terminal command `vmexec("pu_off")`
    //task("sp_on");  //GCS with terminal command `vmexec("sp_on")`
    //task("sp_off"); //GCS with terminal command `vmexec("sp_off")`

    m_pu_cmd_hpwr("on_cmd_hpwr");
    m_sp_cmd_k("on_cmd_k");

#if defined NODE_LEFT
    task("wing_l"); //GCS with terminal command `vmexec("wing_l")`

    task("mc_1"); //GCS with terminal command `vmexec("mc_1")`
    task("mc_2"); //GCS with terminal command `vmexec("mc_2")`

    task("pu_1"); //GCS with terminal command `vmexec("pu_1")`
    task("pu_2"); //GCS with terminal command `vmexec("pu_2")`

    //task("sp_1"); //GCS with terminal command `vmexec("sp_1")`
    //task("sp_2"); //GCS with terminal command `vmexec("sp_2")`

#endif

#if defined NODE_RIGHT
    task("wing_r"); //GCS with terminal command `vmexec("wing_r")`

    task("mc_3"); //GCS with terminal command `vmexec("mc_3")`
    task("mc_4"); //GCS with terminal command `vmexec("mc_4")`

    task("pu_3"); //GCS with terminal command `vmexec("pu_3")`
    task("pu_4"); //GCS with terminal command `vmexec("pu_4")`

    //task("sp_3"); //GCS with terminal command `vmexec("sp_3")`
    //task("sp_4"); //GCS with terminal command `vmexec("sp_4")`
#endif

    task("on_main", TASK_MAIN_MS);                 //20 Hz
    task("on_save_mandala", TASK_SAVE_MANDALA_MS); //10 Hz

    receive(PORT_ID_CAN_AUX, "canAuxHandler");
    receive(PORT_ID_GCU_R, "gcuHandler");
    receive(PORT_ID_WING, "wingHandler");
    receive(PORT_ID_AGL, "aglHandler");

    printf("IFC:%s Script ready...\n", txt_dev);

    return 0;
}

template<typename T>
T limit(T value, T min, T max)
{
    if (value < min)
        return min;
    if (value > max)
        return max;
    return value;
}

template<typename T>
const T interpolate(const T &value, const T &x_low, const T &x_high, const T &y_low, const T &y_high)
{
    if (x_low == x_high) {
        return y_low;
    }

    if ((x_low < x_high && value <= x_low) || (x_low > x_high && value >= x_low)) {
        return y_low;

    } else if ((x_low < x_high && value >= x_high) || (x_low > x_high && value <= x_high)) {
        return y_high;
    }

    T a = (y_high - y_low) / (x_high - x_low);
    T b = y_low - (a * x_low);
    return (a * value) + b;
}

template<typename T, size_t N>
const T interpolateNXY(const T &value, const T (&x)[N], const T (&y)[N])
{
    size_t index = 0;

    if (x[0] < x[N - 1]) {
        // x increasing
        while ((value > x[index + 1]) && (index < (N - 2))) {
            index++;
        }
    } else {
        // x decreasing
        while ((value < x[index + 1]) && (index < (N - 2))) {
            index++;
        }
    }

    return interpolate(value, x[index], x[index + 1], y[index], y[index + 1]);
}

float getTemperature(const float &vin, const TSType &type, const float &vref, const float &rpup)
{
    if (vin < 0.f || vin > vref) {
        return 0.f;
    }

    float res = (vin * rpup) / (vref - vin);

    float temp = 0.f;

    switch (type) {
    case TSType::PT1000: {
        temp = interpolateNXY(res, RES_PT1000, TEMP_PT1000);
        break;
    }
    case TSType::NTC3988: {
        temp = 0.f;
        break;
    }
    case TSType::NTC3570: {
        temp = 0.f;
        break;
    }
    }
    return temp;
}

uint8_t calcTelemetryCRC(const uint8_t *data, uint8_t size)
{
    uint8_t crc = 0xFF;
    for (uint8_t i = 0; i < size; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++)
            crc = crc & 0x80 ? (uint8_t) (crc << 1) ^ 0x31 : (uint8_t) (crc << 1);
    }
    return crc;
}

int16_t unpackInt16(const uint8_t *data, uint8_t index)
{
    return (int16_t) (data[index] | (data[index + 1] << 8));
}

void saveDataToMandala() {}

EXPORT void sendCmdToCan(const uint32_t &can_id, const uint8_t *data, const uint8_t &size)
{
    uint8_t msg[PACK_CAN_SIZE] = {};

    msg[0] = (uint8_t) can_id;         //ID_0_7
    msg[1] = (uint8_t) (can_id >> 8);  //ID_8_15
    msg[2] = (uint8_t) (can_id >> 16); //ID_16_23
    msg[3] = (uint8_t) (can_id >> 24); //ID_24_31

    if (can_id > 0x7FF)
        msg[3] = msg[3] | 0x80;

    for (uint8_t i = 0; i < size; i++) {
        msg[4 + i] = data[i];
    }

    send(PORT_ID_CAN_AUX, msg, 4 + size, false);
    send(PORT_ID_CAN_AUX + 1, msg, 4 + size, true);
}

void pu_cmd_power_on(const uint8_t &value)
{
    uint8_t msg[1] = {value};

    sendCmdToCan(PU_CMD_PON, msg, 1);
    sendCmdToCan(PU_CMD_PON + PU_SHIFT, msg, 1);
}

void pu_cmd_heater(const uint8_t &dev_id, const uint8_t &value)
{
    const uint16_t cmd = value * 100;

    uint8_t msg[2] = {};
    msg[0] = (uint8_t) cmd;
    msg[1] = cmd >> 8;

    sendCmdToCan(PU_CMD_HEATER + dev_id * PU_SHIFT, msg, 2);
}

void sp_cmd_power_on(const uint8_t &value)
{
    uint8_t msg[1] = {value};

    sendCmdToCan(SP_CMD_POWER_ON, msg, 1);
    sendCmdToCan(SP_CMD_POWER_ON + SP_SHIFT, msg, 1);
}

void sp_cmd_k(const float &value)
{
    uint8_t msg[4] = {};
    memcpy(msg, &value, sizeof(float));

    sendCmdToCan(SP_CMD_K, msg, 1);
    sendCmdToCan(SP_CMD_K + SP_SHIFT, msg, 4);
}

void processPU(const uint32_t &can_id, const uint8_t *data, const uint8_t &idx)
{
    auto *pu = &_pu[idx];

    switch (can_id) {
    case PU_PACK1: {
        pu->msg1.vbat = (float) unpackInt16(data, 0) / 100.f;
        memcpy(&pu->msg1.ibat, data + 2, 4);
        pu->msg1.imon = (float) unpackInt16(data, 6) / 100.f;
        break;
    }
    case PU_PACK2: {
        pu->msg2.vout = (float) unpackInt16(data, 0) / 100.f;
        pu->msg2.tbat = (float) unpackInt16(data, 2) / 100.f;
        pu->msg2.pbat = (float) unpackInt16(data, 4);
        pu->msg2.status = data[6];
        break;
    }
    case PU_PACK3: {
        memcpy(&pu->msg3.cbat, data, 8);
        break;
    }
    case PU_PACK4: {
        memcpy(&pu->msg4.res_bar, data, 8);
        break;
    }
    case PU_PACK5: {
        memcpy(&pu->msg5.ibat_flt, data, 8);
        break;
    }
    case PU_PACK6: {
        memcpy(&pu->msg6.cbat_res, data, 8);
        break;
    }
    case PU_PACK7: {
        pu->msg7.h_pwr = unpackInt16(data, 6) / 100.f;
        break;
    }
    }
}

void processMC(const uint32_t &can_id, const uint8_t *data, const uint8_t &idx)
{
    auto *mc = &_mc[idx];

    switch (can_id) {
    case MC_PACK1: {
        mc->vbat = (float) unpackInt16(data, 0) / 100.f;
        mc->tbat = (float) unpackInt16(data, 2) / 100.f;
        mc->tpcb = (float) unpackInt16(data, 4) / 100.f;
        mc->status = data[6];
        break;
    }
    case MC_PACK2: {
        memcpy(mc->cell, data, 8);
        break;
    }
    case MC_PACK3: {
        memcpy(mc->cell + 4, data, 8);
        break;
    }
    }
}

void processSP(const uint32_t &can_id, const uint8_t *data, const uint8_t &idx)
{
    auto *sp = &_sp[idx];

    switch (can_id) {
    case SP_PACK1: {
        sp->msg1.vin = (float) unpackInt16(data, 0) / 100.f;
        sp->msg1.vout = (float) unpackInt16(data, 2) / 100.f;
        sp->msg1.temp = (float) unpackInt16(data, 4) / 100.f;
        sp->msg1.status = data[6];
        break;
    }
    case SP_PACK2: {
        memcpy(&(sp->msg2.cin), data, 8);
        break;
    }
    }
}

EXPORT void canAuxHandler(const uint8_t *data, size_t size)
{
    if (size > PACK_CAN_SIZE) {
        return;
    }

    uint32_t can_id = (uint32_t) (data[0] + (data[1] << 8) + (data[2] << 16) + (data[3] << 24));
    can_id &= 0x7FFFFFFF; // 32nd bit is ext/std flag

    uint8_t can_data[8] = {};
    const uint8_t cnt = (uint8_t) (size - 4);
    for (uint8_t i = 0; i < cnt; i++) {
        can_data[i] = data[4 + i]; // 4 is data position
    }

    //printf("can_id:%x", can_id);

    //CAN ID 0XFFFF
    switch (can_id & 0xFFFF) {
    case PU_PACK1:
    case PU_PACK2:
    case PU_PACK3:
    case PU_PACK4:
    case PU_PACK5:
    case PU_PACK6:
    case PU_PACK7: {
        processPU(can_id, can_data, 0);
        break;
    }
    case PU_PACK1 + 1 * PU_SHIFT:
    case PU_PACK2 + 1 * PU_SHIFT:
    case PU_PACK3 + 1 * PU_SHIFT:
    case PU_PACK4 + 1 * PU_SHIFT:
    case PU_PACK5 + 1 * PU_SHIFT:
    case PU_PACK6 + 1 * PU_SHIFT:
    case PU_PACK7 + 1 * PU_SHIFT: {
        can_id -= 1 * PU_SHIFT;
        processPU(can_id, can_data, 1);
        break;
    }
    case MC_PACK1:
    case MC_PACK2:
    case MC_PACK3: {
        processMC(can_id, can_data, 0);
        break;
    }
    case MC_PACK1 + 1 * MC_SHIFT:
    case MC_PACK2 + 1 * MC_SHIFT:
    case MC_PACK3 + 1 * MC_SHIFT: {
        can_id -= 1 * MC_SHIFT;
        processMC(can_id, can_data, 1);
        break;
    }
    case SP_PACK1:
    case SP_PACK2: {
        processSP(can_id, can_data, 0);
        break;
    }
    case SP_PACK1 + 1 * SP_SHIFT:
    case SP_PACK2 + 1 * SP_SHIFT: {
        can_id -= 1 * SP_SHIFT;
        processSP(can_id, can_data, 1);
        break;
    }
    case SP_POWER_ON:
    case SP_POWER_ON + 1 * SP_SHIFT: {
        printf("SP pwr:%d\n", can_data[0]);
        break;
    }
    case SP_SAVE_K:
    case SP_SAVE_K + 1 * SP_SHIFT: {
        float cmd{};
        memcpy(&cmd, can_data, sizeof(float));
        printf("SP-k:%.2f\n", cmd);
    }
    }
}

EXPORT void gcuHandler(const uint8_t *data, size_t size)
{
    (void) data;
    (void) size;
}

EXPORT void wingHandler(const uint8_t *data, size_t size)
{
    if (size != PACK_WING_SIZE && size_error_wing++ < 3) {
        printf("IFC-%s, wrong telemetry wing packet...\n", txt_dev);
        return;
    }

    //TODO CRC
    if (data[0] == (MSG7_ID | ((NODE_WING_L_ID << 4) & 0xF0))) {
        memcpy(&_wing_l, &data[1], sizeof(WING_DATA));
    }

    if (data[0] == (MSG7_ID | ((NODE_WING_R_ID << 4) & 0xF0))) {
        memcpy(&_wing_r, &data[1], sizeof(WING_DATA));
    }
}

EXPORT void on_main()
{
    bool pwr_ign = (bool) m_sp_pwr_ign::value();
    if (sp_state != pwr_ign) {
        sp_cmd_power_on(pwr_ign);
        sp_state = pwr_ign;
    }
}

EXPORT void on_save_mandala()
{
    //temp
    float raw_adc = m_adc_pt1000_raw::value() * 1000.f;
    float val = getTemperature(raw_adc, TSType::PT1000, VREF, RPUP_PT1000);
    m_pt1000::publish(val);

    //save data
    saveDataToMandala();
}

EXPORT void on_cmd_hpwr()
{
    constexpr const uint8_t PU_MIN_HEATER{0};
    constexpr const uint8_t PU_MAX_HEATER{30};
    uint8_t cmd = limit((uint8_t) m_pu_cmd_hpwr::value(), PU_MIN_HEATER, PU_MAX_HEATER);
    printf("cmd_hpwr:%u", cmd);
    pu_cmd_heater(1, cmd);
}

EXPORT void on_cmd_k()
{
    constexpr const float PU_MIN_CMD_K{0.7f};
    constexpr const float PU_MAX_CMD_K{0.95f};

    float cmd = limit((float) m_sp_cmd_k::value(), PU_MIN_CMD_K, PU_MAX_CMD_K);
    printf("cmd_k:%.2f", cmd);
    sp_cmd_k(cmd);
}

void print_mc(const uint8_t &idx)
{
    printf("v_bat: %.2f", _mc[idx].vbat);
    printf("t_bat: %.2f", _mc[idx].tbat);
    printf("t_pcb: %.2f", _mc[idx].tpcb);
    printf("state: %u", _mc[idx].status);
    printf("C[1]: %.2f", _mc[idx].cell_volt(0));
    printf("C[2]: %.2f", _mc[idx].cell_volt(1));
    printf("C[3]: %.2f", _mc[idx].cell_volt(2));
    printf("C[4]: %.2f", _mc[idx].cell_volt(3));
    printf("C[5]: %.2f", _mc[idx].cell_volt(4));
    printf("C[6]: %.2f", _mc[idx].cell_volt(5));
}

EXPORT void mc_1()
{
    printf("IFC-%s, mc_1...\n", txt_dev);
    print_mc(0);
}

EXPORT void mc_2()
{
    printf("IFC-%s, mc_2...\n", txt_dev);
    print_mc(1);
}

EXPORT void mc_3()
{
    printf("IFC-%s, mc_3...\n", txt_dev);
    print_mc(0);
}

EXPORT void mc_4()
{
    printf("IFC-%s, mc_4...\n", txt_dev);
    print_mc(1);
}

void print_pu(const uint8_t &idx)
{
    printf("vbat: %.2f", _pu[idx].msg1.vbat);
    printf("ibat: %.2f", _pu[idx].msg1.ibat);
    printf("imon: %.2f", _pu[idx].msg1.imon);
    printf("vout: %.2f", _pu[idx].msg2.vout);
    printf("tbat: %.2f", _pu[idx].msg2.tbat);
    printf("pbat: %.2f", _pu[idx].msg2.pbat);
    printf("status: %u", _pu[idx].msg2.status);
    //printf("cbat: %.1f", _pu[idx].msg3.cbat);
    //printf("ebat: %.1f", _pu[idx].msg3.ebat);
    printf("ibat_filt: %.2f", _pu[idx].msg5.ibat_flt);
    printf("vbat_filt: %.2f", _pu[idx].msg5.vbat_flt);
    printf("h_pwr: %.1f", _pu[idx].msg7.h_pwr);
}

EXPORT void pu_1()
{
    printf("IFC-%s, pu_1...\n", txt_dev);
    print_pu(0);
}

EXPORT void pu_2()
{
    printf("IFC-%s, pu_2...\n", txt_dev);
    print_pu(1);
}

EXPORT void pu_3()
{
    printf("IFC-%s, pu_3...\n", txt_dev);
    print_pu(0);
}

EXPORT void pu_4()
{
    printf("IFC-%s, pu_4...\n", txt_dev);
    print_pu(1);
}

void print_sp(const uint8_t &idx)
{
    printf("vin: %.2f", _sp[idx].msg1.vin);
    printf("vout: %.2f", _sp[idx].msg1.vout);
    printf("temp: %.1f", _sp[idx].msg1.temp);
    printf("status: %u", _sp[idx].msg1.status);
    printf("cin: %.2f", _sp[idx].msg2.cin);
    printf("cout: %.2f", _sp[idx].msg2.cout);
}

EXPORT void sp_1()
{
    printf("IFC-%s, sp_1...\n", txt_dev);
    print_sp(0);
}

EXPORT void sp_2()
{
    printf("IFC-%s, sp_2...\n", txt_dev);
    print_sp(1);
}

EXPORT void sp_3()
{
    printf("IFC-%s, sp_3...\n", txt_dev);
    print_sp(0);
}

EXPORT void sp_4()
{
    printf("IFC-%s, sp_4...\n", txt_dev);
    print_sp(1);
}

EXPORT void wing_l()
{
    printf("IFC-%s, wing...\n", txt_dev);
    printf("volt1: %.2f", _wing_l.voltage[0] / 100.f);
    printf("volt2: %.2f", _wing_l.voltage[1] / 100.f);
    printf("temp1: %d", _wing_l.volz_temp[0]);
    printf("temp2: %d", _wing_l.volz_temp[1]);
    printf("pos1: %d", _wing_l.volz_pos[0]);
    printf("pos2: %d", _wing_l.volz_pos[1]);
    printf("temp: %d", _wing_l.gyro_temp);
}

EXPORT void wing_r()
{
    printf("IFC-%s, wing...\n", txt_dev);
    printf("volt1: %.2f", _wing_r.voltage[0] / 100.f);
    printf("volt2: %.2f", _wing_r.voltage[1] / 100.f);
    printf("temp1: %d", _wing_r.volz_temp[0]);
    printf("temp2: %d", _wing_r.volz_temp[1]);
    printf("pos1: %d", _wing_r.volz_pos[0]);
    printf("pos2: %d", _wing_r.volz_pos[1]);
    printf("temp: %d", _wing_r.gyro_temp);
}

EXPORT void pu_on()
{
    printf("IFC-%s, pu_on...\n", txt_dev);
    pu_cmd_power_on(1u);
}

EXPORT void pu_off()
{
    printf("IFC-%s, pu_off...\n", txt_dev);
    pu_cmd_power_on(0u);
}

EXPORT void sp_on()
{
    printf("IFC-%s, sp_on...\n", txt_dev);
    sp_cmd_power_on(1u);
}

EXPORT void sp_off()
{
    printf("IFC-%s, sp_off...\n", txt_dev);
    sp_cmd_power_on(0u);
}
