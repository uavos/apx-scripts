#include <apx.h>

constexpr const char *txt_dev = "C";
constexpr const port_id_t PORT_ID_CAN{31};
constexpr const port_id_t PORT_ID_AGL{33};
constexpr const port_id_t PORT_ID_DCELL{34};

constexpr const uint8_t TASK_MAIN_MS{50}; //20Hz

//--------------------------pu-----------------------------
constexpr const uint16_t PU_ID{128};
constexpr const uint16_t PU_SHIFT{20};
constexpr const uint16_t PU_CMD_ON{PU_ID + 12};
constexpr const uint16_t PU_CMD_RB{PU_ID + 13};

//--------------------------dcell-----------------------------
constexpr const port_id_t DEVICE_ID_DCELL{1};
int g_responseType{-1}; // 0 - read, 1 - write, 2 - exec

enum DCELL {
    SYS,
    TMP,
    ELEC,
    SRAW,
    MVV,
};

uint8_t cmd_dcell{0};
uint8_t REG{1};
uint8_t DCELL_DATA[16] = {};

float dcell_sys{0.f};
float dcell_sys_cor{0.f};

constexpr const float G_NORM{9.80665f};
constexpr const float M_MOT{488.f}; ///Добавить лопасти (73 гр) и кок (? гр) позже
constexpr const float DCELL_BIAS{488.f};

//calc eff
constexpr const float Kt{0.0749f};
constexpr const float FreeSPN_RPM{2300.f};
constexpr const float FreeSPN_PWR{19.f};
constexpr const float FreeSPN_EXP{1.15f};
constexpr const float Rph_esc_wires{27.5f};
constexpr const float Rph_motor{101.4f};

using m_agl = Mandala<mandala::sns::nav::agl::laser>;
using m_ax = Mandala<mandala::est::nav::acc::x>;
using m_rpm = Mandala<mandala::est::nav::eng::rpm>;

//ESC
using m_esc_rpm = Mandala<mandala::est::env::usrw::w1>;
using m_esc_crt = Mandala<mandala::est::env::usrf::f3>;
using m_esc_mt = Mandala<mandala::est::env::usr::u1>;
using m_esc_crtin = Mandala<mandala::est::env::usrf::f1>;
using m_esc_vin = Mandala<mandala::sns::env::eng::voltage>;

//TAS
using m_air = Mandala<mandala::est::nav::air::airspeed>;
using m_ktas = Mandala<mandala::est::nav::air::ktas>;

//DCELL
using m_dcell_sys = Mandala<mandala::est::env::usr::u4>;
using m_dcell_sys_cor = Mandala<mandala::est::env::usr::u5>;
using m_dcell_temp = Mandala<mandala::est::env::usr::u6>;

//EFF
using m_p_mech = Mandala<mandala::est::env::usr::u7>;
using m_p_esc_calc = Mandala<mandala::est::env::usr::u8>;
using m_eff_bldc = Mandala<mandala::est::env::usr::u9>;
using m_eff_prop = Mandala<mandala::est::env::usr::u10>;
using m_eff_overal = Mandala<mandala::est::env::usr::u11>;
using m_kalman_esc_crt = Mandala<mandala::est::env::usrf::f2>;

struct KalmanFilter
{
    KalmanFilter(float q = 0.5f, float r = 10.f, float f = 1.f, float h = 1.f)
    {
        Q = q;
        R = r;
        F = f;
        H = h;
    }

    float KalmanCorrect(float signal)
    {
        X0 = F * State;
        P0 = F * Covariance * F + Q;

        K = H * P0 / (H * P0 * H + R);
        State = X0 + K * (signal - H * X0);
        Covariance = (1.f - K * H) * P0;

        return State;
    }

    float Q = 0.f;
    float R = 0.f;
    float F = 0.f;
    float H = 0.f;

    float X0 = 0.f;
    float P0 = 0.f;

    float State = 0.f;
    float Covariance = 1.f;
    float K = 0.f;
};

KalmanFilter _kalman_esc_crt{};

int main()
{
    m_esc_rpm();
    m_esc_crt();
    m_esc_mt();
    m_esc_crtin();
    m_esc_vin();

    m_air();
    m_ktas();

    m_ax();

    task("pu_on");  //GCS with terminal command `vmexec("pu_on")`
    task("pu_off"); //GCS with terminal command `vmexec("pu_off")`
    task("pu_rb");

    schedule_periodic(task("on_main"), TASK_MAIN_MS);

    receive(PORT_ID_AGL, "aglHandler");
    receive(PORT_ID_DCELL, "dcellHandler");

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

EXPORT void sendCmdToCan(const uint32_t &can_id, const uint8_t *data, const uint8_t &size)
{
    uint8_t msg[12] = {};

    msg[0] = (uint8_t) can_id;         //ID_0_7
    msg[1] = (uint8_t) (can_id >> 8);  //ID_8_15
    msg[2] = (uint8_t) (can_id >> 16); //ID_16_23
    msg[3] = (uint8_t) (can_id >> 24); //ID_24_31

    if (can_id > 0x7FF) {
        msg[3] = msg[3] | 0x80;
    }

    for (uint8_t i = 0; i < size; i++) {
        msg[4 + i] = data[i];
    }

    send(PORT_ID_CAN, msg, 4 + size, false);
    send(PORT_ID_CAN + 1, msg, 4 + size, false);
}

void pu_cmd_power_on(uint8_t val)
{
    uint8_t msg[1] = {val};

    sendCmdToCan(PU_CMD_ON, msg, 1);
    sendCmdToCan(PU_CMD_ON + PU_SHIFT, msg, 1);
}

void pu_cmd_rb(uint8_t val)
{
    uint8_t msg[1] = {val};

    sendCmdToCan(PU_CMD_RB, msg, 1);
    sendCmdToCan(PU_CMD_RB + PU_SHIFT, msg, 1);
}

uint16_t calcCrc(const uint8_t *buf, size_t len)
{
    uint16_t crc = 0xFFFF;

    for (uint8_t pos = 0; pos < len; pos++) {
        crc ^= buf[pos];
        for (uint8_t i = 8; i != 0; i--) {
            if ((crc & 0x0001) != 0) {
                crc >>= 1;
                crc ^= 0xA001;
            } else
                crc >>= 1;
        }
    }
    return crc;
}

void Req(uint8_t addr)
{
    uint8_t cmd[8];

    cmd[0] = DEVICE_ID_DCELL;
    cmd[1] = 3;
    cmd[2] = 0;
    cmd[3] = addr - 1;
    cmd[4] = 0;
    cmd[5] = 2;
    uint16_t crc = calcCrc(cmd, 6);
    cmd[6] = (crc >> (8 * 0)) & 0xFF;
    cmd[7] = (crc >> (8 * 1)) & 0xFF;

    send(PORT_ID_DCELL, cmd, 8, false);
}

bool checkCrc(const uint8_t *buf, size_t len)
{
    uint16_t crc = buf[len - 2] + buf[len - 1] * 256;
    uint16_t crc2 = calcCrc(buf, len - 2);
    return crc == crc2;
}

void ReqestFloat(uint8_t addr)
{
    Req(addr);
    g_responseType = 0;
}

EXPORT void on_main()
{
    m_rpm::publish(m_esc_rpm::value());

    //agl
    send(PORT_ID_AGL, (const uint8_t *) "?LD\r\n", 5, false);

    //dcell
    switch (REG) {
    case 1: {
        cmd_dcell = DCELL::SYS;
        ReqestFloat(21);
        REG = 2;
        break;
    }
    case 2: {
        cmd_dcell = DCELL::TMP;
        ReqestFloat(23);
        REG = 1;
        break;
    }
    }
}

float toFloat(const uint8_t *data, uint8_t index)
{
    float val = {0.f};

    uint8_t temp[4] = {};
    temp[0] = data[index + 1];
    temp[1] = data[index + 0];
    temp[2] = data[index + 3];
    temp[3] = data[index + 2];

    memcpy(&val, temp, 4);

    return val;
}

float correction_ax(float val)
{
    return val + (M_MOT / 1000.f) * m_ax::value();
}

void calc_eff()
{
    const float F = dcell_sys_cor;

    //const float TAS = m_ktas::value() * m_air::value();
    const float TAS = 6.f;

    const float RPM = m_esc_rpm::value();
    const float Iphase = _kalman_esc_crt.KalmanCorrect(m_esc_crt::value());
    const float T_bldc = m_esc_mt::value();

    const float Rph_tot = Rph_motor / 1000.f * (1 + 0.00393f * (T_bldc - 25.f)) + Rph_esc_wires / 1000.f;

    float Pmech = Iphase * Kt * 0.10472f * RPM;
    if (Pmech < 1.f) {
        Pmech = 1.f;
    }

    const float Ploss = 0.75f * Iphase * Iphase * Rph_tot + (pow((RPM / FreeSPN_RPM), FreeSPN_EXP) * FreeSPN_PWR);

    const float P_ESC_CALC = Pmech + Ploss;

    const float BLDC_EFF = limit(Pmech / P_ESC_CALC * 100.f, 0.f, 100.f);
    const float Prop_EFF = limit((F * TAS) / (Pmech) * 100.f, 0.f, 100.f);
    const float Overal_EFF = limit(BLDC_EFF * Prop_EFF / 100.f, 0.f, 100.f);

    //dcell
    m_dcell_sys::publish(dcell_sys);
    m_dcell_sys_cor::publish(dcell_sys_cor);

    //eff
    m_p_mech::publish(Pmech);
    m_p_esc_calc::publish(P_ESC_CALC);
    m_eff_bldc::publish(BLDC_EFF);
    m_eff_prop::publish(Prop_EFF);
    m_eff_overal::publish(Overal_EFF);

    m_kalman_esc_crt::publish(Iphase);
}

void save_data_dcell(uint8_t cmd, float value)
{
    switch (cmd) {
    case DCELL::SYS: {
        value = ((value + DCELL_BIAS) / 1000.f) * G_NORM;

        dcell_sys = value;
        dcell_sys_cor = correction_ax(value);

        calc_eff();
        break;
    }
    case DCELL::TMP: {
        m_dcell_temp::publish(value);
        break;
    }
    }
}

EXPORT void aglHandler(const uint8_t *data, size_t size)
{
    for (uint32_t i = 0; i < size; i++) {
        if (data[i] == ':') {
            int sign = 1;
            float result = 0.0f;
            float divisor = 10.0f;
            bool decimal = false;
            i++;
            if (data[i] == '-') {
                sign = -1;
                i++;
            }
            for (; i < size; i++) {
                if (data[i] >= '0' && data[i] <= '9') {
                    if (decimal) {
                        result += (data[i] - '0') / divisor;
                        divisor *= 10.0f;
                    } else {
                        result = result * 10.0f + (data[i] - '0');
                    }
                } else if (data[i] == '.') {
                    decimal = true;
                } else {
                    break;
                }
            }
            float agl = (float) sign * result;

            //printf("agl:%.2f", agl);
            m_agl::publish(agl);
        }
    }
}

EXPORT void dcellHandler(const uint8_t *data, size_t size)
{
    if (size > 16) {
        return;
    }

    memcpy(DCELL_DATA, data, size);

    if (checkCrc(DCELL_DATA, size)) {
        if (DCELL_DATA[1] == 3) {
            if (g_responseType == 0) {
                float value = toFloat(DCELL_DATA, 3);
                save_data_dcell(cmd_dcell, value);
            } else {
                print("Unknown response type");
            }
        } else {
            printf("Unknown command: %d\n", DCELL_DATA[1]);
        }
        g_responseType = -1;
    } else {
        printf("Invalid crc");
    }
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

EXPORT void pu_rb(int32_t val)
{
    printf("IFC-%s, pu_rb...\n", txt_dev);
    val = limit(val, 0, 100);
    pu_cmd_rb((uint8_t) val);
}
