#include <apx.h>

using m_sw9 = Mandala<mandala::ctr::env::sw::sw9>;   // #1 Antena
using m_sw10 = Mandala<mandala::ctr::env::sw::sw10>; // #2 rel1
using m_sw11 = Mandala<mandala::ctr::env::sw::sw11>; // #4 rel2
using m_sw12 = Mandala<mandala::ctr::env::sw::sw12>; // #8 pwr_en (Only Master)
using m_sw13 = Mandala<mandala::ctr::env::sw::sw13>; // #16 rf_blank (Only Master)
using m_sw14 = Mandala<mandala::ctr::env::sw::sw14>; // #32 fan3 alg manual
using m_sw15 = Mandala<mandala::ctr::env::sw::sw15>; // #64 fan3 (Only Master)

using mode = Mandala<mandala::cmd::nav::proc::mode>;
using b9 = Mandala<mandala::est::env::usrb::b9>;

using m_temp_pyld1 = Mandala<mandala::sns::env::scr::s1>;
using m_temp_pyld2 = Mandala<mandala::sns::env::scr::s2>;

using m_aux_rt = Mandala<mandala::sns::env::aux::rt>;
using m_com_temp = Mandala<mandala::sns::env::com::temp>;

using m_status = Mandala<mandala::est::env::usrc::c5>;

using m_mode = Mandala<mandala::cmd::nav::proc::mode>;
using m_stage = Mandala<mandala::cmd::nav::proc::stage>;

//----------------------------------------------------------------------------------------------------------------------
constexpr uint8_t SIZE_NTC3988{21};
constexpr float RES_NTC3988[SIZE_NTC3988] = {670100.f, 336500.f, 177000.f, 97070.f, 55330.f, 32650.f, 19900.f,
                                             12490.f,  8057.f,   5327.f,   3603.f,  2488.f,  1752.f,  1258.f,
                                             918.f,    680.f,    511.f,    389.f,   301.f,   235.f,   185.f};

constexpr float TEMP_NTC3988[SIZE_NTC3988] = {-50.f, -40.f, -30.f, -20.f, -10.f, 0.f,   10.f,  20.f,  30.f,  40.f, 50.f,
                                              60.f,  70.f,  80.f,  90.f,  100.f, 110.f, 120.f, 130.f, 140.f, 150.f};
//----------------------------------------------------------------------------------------------------------------------

constexpr const float VREF{3.f};
constexpr const float RPUP{3000.f};
constexpr const float K_TEMP{3.74f};

float PYLD_TMP[2] = {};

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

float getTemperature(const float &vin, const float &vref, const float &rpup)
{
    if (vin < 0.f || vin > vref) {
        return 0.f;
    }

    float res = (vin * rpup) / (vref - vin);

    return interpolateNXY(res, RES_NTC3988, TEMP_NTC3988);
}

int main()
{
    //subscribe
    m_sw9();
    m_sw10();
    m_sw11();
    m_sw12();
    m_sw13();
    m_sw14();
    m_sw15();

    m_aux_rt();
    m_com_temp();

    m_temp_pyld1();
    m_temp_pyld2();

    m_status();

    m_mode();
    m_stage();

    schedule_delayed(task("on_startup"), 3000);
    schedule_periodic(task("on_main"), 500);
    task("rs_radio");
    mode("on_mode");

    return 0;
}

void antena_control()
{
    if (m_sw9::value() == 0 &&   // Antena off
        m_mode::value() == 6 &&  // TAKEOFF
        m_stage::value() >= 3) { // LIFTOFF
        m_sw9::publish(true);
        printf("IFC.Antene on\n");
    }

    if (m_sw9::value() == 1 &&   // Antena on
        m_mode::value() == 7 &&  // LANDING
        m_stage::value() >= 6) { // RUN
        m_sw9::publish(false);
        printf("IFC.Antene off\n");
    }
}

EXPORT void on_startup()
{
    m_sw10::publish(true);
    sleep(1000);

    m_sw11::publish(true);
    sleep(1000);

    m_sw12::publish(true);
    sleep(1000);

    m_sw13::publish(true);

    printf("IFC.Radio on\n");
}

EXPORT void on_mode()
{
    auto value = mode::value();
    if (value == (uint32_t) mandala::proc_mode_TAXI) {
        b9::publish(false);
    } else {
        b9::publish(true);
    }
}

EXPORT void on_main()
{
    antena_control();

    uint32_t status = ((uint32_t) m_sw9::value() << 0) | ((uint32_t) m_sw10::value() << 1)
                      | ((uint32_t) m_sw11::value() << 2) | ((uint32_t) m_sw12::value() << 3)
                      | ((uint32_t) m_sw13::value() << 4) | ((uint32_t) m_sw14::value() << 5)
                      | ((uint32_t) m_sw15::value() << 6);

    m_status::publish(status);

    PYLD_TMP[0] = getTemperature(m_temp_pyld1::value() / K_TEMP, VREF, RPUP);
    PYLD_TMP[1] = getTemperature(m_temp_pyld2::value() / K_TEMP, VREF, RPUP);

    m_aux_rt::publish(PYLD_TMP[0]);
    m_com_temp::publish(PYLD_TMP[1]);
}

EXPORT void rs_radio()
{
    m_sw10::publish(false);
    m_sw11::publish(false);
    m_sw12::publish(false);
    m_sw13::publish(false);
    sleep(3000);

    m_sw10::publish(true);
    sleep(1000);

    m_sw11::publish(true);
    sleep(1000);

    m_sw12::publish(true);
    sleep(1000);

    m_sw13::publish(true);

    printf("IFC.Resetting Radio\n");
}
