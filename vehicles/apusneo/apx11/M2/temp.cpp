#include <apx.h>

enum class TSType {
    PT1000,
    NTC3988,
    NTC3570,
};

//----------------------------------------------------------------------------------------------------------------------
constexpr uint8_t SIZE_PT1000{23};
constexpr float RES_PT1000[SIZE_PT1000] = {723.3453f,  763.2784f,  803.0628f,  842.7065f,  882.2166f,  921.5990f,
                                           960.8588f,  1000.f,     1039.0252f, 1077.9350f, 1116.7292f, 1155.4080f,
                                           1193.9713f, 1232.4190f, 1270.7513f, 1308.9680f, 1347.0693f, 1385.0550f,
                                           1422.9253f, 1460.6800f, 1498.3193f, 1535.8430f, 1573.2513f};

constexpr float TEMP_PT1000[SIZE_PT1000] = {-70.f, -60.f, -50.f, -40.f, -30.f, -20.f, -10.f, 0.f,
                                            10.f,  20.f,  30.f,  40.f,  50.f,  60.f,  70.f,  80.f,
                                            90.f,  100.f, 110.f, 120.f, 130.f, 140.f, 150.f};
//----------------------------------------------------------------------------------------------------------------------
constexpr uint8_t SIZE_NTC3988{21};
constexpr float RES_NTC3988[SIZE_NTC3988] = {670100.f, 336500.f, 177000.f, 97070.f, 55330.f, 32650.f, 19900.f,
                                             12490.f,  8057.f,   5327.f,   3603.f,  2488.f,  1752.f,  1258.f,
                                             918.f,    680.f,    511.f,    389.f,   301.f,   235.f,   185.f};

constexpr float TEMP_NTC3988[SIZE_NTC3988] = {-50.f, -40.f, -30.f, -20.f, -10.f, 0.f,   10.f,  20.f,  30.f,  40.f, 50.f,
                                              60.f,  70.f,  80.f,  90.f,  100.f, 110.f, 120.f, 130.f, 140.f, 150.f};
//----------------------------------------------------------------------------------------------------------------------
constexpr uint8_t SIZE_NTC3570{21};
constexpr float RES_NTC3570[SIZE_NTC3570] = {232600.f, 130800.f, 76400.f, 46200.f, 28800.f, 18500.f, 12200.f,
                                             8200.f,   5700.f,   4000.f,  2900.f,  2100.f,  1600.f,  1200.f,
                                             900.f,    700.f,    500.f,   400.f,   300.f,   300.f,   300.f};

constexpr float TEMP_NTC3570[SIZE_NTC3570] = {-40.f, -30.f, -20.f, -10.f, 0.f,   10.f,  20.f,  30.f,  40.f,  50.f, 60.f,
                                              70.f,  80.f,  90.f,  100.f, 110.f, 120.f, 130.f, 140.f, 145.f, 150.f};
//----------------------------------------------------------------------------------------------------------------------

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
        temp = interpolateNXY(res, RES_NTC3988, TEMP_NTC3988);
        break;
    }
    case TSType::NTC3570: {
        temp = interpolateNXY(res, RES_NTC3570, TEMP_NTC3570);
        break;
    }
    }

    return temp;
}

const float VREF{3.f};

int main()
{
    //for (int i = 0; i < 30; i++) {
    //    float temp = getTemperature((float)i / 10.f, TSType::PT1000, VREF, 3160.f);
    //    printf("v:%.2f", (float)i / 10.f);
    //    printf("temp = %.2f", temp);
    //}

    float temp = getTemperature(1.3f, TSType::PT1000, VREF, 2000.f);
    printf("temp = %.2f", temp);

    return 0;
}
