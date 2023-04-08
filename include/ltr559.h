#include <Arduino.h>

#define LTR559_I2C_ADDRESS 0x23

#define LTR559_USE_INTERRUPTS 0 // interrupt routines not implement yet!

/*
 * The data structure holding the sensor information
 */
typedef struct {
    uint16_t proximity;
    uint16_t als0;
    uint16_t als1;
    uint16_t integration_time;
    uint16_t gain;
    float ratio;
    float lux;
} ltr559_reading;

class LTR559 {
    //--------------------------------------------------
    // Constants
    //--------------------------------------------------
public:
    static const uint8_t DEFAULT_I2C_ADDRESS = LTR559_I2C_ADDRESS;

private:
    const int ch0_c[4] = { 17743, 42785, 5926, 0 };
    const int ch1_c[4] = { -11059, 19548, -1185, 0 };

    //--------------------------------------------------
    // Variables
    //--------------------------------------------------
public:
    ltr559_reading data;

private:
    // interface pins with our standard defaults where appropriate
    const uint8_t address = DEFAULT_I2C_ADDRESS;

    //--------------------------------------------------
    // Constructors/Destructor
    //--------------------------------------------------
public:
    LTR559(); // not doing anything on empty constructor.

    LTR559(int sda, int scl); // init the connection (same as init)

    //--------------------------------------------------
    // Methods
    //--------------------------------------------------
public:
    bool init();
    void reset();
    uint8_t part_id();
    uint8_t revision_id();
    uint8_t manufacturer_id();
    bool get_reading();
    void proximity_led(uint8_t current, uint8_t duty_cycle, uint8_t pulse_freq,
        uint8_t num_pulses);
    void light_control(bool active, uint8_t gain);
    void proximity_control(bool active, bool saturation_indicator);
#if LTR559_USE_INTERRUPTS == 1
    void light_threshold(uint16_t lower, uint16_t upper);
    void proximity_threshold(uint16_t lower, uint16_t upper);
#endif
    void light_measurement_rate(uint16_t integration_time, uint16_t rate);
    void proximity_measurement_rate(uint16_t rate);
    void proximity_offset(uint16_t offset);
    void show_registers();


// template of a function to lookup value in an array of known size and return its position or 0 if not found.
// note that is is used for parameters values

template <typename T, size_t N>
const uint16_t lookup(const T (&arr)[N], uint16_t value) {
    for (size_t i = 0; i < N; ++i) {
        // log_d("length is :%d",N);
        if (arr[i]==value) {return (uint16_t)i;}
    }
    log_e("Parameter not found in possible values! check your LTR559 calls!");
    return 0;
}


private:
    uint16_t bit12_to_uint16(uint16_t value);
    uint16_t uint16_to_bit12(uint16_t value);
};
