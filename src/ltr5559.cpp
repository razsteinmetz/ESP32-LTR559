// various I2C of Sensor (see documentation)
#include "ltr559.h"
#include <Arduino.h>
#include <Wire.h>
#include "i2c_raz.h"
/*

LTR-559 Library - taken from the original code + some updates
please refer to https://optoelectronics.liteon.com/upload/download/ds86-2013-0003/ltr-559als-01_ds_v1.pdf for the sensor documentation

*/

// parameters to various functions (as real values.  the position in the array is the actual register value)
static constexpr uint16_t lookup_light_gain[8] = { 1, 2, 4, 8, 0, 0, 48, 96 };
static constexpr uint16_t lookup_led_current[5] = { 5, 10, 20, 50, 100 };
static constexpr uint16_t lookup_led_duty_cycle[4] = { 25, 50, 75, 100 };
static constexpr uint16_t lookup_led_pulse_freq[8] = { 30, 40, 50, 60, 70, 80, 90, 100 };
static constexpr uint16_t lookup_proximity_meas_rate[8] = { 10, 50, 70, 100, 200, 500, 1000, 2000 };
static constexpr uint16_t lookup_light_integration_time[8] = { 100, 50, 200, 400, 150, 250, 300, 350 };
static constexpr uint16_t lookup_light_repeat_rate[6] = { 50, 100, 200, 500, 1000, 2000 };
// various constants

#define LTR559_ALS_CONTROL 0x80
#define LTR559_ALS_CONTROL_GAIN_MASK 0b111
#define LTR559_ALS_CONTROL_GAIN_SHIFT 2
#define LTR559_ALS_CONTROL_SW_RESET_BIT 1
#define LTR559_ALS_CONTROL_MODE_BIT 0

#define LTR559_PS_CONTROL 0x81
#define LTR559_PS_CONTROL_SATURATION_INDICATOR_ENABLE_BIT 5
#define LTR559_PS_CONTROL_ACTIVE_MASK 0b11

#define LTR559_PS_LED 0x82
#define LTR559_PS_LED_PULSE_FREQ_MASK 0b111
#define LTR559_PS_LED_PULSE_FREQ_SHIFT 5
#define LTR559_PS_LED_DUTY_CYCLE_MASK 0b11
#define LTR559_PS_LED_DUTY_CYCLE_SHIFT 3
#define LTR559_PS_LED_CURRENT_MASK 0b111

#define LTR559_PS_N_PULSES 0x83
#define LTR559_PS_N_PULSES_MASK 0b1111

#define LTR559_PS_MEAS_RATE 0x84
#define LTR559_PS_MEAS_RATE_RATE_MS_MASK 0b1111

#define LTR559_ALS_MEAS_RATE 0x85
#define LTR559_ALS_MEAS_RATE_INTEGRATION_TIME_MASK 0b111
#define LTR559_ALS_MEAS_RATE_INTEGRATION_TIME_SHIFT 3
#define LTR559_ALS_MEAS_RATE_REPEAT_RATE_MASK 0b111

#define LTR559_PART_ID 0x86
#define LTR559_PART_ID_PART_NUMBER_MASK 0b1111
#define LTR559_PART_ID_PART_NUMBER_SHIFT 4
#define LTR559_PART_ID_REVISION_MASK 0b1111

#define LTR559_MANUFACTURER_ID 0x87

#define LTR559_ALS_DATA 0x88
#define LTR559_ALS_DATA_CH1 0x88
#define LTR559_ALS_DATA_CH0 0x8a

#define LTR559_ALS_PS_STATUS 0x8c
#define LTR559_ALS_PS_STATUS_INTERRUPT_MASK 0b00001010
#define LTR559_ALS_PS_STATUS_ALS_DATA_VALID_BIT 7
#define LTR559_ALS_PS_STATUS_ALS_GAIN_MASK 0b111
#define LTR559_ALS_PS_STATUS_ALS_GAIN_SHIFT 4
#define LTR559_ALS_PS_STATUS_ALS_INTERRUPT_BIT 3
#define LTR559_ALS_PS_STATUS_ALS_DATA_BIT 2
#define LTR559_ALS_PS_STATUS_PS_INTERRUPT_BIT 1
#define LTR559_ALS_PS_STATUS_PS_DATA_BIT 0

#define LTR559_PS_DATA 0x8d
#define LTR559_PS_DATA_MASK 0x07FF

#define LTR559_PS_DATA_SATURATION 0x8e
#define LTR559_PS_DATA_SATURATION_SHIFT 4

#define LTR559_INTERRUPT 0x8f
#define LTR559_INTERRUPT_POLARITY_BIT 2
#define LTR559_INTERRUPT_ALS_PS_MASK 0b11
#define LTR559_INTERRUPT_PS_BIT 0
#define LTR559_INTERRUPT_ALS_BIT 1

#define LTR559_PS_THRESHOLD_UPPER 0x90
#define LTR559_PS_THRESHOLD_LOWER 0x92

#define LTR559_PS_OFFSET 0x94
#define LTR559_PS_OFFSET_MASK 0x03FF

#define LTR559_ALS_THRESHOLD_UPPER 0x97
#define LTR559_ALS_THRESHOLD_LOWER 0x99

#define LTR559_INTERRUPT_PERSIST 0x9e
#define LTR559_INTERRUPT_PERSIST_PS_MASK 0b1111
#define LTR559_INTERRUPT_PERSIST_PS_SHIFT 4
#define LTR559_INTERRUPT_PERSIST_ALS_MASK 0b1111

#define LTR559_VALID_PART_ID 0x09
#define LTR559_VALID_REVISION_ID 0x02

LTR559::LTR559()
{
}

LTR559::LTR559(int sda, int scl)
{
    Wire.setPins(sda, scl);
    Wire.begin();
}

void LTR559::reset()
{
    data.integration_time = 100u;
    log_d("LTR559 - Reset Start!");
    set_bits(address, LTR559_ALS_CONTROL, LTR559_ALS_CONTROL_SW_RESET_BIT);
    while (
        get_bits(address, LTR559_ALS_CONTROL, LTR559_ALS_CONTROL_SW_RESET_BIT)) {
        delay(100);
    }
    // show_registers();

    // 50mA, 100% duty cycle, 30Hz, 1 pulse
    // proximity_led(50, 100, 30, 1);

    // enabled, gain 4x
    light_control(true, 4);

    // enabled, saturation indicator enabled
    proximity_control(false, false);

    // 100ms measurement rate
    proximity_measurement_rate(100);

    // 50ms integration time and repeat rate
    light_measurement_rate(50, 50);

    // light_threshold(0xFFFF, 0x0000);
    // proximity_threshold(0x7FFF, 0x7FFF);
    proximity_offset(0);
    uint8_t reg80 = reg_read_uint8(address, LTR559_ALS_CONTROL);
    // log_d("LTR559 - Reset Done! register 80x is %X", reg80);
    // show_registers();
}
/**
 * returns the sensor part id (default 0x9).
 *
 * @return the part id
 */
uint8_t LTR559::part_id()
{
    uint8_t part_id;
    read_bytes(address, LTR559_PART_ID, &part_id, 1);
    return (part_id >> LTR559_PART_ID_PART_NUMBER_SHIFT) & LTR559_PART_ID_PART_NUMBER_MASK;
}
/**
 * returns the sensor revision id (default 0x2).
 *
 * @return the revision id
 */
uint8_t LTR559::revision_id()
{
    uint8_t revision_id;
    read_bytes(address, LTR559_PART_ID, &revision_id, 1);
    return revision_id & LTR559_PART_ID_REVISION_MASK;
}

/**
 * returns the sensor manufacturer id (default 0x5).
 *
 * @return the manufacturer id
 */
uint8_t LTR559::manufacturer_id()
{
    uint8_t manufacturer;
    read_bytes(address, LTR559_MANUFACTURER_ID, &manufacturer, 1);
    return manufacturer;
}
/**
 * Read sensor data and fill the data variable with any new values
 *
 * @return true if there is new data.
 */
bool LTR559::get_reading()
{
    uint8_t reg80 = reg_read_uint8(address, LTR559_ALS_CONTROL);
    // log_d("LTR559 -READING STARTING! register 80x is %X", reg80);
    bool has_updated = false;
    uint8_t status;
    read_bytes(address, LTR559_ALS_PS_STATUS, &status, 1);
    // log_d("Read status returned %X", status);
    bool als_int = (status >> LTR559_ALS_PS_STATUS_ALS_INTERRUPT_BIT) & 0b1;
    bool ps_int = (status >> LTR559_ALS_PS_STATUS_PS_INTERRUPT_BIT) & 0b1;
    bool als_data = (status >> LTR559_ALS_PS_STATUS_ALS_DATA_BIT) & 0b1;
    bool ps_data = (status >> LTR559_ALS_PS_STATUS_PS_DATA_BIT) & 0b1;
    // log_d("alis_int %d ps_int %d als_data %d ps_data %d", als_int, ps_int, als_data, ps_data);
    if (ps_int || ps_data) {
        has_updated = true;
        uint16_t ps0;
        read_bytes(address, LTR559_PS_DATA, (uint8_t*)&ps0, 2);
        ps0 &= LTR559_PS_DATA_MASK;
        data.proximity = ps0;
    }
    if (als_int || als_data) {
        has_updated = true;
        uint16_t als[2];
        read_bytes(address, LTR559_ALS_DATA_CH1, (uint8_t*)&als, 4);
        data.als0 = als[1];
        data.als1 = als[0];
        data.gain = lookup_light_gain[(status >> LTR559_ALS_PS_STATUS_ALS_GAIN_SHIFT) & LTR559_ALS_PS_STATUS_ALS_GAIN_MASK];
        data.ratio = 101.0f;
        if ((uint32_t)data.als0 + data.als1 > 0) {
            data.ratio = (float)data.als1 * 100.0f / ((float)data.als1 + data.als0);
        }
        uint8_t ch_idx = 3;
        if (this->data.ratio < 45)
            ch_idx = 0;
        else if (data.ratio < 64)
            ch_idx = 1;
        else if (data.ratio < 85)
            ch_idx = 2;
        float lux = ((int32_t)data.als0 * ch0_c[ch_idx]) - ((int32_t)data.als1 * ch1_c[ch_idx]);
        lux /= (float)this->data.integration_time / 100.0f;
        lux /= (float)this->data.gain;
        data.lux = lux / 10000.0f;
    }

    return has_updated;
}
/**
 * Set the proximity led :
 * The PS_LED register controls the LED pulse modulation frequency, LED current duty cycle and LED peak current and number of pulses
 * it affects both registers 0x82 & 0x83 on the sensor
 * @param current - the led current (5,10,20,50 or 100ma)
 * @param duty_cycle - the led duty cycle (25%,50%,75%,100%)
 * @param pulse_freq - the led current (30,40,50,60,70,80,90,100Khz)
 *  @param num_pulses - Led# of pulses 1-15
 * @return the manufacturer id
 */
void LTR559::proximity_led(uint8_t current, uint8_t duty_cycle, uint8_t pulse_freq, uint8_t num_pulses)
{
    current = lookup(lookup_led_current, current);

    duty_cycle = lookup(lookup_led_duty_cycle, duty_cycle);
    duty_cycle <<= LTR559_PS_LED_DUTY_CYCLE_SHIFT;

    pulse_freq = lookup(lookup_led_pulse_freq, pulse_freq);
    pulse_freq <<= LTR559_PS_LED_PULSE_FREQ_SHIFT;

    uint8_t buf = current | duty_cycle | pulse_freq;
    write_bytes(address, LTR559_PS_LED, &buf, 1);

    buf = num_pulses & LTR559_PS_N_PULSES_MASK;
    write_bytes(address, LTR559_PS_N_PULSES, &buf, 1);
}

/**
 * Activate and set the gain for the light sensor (ALS) If not activated it will not read new data.

 * @param active - Set to true to activate the light sensor
 * @param gain - the ALS gain (1x,2x,4x,8x,48x,96x)

 */
void LTR559::light_control(bool active, uint8_t gain)
{
    uint8_t buf = 0;
    gain = lookup(lookup_light_gain, gain);
    buf |= gain << LTR559_ALS_CONTROL_GAIN_SHIFT;

    if (active)
        buf |= (0b1 << LTR559_ALS_CONTROL_MODE_BIT);
    else
        buf &= ~(0b1 << LTR559_ALS_CONTROL_MODE_BIT);

    write_bytes(address, LTR559_ALS_CONTROL, &buf, 1);
}
/**
 * Activate and set the proximity sensor If not activated it will not read new data.

 * @param active - Set to true to activate the proximity sensor
 * @param saturation_indicator - set to true to indicate saturation in the result

 */
void LTR559::proximity_control(bool active, bool saturation_indicator)
{
    uint8_t buf = 0;
    read_bytes(address, LTR559_PS_CONTROL, &buf, 1);
    if (active)
        buf |= LTR559_PS_CONTROL_ACTIVE_MASK;
    else
        buf &= ~LTR559_PS_CONTROL_ACTIVE_MASK;

    if (saturation_indicator)
        buf |= 0b1 << LTR559_PS_CONTROL_SATURATION_INDICATOR_ENABLE_BIT;
    else
        buf &= ~(0b1 << LTR559_PS_CONTROL_SATURATION_INDICATOR_ENABLE_BIT);

    write_bytes(address, LTR559_PS_CONTROL, &buf, 1);
}

#if LTR559_USE_INTERRUPTS == 1
/**
 * Set ALS Light threshold for interrupt situation (Not implemented yet!)

 * @param lower,upper - lower & upper limits on when to indicate
 */
void LTR559::light_threshold(uint16_t lower, uint16_t upper)
{
    lower = __builtin_bswap16(lower);
    upper = __builtin_bswap16(upper);
    write_bytes(address, LTR559_ALS_THRESHOLD_LOWER, (uint8_t*)&lower, 2);
    write_bytes(address, LTR559_ALS_THRESHOLD_UPPER, (uint8_t*)&upper, 2);
}
/**
 * Set Proximity threshold for interrupt situation (Not implemented yet!)

 * @param lower,upper - lower & upper limits on when to indicate
 */
void LTR559::proximity_threshold(uint16_t lower, uint16_t upper)
{
    lower = uint16_to_bit12(lower);
    upper = uint16_to_bit12(upper);
    write_bytes(address, LTR559_PS_THRESHOLD_LOWER, (uint8_t*)&lower, 2);
    write_bytes(address, LTR559_PS_THRESHOLD_UPPER, (uint8_t*)&upper, 2);
}
#endif

/**
 * Set light measurement rate for the ALS sensor (register 0x85)
 * controls the integration time and timing of the periodic measurement of the ALS in active
 *  mode. ALS Measurement Repeat Rate is the interval between ALS_DATA registers update. ALS Integration Time is the
 * measurement time for each ALS cycle.
 * @param integration_time - the integration time (50,100,200,500,1000,2000 ms)
 * @param rate - the integration rate (50,100,150,200,250,300,350,400)
 */
void LTR559::light_measurement_rate(uint16_t integration_time, uint16_t rate)
{
    data.integration_time = integration_time;
    integration_time = lookup(lookup_light_integration_time, integration_time);
    rate = lookup(lookup_light_repeat_rate, rate);
    uint8_t buf = 0;
    buf |= rate;
    buf |= integration_time << LTR559_ALS_MEAS_RATE_INTEGRATION_TIME_SHIFT;
    write_bytes(address, LTR559_ALS_MEAS_RATE, &buf, 1);
}
/**
 * Set proximity measurement rate for the proximity sensor (register 0x84)
 * controls the timing of the periodic measurements of the PS in active mode. PS
 * Measurement Repeat Rate is the interval between PS_DATA registers update
 * @param rate - the integration rate (10,50,70,100,200,500,1000,2000)
 */
void LTR559::proximity_measurement_rate(uint16_t rate)
{
    uint8_t buf = lookup(lookup_proximity_meas_rate, rate);
    write_bytes(address, LTR559_PS_MEAS_RATE, &buf, 1);
}

void LTR559::proximity_offset(uint16_t offset)
{
    offset &= LTR559_PS_OFFSET_MASK;
    write_bytes(address, LTR559_PS_OFFSET, (uint8_t*)&offset, 1);
}

uint16_t LTR559::bit12_to_uint16(uint16_t value)
{
    return ((value & 0xFF00) >> 8) | ((value & 0x000F) << 8);
}

uint16_t LTR559::uint16_to_bit12(uint16_t value)
{
    return ((value & 0xFF) << 8) | ((value & 0xF00) >> 8);
}

void LTR559::show_registers()
{
    uint8_t r80[15];
    log_d("-------------------------REGISTERS----------------------------");
    for (uint8_t i = 0u; i < sizeof(r80); i++) {
        r80[i] = reg_read_uint8(address, 0x80 + i);
        log_d("REG: %X  value:  %X ", 0x80 + i, r80[i]);
    }
    log_d("-----------------------------------------------------------------");
}
