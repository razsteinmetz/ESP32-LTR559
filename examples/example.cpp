#include "Wire.h"
#include "ltr559.h"
#include <Arduino.h>
LTR559 ltrSensor(4, 5);
void setup()
{
    Serial.begin(115200);
    log_e("Starting.....");
    delay(500);
    Wire.begin(4, 5);
    delay(500);
    ltrSensor.reset();
    uint8_t part_id = ltrSensor.part_id();
    uint8_t revision_id = ltrSensor.revision_id();
    uint8_t manufacturer_id = ltrSensor.manufacturer_id();
    log_v("Part ID=%X Revision ID=%X   Manufacturer ID=%X", part_id, revision_id, manufacturer_id);
}
// create the class with SDA & SCL at pins 4 & 5 (esp32-c3)


void loop()
{
    log_v("Reading sensor data:");
    bool res = ltrSensor.get_reading();
    // log_v("Get reading returned %d",res);
    log_v("data:  Light0 %X Light1 %X Integ. time=%d gain=%d ratio=%f lux=%f", ltrSensor.data.als0, ltrSensor.data.als1, ltrSensor.data.integration_time, ltrSensor.data.gain, ltrSensor.data.ratio, ltrSensor.data.lux);
    delay(3000);
}
