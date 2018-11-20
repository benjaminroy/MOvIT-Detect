#ifndef FORCE_SENSOR_H
#define FORCE_SENSOR_H

#include "MAX11611.h"
#include "Utils.h"

class ForceSensor
{
  public:
    ForceSensor();
    ~ForceSensor();

    void CalibrateForceSensor(MAX11611 &max11611, uint16_t *max11611Data, uint8_t maxIteration);
    bool IsUserDetected();

    pressure_mat_offset_t GetOffsets();
    void SetOffsets(pressure_mat_offset_t offset);

    uint16_t GetAnalogData(uint8_t index) {return _analogData[index];}
    void SetAnalogData(uint8_t index, uint16_t analogData) {RearrangeADCData(index, analogData);}
    uint16_t RearrangeADCData(uint8_t index, uint16_t analogdata);

  private:
    uint16_t _analogData[PRESSURE_SENSOR_COUNT];
    uint16_t _analogOffset[PRESSURE_SENSOR_COUNT];
    float _totalSensorMean;
    float _detectionThreshold;
};

#endif // FORCE_SENSOR_H
