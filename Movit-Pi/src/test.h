#ifndef _TEST_H_
#define _TEST_H_

#include "alarm.h"
#include "forceSensor.h"
#include "forcePlate.h"

bool program_test(Alarm &alarm,  uint16_t* max11611Data, forceSensor &sensorMatrix, forcePlate &globalForcePlate);
void printStuff();

#endif /* _TEST_H_ */