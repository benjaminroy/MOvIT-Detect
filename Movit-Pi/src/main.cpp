#include "I2Cdev.h"   //I2C librairy
#include "MPU6050.h"  //Implementation of Jeff Rowberg's driver
#include "MAX11611.h" //10-Bit ADC librairy

//Include : Modules
#include "init.h"         //variables and modules initialisation
#include "alarm.h"
#include "accel_module.h" //variables and modules initialisation

#include "forcePlate.h"
#include "forceSensor.h"
#include "program.h"      //variables and modules initialisation
#include "test.h"         //variables and modules initialisation
#include "DateTimeRTC.h"

#include <string>
#include <stdio.h>
#include <unistd.h>
#include "mosquitto_broker/mosquitto_broker.h"

using std::string;

#define SLEEP_TIME 5000000

void sendDataToWebServer(MosquittoBroker *mosquittoBroker)
{
    mosquittoBroker->sendBackRestAngle(10);
    mosquittoBroker->sendDistanceTraveled(1000);
}


MCP79410 mcp79410;       //Initialisation of the MCP79410
MAX11611 max11611;       //Initialisation of the 10-bit ADC

//Movement detection variables
const float isMovingTrigger = 1.05;
float isMovingValue;

//Event variables
bool sleeping = false;
int alertNotification = 0;
bool testSequence = true;

//Debug variable
bool affichageSerial = true;

bool bIsBtnPushed = false;

int main()
{
    I2Cdev::initialize();

    MosquittoBroker *mosquittoBroker = new MosquittoBroker("actionlistener");
    BackSeatAngleTracker imu;

    uint16_t max11611Data[9];
    forceSensor sensorMatrix;
    forcePlate globalForcePlate;

    DateTimeRTC *datetimeRTC = DateTimeRTC::getInstance();
    datetimeRTC->set24HourFormat(true);

    unsigned char currentDateTime[DATE_TIME_SIZE] = {0x50, 0x59, 0x21, 0x00, 0x11, 0x03, 0x18}; //{seconds, minutes, hours, AM = 0 PM = 1, day, date, month, year}
    datetimeRTC->setDateTime(currentDateTime);

    Alarm alarm(700, 0.1);
    init_ADC(sensorMatrix);
    printf("Setup Done\n");

    bool done = false;
    while (!done)
    {
        done = program_loop(alarm, datetimeRTC, imu, max11611Data, sensorMatrix, globalForcePlate);

        // sendDataToWebServer(mosquittoBroker);
        // usleep(SLEEP_TIME);
    }

    delete mosquittoBroker;
    return 0;
}

//---------------------------------------------------------------------------------------
// OPERATION CALLBACK DU TIMER A CHAQUE SECONDE
// on obtient les datas : date and time, capteur angle, capteur force
// on verifie si les led doivent blink, puis on les fait blink pour la duree demandee
// flagrunning : utilite a determiner
//---------------------------------------------------------------------------------------
 //void callback()
 //{
//     getData();     //Lit l'état des capteurs
//     led_control(); //Gère la LED du bouton, Etats: On,Off ou Blink

//     if (flagRunning == 0)
//     {
//         digitalWrite(LED_BUILTIN, HIGH); // turn the LED on (HIGH is the voltage level)
//         flagRunning = 1;
//     }
//     else
//     {
//         digitalWrite(LED_BUILTIN, LOW); // turn the LED off by making the voltage LOW
//         flagRunning = 0;
//     }
// }
