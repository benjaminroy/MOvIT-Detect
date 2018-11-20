#include "DeviceManager.h"
#include "NetworkManager.h"
#include "FixedImu.h"
#include "MobileImu.h"
#include "I2Cdev.h"
#include "Utils.h"
#include "MAX11611.h"
#include "ForceSensor.h"
#include "SysTime.h"
#include "MAX11611.h"
#include "ForceSensor.h"
#include "SysTime.h"

#include <unistd.h>
#include <thread>

DeviceManager::DeviceManager(FileManager *fileManager) : _fileManager(fileManager),
                                                         _alarm(10)
{
    _motionSensor = MotionSensor::GetInstance();
    _mobileImu = MobileImu::GetInstance();
    _fixedImu = FixedImu::GetInstance();
    _datetimeRTC = DateTimeRTC::GetInstance();
    _pressureMat = PressureMat::GetInstance();
}

void DeviceManager::InitializeDevices()
{
    I2Cdev::Initialize();

    _fileManager->Read();

    _notificationsSettings = _fileManager->GetNotificationsSettings();

    InitializePressureMat();
    InitializeMobileImu();
    InitializeFixedImu();

    _datetimeRTC->SetCurrentDateTimeThread().detach();
    _isAlarmInitialized = _alarm.Initialize();
    _isMotionSensorInitialized = _motionSensor->Initialize();

    _fileManager->Save();

    printf("Setup Done\n");
}

void DeviceManager::InitializePressureMat()
{
    pressure_mat_offset_t pressureMatOffset = _fileManager->GetPressureMatoffset();
    _pressureMat->SetOffsets(pressureMatOffset);
    _pressureMat->Initialize();
}
void DeviceManager::InitializeMobileImu()
{
    _isMobileImuInitialized = _mobileImu->Initialize();
    _isMobileImuCalibrated = false;

    if (!_isMobileImuInitialized)
    {
        return;
    }

    imu_offset_t mobileOffset = _fileManager->GetMobileImuOffsets();

    if (!Imu::IsImuOffsetValid(mobileOffset))
    {
        CalibrateMobileIMU();
    }
    else
    {
        _mobileImu->SetOffset(mobileOffset);
        _isMobileImuCalibrated = true;
    }
}

void DeviceManager::InitializeFixedImu()
{
    _isFixedImuInitialized = _fixedImu->Initialize();
    _isFixedImuCalibrated = false;

    if (!_isFixedImuInitialized)
    {
        return;
    }

    imu_offset_t fixedOffset = _fileManager->GetFixedImuOffsets();

    if (!Imu::IsImuOffsetValid(fixedOffset))
    {
        CalibrateFixedIMU();
    }
    else
    {
        _fixedImu->SetOffset(fixedOffset);
        _isFixedImuCalibrated = true;
    }
}

Sensor *DeviceManager::GetSensor(const int device)
{
    switch (device)
    {
    case alarmSensor:
        return &_alarm;
    case mobileImu:
        return _mobileImu;
    case fixedImu:
        return _fixedImu;
    case motionSensor:
        return _motionSensor;
    default:
        throw "Error: Invalid device";
        break;
    }
}

void DeviceManager::ReconnectSensor(const int device)
{
    Sensor *sensor;
    try
    {
        sensor = GetSensor(device);
    }
    catch (const std::exception &e)
    {
        printf("Error: Invalid device\n");
        return;
    }

    if (!sensor->IsConnected())
    {
        if (typeid(sensor) == typeid(FixedImu))
        {
            InitializeFixedImu();
        }
        else if (typeid(sensor) == typeid(MobileImu))
        {
            InitializeMobileImu();
        }
        else if (typeid(sensor) == typeid(ForcePlate))
        {
            _pressureMat->Initialize();
        }
        else
        {
            sensor->Initialize();
        }
    }
}

bool DeviceManager::IsSensorStateChanged(const int device)
{
    Sensor *sensor;
    try
    {
        sensor = GetSensor(device);
    }
    catch (const std::exception &e)
    {
        printf("Error: Invalid device\n");
        return false;
    }

    if (sensor->IsStateChanged())
    {
        return true;
    }

    return false;
}

void DeviceManager::TurnOff()
{
    if (_isAlarmInitialized)
    {
        GetAlarm()->TurnOffAlarm();
    }
}

void DeviceManager::CalibratePressureMat()
{
    printf("Calibrating pressure mat ... \n");
    _pressureMat->Calibrate();
    _fileManager->SetPressureMatOffsets(_pressureMat->GetOffsets());
    _fileManager->Save();
    printf("DONE\n");
}

void DeviceManager::CalibrateIMU()
{
    CalibrateFixedIMU();
    CalibrateMobileIMU();
}

void DeviceManager::CalibrateFixedIMU()
{
    printf("Calibrating FixedIMU ... \n");
    _fixedImu->CalibrateAndSetOffsets();
    _fileManager->SetFixedImuOffsets(_fixedImu->GetOffset());
    _fileManager->Save();
    _isFixedImuCalibrated = true;
    printf("DONE\n");
}

void DeviceManager::CalibrateMobileIMU()
{
    printf("Calibrating MobileIMU ... \n");
    _mobileImu->CalibrateAndSetOffsets();
    _fileManager->SetMobileImuOffsets(_mobileImu->GetOffset());
    _fileManager->Save();
    _isMobileImuCalibrated = true;
    printf("DONE\n");
}

void DeviceManager::Update()
{
    _timeSinceEpoch = _datetimeRTC->GetTimeSinceEpoch();
    if (_isMotionSensorInitialized)
    {
        _isMoving = _motionSensor->IsMoving();
    }

    if (_isFixedImuInitialized && _isMobileImuInitialized && _isFixedImuCalibrated && _isMobileImuCalibrated)
    {
        // Data: Angle (centrales intertielles mobile/fixe)
        _backSeatAngle = _backSeatAngleTracker.GetBackSeatAngle();
    }
    else
    {
        _backSeatAngle = DEFAULT_BACK_SEAT_ANGLE;
    }

    if (_isFixedImuInitialized && _isFixedImuCalibrated)
    {
        _isChairInclined = _backSeatAngleTracker.IsInclined();
    }

    if (IsPressureMatCalibrated())
    {
        _pressureMat->Update();
    }
}

double DeviceManager::GetXAcceleration()
{
    if (_isFixedImuInitialized)
    {
        return _fixedImu->GetXAcceleration();
    }
    return 0;
}

void DeviceManager::UpdateNotificationsSettings(std::string notificationsSettings)
{
    _fileManager->SetNotificationsSettings(notificationsSettings);
    _fileManager->Save();
}

bool DeviceManager::TestDevices()
{
    printf("\n.-.--..--- TEST MENU .--.---.--.-\n");
    printf("\n\tTestID\tDescription");
    //--------------------------------------------------------------------------
    // MODULE : Notification
    // PCB Validation tests:
    printf("\n\t 0 \t LEDs RG (notif) validation");
    printf("\n\t 1 \t DC motor - buzzer (notif) validation");
    // Functionnal tests:
    printf("\n\t 2 \t Red blink alarm (notif) validation");
    printf("\n\t 3 \t Blink alarm 5 seconds (notif) validation");
    //--------------------------------------------------------------------------
    //--------------------------------------------------------------------------
    // MODULE : Pressure Sensor
    // PCB Validation tests:
    printf("\n\t 5 \t Force sensors (ADC) validation");
    // Functionnal tests:
    printf("\n\t 6 \t Force sensors calibration validation");
    printf("\n\t 7 \t Presence detection validation");
    printf("\n\t 8 \t Force plates validation");
    printf("\n\t 9 \t Centers of pressure validation");
    //--------------------------------------------------------------------------
    printf("\n");
    printf("\nEnter a Test No. and press the return key to run test\n");
    printf("Press 'q' to close test menu\n");
    char testNoID = getchar();
    int loop = 0;
    getchar(); // To consume '\n'

    switch (testNoID)
    {
    case '0':
    {
        printf("\n.-.--..---.-.-.--.--.--.---.--.-\n");
        printf("TEST NO. : %c\n", testNoID);
        printf("LEDs RG (notif) validation\n");
        while (loop != 27)
        {
            printf("\nGREEN LED ON - ENTER for next test");
            _alarm.TurnOnGreenLed();
            _alarm.TurnOffRedLed();
            getchar();

            printf("RED LED ON - ENTER for next test");
            _alarm.TurnOnRedLed();
            _alarm.TurnOffGreenLed();
            getchar();

            printf("BLINK GREEN-RED - ENTER to power off LEDs");
            _alarm.TurnOnBlinkLedsAlarmThread().detach();
            getchar();
            _alarm.StopBlinkLedsAlarm();

            printf("\nENTER to restart test sequence\n");
            printf("ESC+ENTER to exit test sequence\n");
            printf(".-.--..---.-.-.--.--.--.---.--.-\n");
            loop = getchar();
        }
        getchar();
        break;
    }
    case '1':
    {
        printf("\n.-.--..---.-.-.--.--.--.---.--.-\n");
        printf("TEST NO. : %c\n", testNoID);
        printf("DC motor (notif) validation\n");
        while (loop != 27)
        {
            printf("\nDC MOTOR/BUZZER ON - ENTER to power off DC motor");
            _alarm.TurnOnDCMotor();
            getchar();
            _alarm.TurnOffDCMotor();

            printf("\nENTER to repeat test\n");
            printf("ESC+ENTER to exit test\n");
            printf(".-.--..---.-.-.--.--.--.---.--.-\n");
            loop = getchar();
        }
        getchar();
        break;
    }
    case '2':
    {
        printf("\n.-.--..---.-.-.--.--.--.---.--.-\n");
        printf("TEST NO. : %c\n", testNoID);
        printf("Red blink alarm (notif) validation\n");
        while (loop != 27)
        {
            printf("\nALARM ON - ENTER to stop alarm");
            _alarm.TurnOnBlinkRedAlarmThread().detach();
            getchar();
            _alarm.StopBlinkRedAlarm();

            printf("\nENTER to repeat test\n");
            printf("ESC+ENTER to exit test\n");
            printf(".-.--..---.-.-.--.--.--.---.--.-\n");
            loop = getchar();
        }
        getchar();
        break;
    }
    case '3':
    {
        printf("\n.-.--..---.-.-.--.--.--.---.--.-\n");
        printf("TEST NO. : %c\n", testNoID);
        printf("Blink alarm 5 seconds (notif) validation\n");
        while (loop != 27)
        {
            printf("\nALARM ON - 5 seconds");

            _alarm.TurnOnBlinkLedsAlarmThread().detach();
            _alarm.TurnOnDCMotor();
            sleep_for_milliseconds(5000);
            _alarm.TurnOffDCMotor();
            _alarm.StopBlinkLedsAlarm();
            for (int i = 0; i < 100000; i++)
            {
                if (_alarm.ButtonPressed())
                {
                    printf("BUTTON PRESSED\n");
                }
                else
                {
                    printf("BUTTON NOT PRESSED\n");
                }
                sleep_for_milliseconds(100);
            }

            printf("\nENTER to repeat test\n");
            printf("ESC+ENTER to exit test\n");
            printf(".-.--..---.-.-.--.--.--.---.--.-\n");
            loop = getchar();
        }
        getchar();
        break;
    }
    case '5':
    {
        printf("\n.-.--..---.-.-.--.--.--.---.--.-\n");
        printf("TEST NO. : %c\n", testNoID);
        printf("Force sensors (ADC) validation\n");
        while (loop != 27)
        {
            _max11611.GetData(PRESSURE_SENSOR_COUNT, _max11611Data);
            for (uint8_t i = 0; i < PRESSURE_SENSOR_COUNT; i++)
            {
                _forceSensor.SetAnalogData(i, _max11611Data[i]);
            }
            printf("\nFORCE SENSORS VALUES\n");
            printf("Sensor Number \t Analog value\n");
            for (uint8_t i = 0; i < PRESSURE_SENSOR_COUNT; i++)
            {
                printf("Sensor No: %i \t %i\n", i + 1, (_forceSensor.GetAnalogData(i)));
            }
            printf("\nENTER to repeat test\n");
            printf("ESC+ENTER to exit test\n");
            printf(".-.--..---.-.-.--.--.--.---.--.-\n");
            loop = getchar();
        }
        getchar();
        break;
    }
    case '6':
    {
        printf("\n.-.--..---.-.-.--.--.--.---.--.-\n");
        printf("TEST NO. : %c\n", testNoID);
        printf("Force sensors calibration validation\n");
        while (loop != 27)
        {
            _max11611.GetData(PRESSURE_SENSOR_COUNT, _max11611Data);
            for (uint8_t i = 0; i < PRESSURE_SENSOR_COUNT; i++)
            {
                _forceSensor.SetAnalogData(i, _max11611Data[i]);
            }
            printf("\nCalibration Start ...");
            const uint8_t maxIterations = 5; //5s instead of 10s to acelerate tests
            _forceSensor.CalibrateForceSensor(_max11611, _max11611Data, maxIterations);

            printf("\nINITIAL OFFSET VALUES\n");
            printf("Sensor Number\tAnalog Offset\n");
            pressure_mat_offset_t forceSensor = _forceSensor.GetOffsets();
            for (uint8_t i = 0; i < PRESSURE_SENSOR_COUNT; i++)
            {
                printf("Sensor No: %i \t %u \n", i + 1, forceSensor.analogOffset[i]);
            }
            printf("\nSensor mean from calibration : \t %f \n", forceSensor.totalSensorMean);
            printf("Detection Threshold : %f \n", forceSensor.detectionThreshold);

            printf("\nENTER to repeat test\n");
            printf("ESC+ENTER to exit test\n");
            printf(".-.--..---.-.-.--.--.--.---.--.-\n");
            loop = getchar();
        }
        getchar();
        break;
    }
    case '7':
    {
        printf("\n.-.--..---.-.-.--.--.--.---.--.-\n");
        printf("TEST NO. : %c\n", testNoID);
        printf("Presence detection validation\n");
        while (loop != 27)
        {
            _max11611.GetData(PRESSURE_SENSOR_COUNT, _max11611Data);
            for (uint8_t i = 0; i < PRESSURE_SENSOR_COUNT; i++)
            {
                _forceSensor.SetAnalogData(i, _max11611Data[i]);
            }
            uint16_t sensedPresence = 0;
            for (uint8_t i = 0; i < PRESSURE_SENSOR_COUNT; i++)
            {
                sensedPresence += _forceSensor.GetAnalogData(i);
            }
            if (PRESSURE_SENSOR_COUNT != 0)
            {
                sensedPresence /= PRESSURE_SENSOR_COUNT;
            }
            pressure_mat_offset_t forceSensor = _forceSensor.GetOffsets();
            printf("\nSensed presence (mean(Analog Value)) = %i\n", sensedPresence);
            printf("Detection Threshold set to : %f \n", forceSensor.detectionThreshold);
            printf("Presence detection result : ");
            if (_forceSensor.IsUserDetected())
            {
                printf("User detected \n");
            }
            else
            {
                printf("No user detected \n");
            }
            printf("\nENTER to repeat test\n");
            printf("ESC+ENTER to exit test\n");
            printf(".-.--..---.-.-.--.--.--.---.--.-\n");
            loop = getchar();
        }
        getchar();
        break;
    }
    case '8':
    {
        printf("\n.-.--..---.-.-.--.--.--.---.--.-\n");
        printf("TEST NO. : %c\n", testNoID);
        printf("Force plates validation\n");
        while (loop != 27)
        {
            _max11611.GetData(PRESSURE_SENSOR_COUNT, _max11611Data);
            for (uint8_t i = 0; i < PRESSURE_SENSOR_COUNT; i++)
            {
                _forceSensor.SetAnalogData(i, _max11611Data[i]);
            }
            _forcePlates.UpdateForcePlateData();
            pressure_mat_data_t forcePlates = _forcePlates.GetPressureMatData();
            printf("\nFORCE PLATES CENTER OF PRESSURES\n");
            printf("Relative position of the center of pressure for each quadrants (inches) \n");
            printf("COP Axis \t forcePlate1 \t forcePlate2 \t forcePlate3 \t forcePlate4 \n");
            printf("COP (X): \t %f \t %f \t %f \t %f \n",
                   forcePlates.quadrantPressure[1].x,
                   forcePlates.quadrantPressure[2].x,
                   forcePlates.quadrantPressure[3].x,
                   forcePlates.quadrantPressure[4].x);
            printf("COP (Y): \t %f \t\%f \t %f \t %f \n",
                   forcePlates.quadrantPressure[1].y,
                   forcePlates.quadrantPressure[2].y,
                   forcePlates.quadrantPressure[3].y,
                   forcePlates.quadrantPressure[4].y);

            printf("\nENTER to repeat test\n");
            printf("ESC+ENTER to exit test\n");
            printf(".-.--..---.-.-.--.--.--.---.--.-\n");
            loop = getchar();
        }
        getchar();
        break;
    }
    case 'q':
    {
        return true;
    }
    default:
    {
        printf("\nInvalid testNoID = %i\n", testNoID);
    }
    }
    return false;
}
// if (inSerialChar == 'a')

// printf("\n\nFunction testing :");
// printf("\n\tTestID\tDescription");

// printf("\n\t d \t Activate blink leds alarm.");
// printf("\n\t e \t Activate red alarm.");
// //printf("\n\t f \t Activate force sensors calibration");
// //printf("\n\t g \t Check force sensors centers of pressure");
// printf("\n\t i \t Activate IMU calibration");
// //printf("\n\t j \t Detect relative pressure in quadrants");
// printf("\n\t k \t Print date and time");
// printf("\n\t q \t Close program");
//
// printf("\n\nType in TestID then press the return key\n");
//
// char inSerialChar = getchar();
// getchar(); // To consume '\n'
//
// //---------------------------------------------------------------------------------------
// // TEST DU MODULE NOTIFICATION : OK
// // DEL Red/Green, MoteurDC, Bouton poussoir
// //---------------------------------------------------------------------------------------
//
// if (inSerialChar == 'a')
// {
//     printf("Allumer la DEL verte.\n");
//     _alarm.TurnOnGreenLed();
// }
// else if (inSerialChar == 'b')
// {
//     printf("Allumer la DEL rouge.\n");
//     _alarm.TurnOnRedLed();
// }
// else if (inSerialChar == 'c')
// {
//     printf("Activer le moteur DC.\n");
//     uint8_t count = 0;
//     while (count++ < 70)
//     {
//         _alarm.TurnOnDCMotor();
//         sleep_for_microseconds(0.1 * 1000 * 1000);
//     }
//     _alarm.TurnOffDCMotor();
// }
// else if (inSerialChar == 'd')
// {
//     printf("Activer une alarme <rouge/verte>.\n");
//     _alarm.TurnOnBlinkLedsAlarm();
// }
// else if (inSerialChar == 'e')
// {
//     printf("Activer une alarme <rouge/moteur>.\n");
//     // _alarm.TurnOnRedAlarm();
// }
// /*
// else if (inSerialChar == 'g')
// {
//
//     uint16_t sensedPresence = 0;
//     for (uint8_t i = 0; i < PRESSURE_SENSOR_COUNT; i++)
//     {
//         sensedPresence += _sensorMatrix.GetAnalogData(i);
//     }
//     if (PRESSURE_SENSOR_COUNT != 0)
//     {
//         sensedPresence /= PRESSURE_SENSOR_COUNT;
//     }
//
//     printf("\nDEBUG CENTER OF PRESSURE FORCE SENSORS START");
//     printf("\nFunction(s) under test:");
//     printf("\n DetectCenterOfPressure()");
//     printf("\n\t CreateForcePlate()");
//     printf("\n\t AnalyseForcePlate()");
//
//     UpdateForcePlateData();
//     _globalForcePlate.DetectCenterOfPressure(_globalForcePlate, _sensorMatrix);
//
//     printf("\n.-.--..---MESURE DES CAPTEURS DE FORCE--.---.--.-\n");
//     printf("Sensor Number \t Analog value \t Voltage (mV) \t Force (N) \n");
//     for (uint8_t i = 0; i < PRESSURE_SENSOR_COUNT; i++)
//     {
//         printf("Sensor No: %i \t %i \t\t %u \t\t %f \n", i + 1, _sensorMatrix.GetAnalogData(i), _sensorMatrix.GetVoltageData(i), _sensorMatrix.GetForceData(i));
//     }
//     printf(".-.--..---.-.-.--.--.--.---.--.-\n");
//
//     printf("\n.-.--..---MESURE DES CENTRES DE PRESSION.--.---.--.-\n");
//     printf("Position relative au centre des quadrants sur le siege (cm) \n");
//     printf("COP Axis \t forcePlate1 \t forcePlate2 \t forcePlate3 \t forcePlate4 \n");
//     printf("COP (X): \t %f \t %f \t %f \t %f \n", _globalForcePlate.GetFp1COPx(), _globalForcePlate.GetFp2COPx(), _globalForcePlate.GetFp3COPx(), _globalForcePlate.GetFp4COPx());
//     printf("COP (Y): \t %f \t\%f \t %f \t %f \n", _globalForcePlate.GetFp1COPy(), _globalForcePlate.GetFp2COPy(), _globalForcePlate.GetFp3COPy(), _globalForcePlate.GetFp4COPy());
//
//     printf("\nPosition globale relative au centre du siege (cm) \n");
//     printf("COP Axis \t globalForcePlate \n");
//     printf("COP (X): \t %f \n", _globalForcePlate.GetCOPx());
//     printf("COP (Y): \t %f \n", _globalForcePlate.GetCOPy());
//     printf(".-.--..---.-.-.--.--.--.---.--.-");
//     printf("\n");
//
//     printf("\n.-.--..---DETECTION DUNE PERSONNE SUR LA CHAISE--.---.--.-\n");
//     printf("Detected Presence : %u \n", sensedPresence);
//     printf("Detection Threshold : %f \n", _sensorMatrix.GetDetectionThreshold());
//     printf("Presence verification result : ");
//
//     if (_sensorMatrix.IsUserDetected())
//     {
//         printf("User detected \n");
//     }
//     else
//     {
//         printf("No user detected \n");
//     }
//     printf(".-.--..---.-.-.--.--.--.---.--.-\n\n");
// }
// */
// else if (inSerialChar == 'h')
// {
//     printf("Eteindre les DELs et arrêter le moteur DC.\n");
//     // _alarm.TurnOffAlarm();
//     _backSeatAngleTracker.GetBackSeatAngle();
// }
// else if (inSerialChar == 'i')
// {
//     //_imu.SetCalibrationArray(fixedImu, 0);
//     delay(50);
//     //_imu.SetCalibrationArray(imuMobile, 0);
//     printf("Calibration des capteurs effectuée.\n");
//     inSerialChar = 'x';
// }
// /*
// else if (inSerialChar == 'j')
// {
//     printf("\nDEBUG RELATIVE PRESSURE IN QUADRANTS START");
//     printf("\nFunction(s) under test:");
//     printf("\n DetectRelativePressure()\n");
//
//     UpdateForcePlateData();
//
//     printf("\n.-.--..---MESURE DES PRESSIONS RELATIVES DES QUADRANTS--.---.--.-\n");
//     int *relativePressureLevel = _sensorMatrix.DetectRelativePressure();
//     for (int i = 0; i < 4; i++)
//     {
//         printf("\nQuadrant %d : ", (i + 1));
//         //printf(" %d \n", *(relativePressureLevel + i));
//         if (*(relativePressureLevel + i) == 1)
//         {
//             printf("Really low");
//         }
//         else if (*(relativePressureLevel + i) == 2)
//         {
//             printf("Low");
//         }
//         else if (*(relativePressureLevel + i) == 3)
//         {
//             printf("Normal");
//         }
//         else if (*(relativePressureLevel + i) == 4)
//         {
//             printf("High");
//         }
//         else if (*(relativePressureLevel + i) == 5)
//         {
//             printf("Really high");
//         }
//         else if (*(relativePressureLevel + i) == 0)
//         {
//             printf("No pressure data to read");
//         }
//         else
//         {
//             //Reading error
//         }
//     }
//     printf("\n");
// }
// */
// else if (inSerialChar == 'k')
// {
//     int timeSinceEpoch = _datetimeRTC->GetTimeSinceEpoch();
//     printf("Time since epoch: %d\n", timeSinceEpoch);
// }
// if (inSerialChar == 'q')
// {
//     return true;
// }
// if (inSerialChar == 'a')

// printf("\n\nFunction testing :");
// printf("\n\tTestID\tDescription");

// printf("\n\t d \t Activate blink leds alarm.");
// printf("\n\t e \t Activate red alarm.");
// //printf("\n\t f \t Activate force sensors calibration");
// //printf("\n\t g \t Check force sensors centers of pressure");
// printf("\n\t i \t Activate IMU calibration");
// //printf("\n\t j \t Detect relative pressure in quadrants");
// printf("\n\t k \t Print date and time");
// printf("\n\t q \t Close program");
//
// printf("\n\nType in TestID then press the return key\n");
//
// char inSerialChar = getchar();
// getchar(); // To consume '\n'
//
// //---------------------------------------------------------------------------------------
// // TEST DU MODULE NOTIFICATION : OK
// // DEL Red/Green, MoteurDC, Bouton poussoir
// //---------------------------------------------------------------------------------------
//
// if (inSerialChar == 'a')
// {
//     printf("Allumer la DEL verte.\n");
//     _alarm.TurnOnGreenLed();
// }
// else if (inSerialChar == 'b')
// {
//     printf("Allumer la DEL rouge.\n");
//     _alarm.TurnOnRedLed();
// }
// else if (inSerialChar == 'c')
// {
//     printf("Activer le moteur DC.\n");
//     uint8_t count = 0;
//     while (count++ < 70)
//     {
//         _alarm.TurnOnDCMotor();
//         sleep_for_microseconds(0.1 * 1000 * 1000);
//     }
//     _alarm.TurnOffDCMotor();
// }
// else if (inSerialChar == 'd')
// {
//     printf("Activer une alarme <rouge/verte>.\n");
//     _alarm.TurnOnBlinkLedsAlarm();
// }
// else if (inSerialChar == 'e')
// {
//     printf("Activer une alarme <rouge/moteur>.\n");
//     // _alarm.TurnOnRedAlarm();
// }
// /*
// else if (inSerialChar == 'g')
// {
//
//     uint16_t sensedPresence = 0;
//     for (uint8_t i = 0; i < PRESSURE_SENSOR_COUNT; i++)
//     {
//         sensedPresence += _sensorMatrix.GetAnalogData(i);
//     }
//     if (PRESSURE_SENSOR_COUNT != 0)
//     {
//         sensedPresence /= PRESSURE_SENSOR_COUNT;
//     }
//
//     printf("\nDEBUG CENTER OF PRESSURE FORCE SENSORS START");
//     printf("\nFunction(s) under test:");
//     printf("\n DetectCenterOfPressure()");
//     printf("\n\t CreateForcePlate()");
//     printf("\n\t AnalyseForcePlate()");
//
//     UpdateForcePlateData();
//     _globalForcePlate.DetectCenterOfPressure(_globalForcePlate, _sensorMatrix);
//
//     printf("\n.-.--..---MESURE DES CAPTEURS DE FORCE--.---.--.-\n");
//     printf("Sensor Number \t Analog value \t Voltage (mV) \t Force (N) \n");
//     for (uint8_t i = 0; i < PRESSURE_SENSOR_COUNT; i++)
//     {
//         printf("Sensor No: %i \t %i \t\t %u \t\t %f \n", i + 1, _sensorMatrix.GetAnalogData(i), _sensorMatrix.GetVoltageData(i), _sensorMatrix.GetForceData(i));
//     }
//     printf(".-.--..---.-.-.--.--.--.---.--.-\n");
//
//     printf("\n.-.--..---MESURE DES CENTRES DE PRESSION.--.---.--.-\n");
//     printf("Position relative au centre des quadrants sur le siege (cm) \n");
//     printf("COP Axis \t forcePlate1 \t forcePlate2 \t forcePlate3 \t forcePlate4 \n");
//     printf("COP (X): \t %f \t %f \t %f \t %f \n", _globalForcePlate.GetFp1COPx(), _globalForcePlate.GetFp2COPx(), _globalForcePlate.GetFp3COPx(), _globalForcePlate.GetFp4COPx());
//     printf("COP (Y): \t %f \t\%f \t %f \t %f \n", _globalForcePlate.GetFp1COPy(), _globalForcePlate.GetFp2COPy(), _globalForcePlate.GetFp3COPy(), _globalForcePlate.GetFp4COPy());
//
//     printf("\nPosition globale relative au centre du siege (cm) \n");
//     printf("COP Axis \t globalForcePlate \n");
//     printf("COP (X): \t %f \n", _globalForcePlate.GetCOPx());
//     printf("COP (Y): \t %f \n", _globalForcePlate.GetCOPy());
//     printf(".-.--..---.-.-.--.--.--.---.--.-");
//     printf("\n");
//
//     printf("\n.-.--..---DETECTION DUNE PERSONNE SUR LA CHAISE--.---.--.-\n");
//     printf("Detected Presence : %u \n", sensedPresence);
//     printf("Detection Threshold : %f \n", _sensorMatrix.GetDetectionThreshold());
//     printf("Presence verification result : ");
//
//     if (_sensorMatrix.IsUserDetected())
//     {
//         printf("User detected \n");
//     }
//     else
//     {
//         printf("No user detected \n");
//     }
//     printf(".-.--..---.-.-.--.--.--.---.--.-\n\n");
// }
// */
// else if (inSerialChar == 'h')
// {
//     printf("Eteindre les DELs et arrêter le moteur DC.\n");
//     // _alarm.TurnOffAlarm();
//     _backSeatAngleTracker.GetBackSeatAngle();
// }
// else if (inSerialChar == 'i')
// {
//     //_imu.SetCalibrationArray(fixedImu, 0);
//     delay(50);
//     //_imu.SetCalibrationArray(imuMobile, 0);
//     printf("Calibration des capteurs effectuée.\n");
//     inSerialChar = 'x';
// }
// /*
// else if (inSerialChar == 'j')
// {
//     printf("\nDEBUG RELATIVE PRESSURE IN QUADRANTS START");
//     printf("\nFunction(s) under test:");
//     printf("\n DetectRelativePressure()\n");
//
//     UpdateForcePlateData();
//
//     printf("\n.-.--..---MESURE DES PRESSIONS RELATIVES DES QUADRANTS--.---.--.-\n");
//     int *relativePressureLevel = _sensorMatrix.DetectRelativePressure();
//     for (int i = 0; i < 4; i++)
//     {
//         printf("\nQuadrant %d : ", (i + 1));
//         //printf(" %d \n", *(relativePressureLevel + i));
//         if (*(relativePressureLevel + i) == 1)
//         {
//             printf("Really low");
//         }
//         else if (*(relativePressureLevel + i) == 2)
//         {
//             printf("Low");
//         }
//         else if (*(relativePressureLevel + i) == 3)
//         {
//             printf("Normal");
//         }
//         else if (*(relativePressureLevel + i) == 4)
//         {
//             printf("High");
//         }
//         else if (*(relativePressureLevel + i) == 5)
//         {
//             printf("Really high");
//         }
//         else if (*(relativePressureLevel + i) == 0)
//         {
//             printf("No pressure data to read");
//         }
//         else
//         {
//             //Reading error
//         }
//     }
//     printf("\n");
// }
// */
// else if (inSerialChar == 'k')
// {
//     int timeSinceEpoch = _datetimeRTC->GetTimeSinceEpoch();
//     printf("Time since epoch: %d\n", timeSinceEpoch);
// }
// if (inSerialChar == 'q')
// {
//     return true;
// }
