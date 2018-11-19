#include "DeviceManager.h"
#include "NetworkManager.h"
#include "FixedImu.h"
#include "MobileImu.h"
#include "I2Cdev.h"
#include "Utils.h"
#include "MAX11611.h"
#include "ForceSensor.h"

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

bool DeviceManager::TestDevices()
{
    printf("\n.-.--..--- TEST MENU .--.---.--.-\n");
    // printf("\n\nFunction testing :");
    // printf("\n\tTestID\tDescription");
    //--------------------------------------------------------------------------
    // MODULE : Raspberry Pi Header
    // PCB Validation tests:
    // 1
    // 2
    // 3
    // Functionnal tests:
    // 1
    // 2
    //--------------------------------------------------------------------------
    // MODULE : Pressure Sensor
    // PCB Validation tests:
    printf("\n\t 1 \t Force sensors (ADC) validation");
    // Functionnal tests:
    printf("\n\t 2 \t Force sensors calibration validation");
    printf("\n\t 3 \t Force plates validation");
    printf("\n\t 4 \t Centers of pressure validation");
    //--------------------------------------------------------------------------
    printf("\n");
    printf("\nEnter a Test No. and press the return key to run test\n");
    char testNoID = getchar();
    getchar(); // To consume '\n'

    switch(testNoID)
    {
      case '1':
      {
        printf("\nTEST: case 1\n");
        break;
      }
      case '2':
      {
        printf("\nTEST: case 2\n");
        break;
      }
      case '3':
      {
        printf("\nTEST: case 3\n");
        for (uint8_t i = 0; i < PRESSURE_SENSOR_COUNT; i++)
        {
           printf("\nTEST 1: case 3 in for round %i\n", i);
            _forceSensor->SetAnalogData(i, _max11611Data[i]);
        }

        float sensedPresence = 0;
        for (uint8_t i = 0; i < PRESSURE_SENSOR_COUNT; i++)
        {
          printf("\nTEST: case 3 in for round %i\n", i);
          printf("\nTEST: psensorcount %i\n", PRESSURE_SENSOR_COUNT);
          printf("\nTEST: psensorcount %f\n", sensedPresence);
          sensedPresence += static_cast<float>(_forceSensor->GetAnalogData(i));
        }

              printf("\nTEST: case 3 between for and if\n");

        if (PRESSURE_SENSOR_COUNT != 0)
        {
            sensedPresence /= PRESSURE_SENSOR_COUNT;
        }
        printf("\n.-.--..---MESURE DES CAPTEURS DE FORCE--.---.--.-\n");
        printf("Sensor Number \t Analog value \t Voltage (mV) \t Force (N) \n");
        for (uint8_t i = 0; i < PRESSURE_SENSOR_COUNT; i++)
        {
            printf("Sensor No: %i \t %i \t\t %u \t\t %f \n", i + 1, (_forceSensor->GetAnalogData(i)));
        }
        printf(".-.--..---.-.-.--.--.--.---.--.-\n");
        break;
      }
      case '4':
      {
        printf("\nTEST: case 4\n");
        break;
      }
      default:
      {
          printf("\nTEST: default, char value = %c\n", testNoID);
      }
    }
    // if (inSerialChar == 'a')

    // printf("\n\nFunction testing :");
    // printf("\n\tTestID\tDescription");
    // printf("\n\t a \t Activate GREEN LED on notification module");
    // printf("\n\t b \t Activate RED LED on notification module");
    // printf("\n\t c \t Activate DC Motor on notification module");
    // printf("\n\t d \t Activate blink leds alarm.");
    // printf("\n\t e \t Activate red alarm.");
    // //printf("\n\t f \t Activate force sensors calibration");
    // //printf("\n\t g \t Check force sensors centers of pressure");
    // printf("\n\t h \t De-activate all R&G LED + DC Motor");
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
    // else if (inSerialChar == 'f')
    // {
    //
    //     printf("\nDEBUG CALIBRATION FORCE SENSORS START");
    //     printf("\nFunction(s) under test:");
    //     printf("\n CalibrateForceSensor()");
    //     printf("\n IsUserDetected()");
    //
    //     //Calibration
    //     UpdateForcePlateData();
    //     _sensorMatrix.CalibrateForceSensor(_max11611Data, _max11611);
    //     //Last sensed presence analog reading to compare with calibration
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
    //     printf("\n.-.--..---DERNIERE MESURE DES CAPTEURS EN TEMPS REEL.--.---.--.-\n");
    //     printf("Sensor Number \t Analog value \t Voltage (mV) \t Force (N) \n");
    //     for (uint8_t i = 0; i < PRESSURE_SENSOR_COUNT; i++)
    //     {
    //         printf("Sensor No: %i \t %i \t\t %u \t\t  %f \n", i + 1, _sensorMatrix.GetAnalogData(i), _sensorMatrix.GetVoltageData(i), _sensorMatrix.GetForceData(i));
    //     }
    //     printf(".-.--..---.-.-.--.--.--.---.--.-\n");
    //
    //     printf("\n.-.--..---.-.OFFSET INITIAUX.--.---.--.-\n");
    //     printf("Sensor Number\t");
    //     printf("Analog value\n");
    //     for (uint8_t i = 0; i < PRESSURE_SENSOR_COUNT; i++)
    //     {
    //         printf("Sensor No: %i \t %u \n", i + 1, _sensorMatrix.GetAnalogOffset(i));
    //     }
    //     printf(".-.--..---.-.-.--.--.--.---.--.-\n");
    //
    //     printf("\n.-.--..---.-.OFFSET INITIAUX.--.---.--.-\n");
    //     printf("Total sensor mean : \t %i \n", _sensorMatrix.GetTotalSensorMean());
    //     printf(".-.--..---.-.-.--.--.--.---.--.-\n");
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

    return false;
}
