#include "DeviceManager.h"
#include "NetworkManager.h"
#include "FixedImu.h"
#include "MobileImu.h"
#include "I2Cdev.h"
#include "Utils.h"
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
  printf("\n.-.--..--- MAIN MODULES TEST MENU .--.---.--.-\n");
  printf("\n ID \t Module Description");
  printf("\n--------------------------------------");
  printf("\n 1 \t Module Header Raspberry Pi");
  printf("\n 2 \t Module Notification");
  printf("\n 3 \t Module IMU");
  printf("\n 4 \t Module Pressure Mat");
  printf("\n 5 \t Module Camera");
  printf("\n q \t *Close Main Test Menu");
  printf("\n--------------------------------------");
  printf("\n Enter a ModuleID and press ENTER to see module tests\n");

  char moduleID = getchar();
  char testNoID = 99; // out of range value
  int loopTest = 0;
  int subTestExit = 0;
  getchar(); // To consume '\n'

  switch (moduleID)
  {
    case '1':
    {
      //asda




      printf("\nModule Header Raspberry Pi\n");
      break;
    }
    case '2':
    {
      subTestExit = 0;
      while (subTestExit != 1)
      {
        printf("\nMODULE NOTIFICATION\n");

        printf("\n.-.--..--- NOTIFICATION TEST MENU .--.---.--.-\n");
        printf("\nTestID\tDescription");
        printf("\n--------------------------------------");
        printf("\n 1 \t LEDs RG validation");
        printf("\n 2 \t DC motor - buzzer validation");
        printf("\n 3 \t Red blink alarm validation");
        printf("\n 4 \t Push-button validation");
        printf("\n q \t *Back to Main Test Menu");
        printf("\n--------------------------------------");
        printf("\nEnter a Test No. and press the return key to run test\n");
        testNoID = getchar();
        getchar(); // To consume '\n'
        loopTest = 0;

        switch (testNoID)
        {
        case '1':
        {
            printf("\n.-.--..---.-.-.--.--.--.---.--.-\n");
            printf("TEST NO. : %c\n", testNoID);
            printf("LEDs RG validation\n");
            while (loopTest != 27)
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
                loopTest = getchar();
            }
            getchar();
            break;
        }
        case '2':
        {
            printf("\n.-.--..---.-.-.--.--.--.---.--.-\n");
            printf("TEST NO. : %c\n", testNoID);
            printf("DC motor validation\n");
            while (loopTest != 27)
            {
                printf("\nDC MOTOR/BUZZER ON - ENTER to power off DC motor");
                _alarm.TurnOnDCMotor();
                getchar();
                _alarm.TurnOffDCMotor();

                printf("\nENTER to repeat test\n");
                printf("ESC+ENTER to exit test\n");
                printf(".-.--..---.-.-.--.--.--.---.--.-\n");
                loopTest = getchar();
            }
            getchar();
            break;
        }
        case '3':
        {
            printf("\n.-.--..---.-.-.--.--.--.---.--.-\n");
            printf("TEST NO. : %c\n", testNoID);
            printf("Red blink alarm validation\n\n");
            while(loopTest != 27)
            {
              printf("RED ALARM ON (3 sec.) - ENTER to stop alarm\n");
              _alarm.TurnOnBlinkRedAlarmThread().detach();
              int alarmTime = 3;
              for (int i = 0; i < alarmTime; i++)
              {
                printf("%i\n", (alarmTime - i));
                sleep_for_milliseconds(1000);
              }
              _alarm.StopBlinkRedAlarm();

              printf("\nENTER to repeat test\n");
              printf("ESC+ENTER to exit test\n");
              printf(".-.--..---.-.-.--.--.--.---.--.-\n");
              loopTest = getchar();
            }
            getchar();
            break;
        }
        case '4':
        {
            printf("\n.-.--..---.-.-.--.--.--.---.--.-\n");
            printf("TEST NO. : %c\n", testNoID);
            printf("Push-button validation\n");
            while(loopTest != 27)
            {
              printf("\nPUSH BUTTON STATE\n");
              for (int i = 0; i < 5; i++)
              {
                if (_alarm.ButtonPressed())
                {
                  printf("Reading %i : PUSH-BUTTON PRESSED\n", (i+1));
                }
                else
                {
                  printf("Reading %i : ... \n", (i+1));
                }
                sleep_for_milliseconds(1000);
              }
                printf("\nENTER to repeat test\n");
                printf("ESC+ENTER to exit test\n");
                printf(".-.--..---.-.-.--.--.--.---.--.-\n");
                loopTest = getchar();
            }
            getchar();
            break;
          }
          case 'q':
          {
            subTestExit = 1;
            break;
          }
          default:
          {
            printf("\nInvalid TestID = %i\n", testNoID);
          }
          break;
        }
      }
    }
    case '3':
    {
      subTestExit = 0;
      while (subTestExit != 1)
      {
        printf("\nMODULE IMUs\n");

        printf("\n.-.--..--- IMUs TEST MENU .--.---.--.-\n");
        printf("\nTestID\tDescription");
        printf("\n--------------------------------------");
        printf("\n 1 \t asda validation");
        printf("\n--------------------------------------");
        printf("\nEnter a Test No. and press the return key to run test\n");
        testNoID = getchar();
        getchar(); // To consume '\n'
        loopTest = 0;

        switch (testNoID)
        {
          case '1':
          {
            printf("\n.-.--..---.-.-.--.--.--.---.--.-\n");
            printf("TEST NO. : %c\n", testNoID);
            printf("asd validation\n");
            while (loopTest != 27)
            {
              //asdahsd

              printf("\nENTER to restart test sequence\n");
              printf("ESC+ENTER to exit test sequence\n");
              printf(".-.--..---.-.-.--.--.--.---.--.-\n");
              loopTest = getchar();
            }
            getchar();
            break;
          }
          case '2':
          {
            printf("\n.-.--..---.-.-.--.--.--.---.--.-\n");
            printf("TEST NO. : %c\n", testNoID);
            printf("asd validation\n");
            while (loopTest != 27)
            {
              //asdahsd

              printf("\nENTER to restart test sequence\n");
              printf("ESC+ENTER to exit test sequence\n");
              printf(".-.--..---.-.-.--.--.--.---.--.-\n");
              loopTest = getchar();
            }
            getchar();
            break;
          }
          case 'q':
          {
            subTestExit = 1;
            moduleID = 0;
            break;
          }
          default:
          {
            printf("\nInvalid TestID = %i\n", moduleID);
          }
          break;
        }
      }
    }
    case '4':
    {
      printf("\nModule Pressure Mat\n");
      // // MODULE : Pressure Sensor
      // // PCB Validation tests:
      // printf("\n\t 5 \t Force sensors (ADC) validation");
      // // Functionnal tests:
      // printf("\n\t 6 \t Force sensors calibration validation");
      // printf("\n\t 7 \t Presence detection validation");
      // printf("\n\t 8 \t Force plates validation");
      // printf("\n\t 9 \t Centers of pressure validation");

      // case '5':
      // {
      //     printf("\n.-.--..---.-.-.--.--.--.---.--.-\n");
      //     printf("TEST NO. : %c\n", testNoID);
      //     printf("Force sensors (ADC) validation\n");
      //     while (loopTest != 27)
      //     {
      //         _max11611.GetData(PRESSURE_SENSOR_COUNT, _max11611Data);
      //         for (uint8_t i = 0; i < PRESSURE_SENSOR_COUNT; i++)
      //         {
      //             _forceSensor.SetAnalogData(i, _max11611Data[i]);
      //         }
      //         printf("\nFORCE SENSORS VALUES\n");
      //         printf("Sensor Number \t Analog value\n");
      //         for (uint8_t i = 0; i < PRESSURE_SENSOR_COUNT; i++)
      //         {
      //             printf("Sensor No: %i \t %i\n", i + 1, (_forceSensor.GetAnalogData(i)));
      //         }
      //         printf("\nENTER to repeat test\n");
      //         printf("ESC+ENTER to exit test\n");
      //         printf(".-.--..---.-.-.--.--.--.---.--.-\n");
      //         loopTest = getchar();
      //     }
      //     getchar();
      //     break;
      // }
      // case '6':
      // {
      //     printf("\n.-.--..---.-.-.--.--.--.---.--.-\n");
      //     printf("TEST NO. : %c\n", testNoID);
      //     printf("Force sensors calibration validation\n");
      //     while (loopTest != 27)
      //     {
      //         _max11611.GetData(PRESSURE_SENSOR_COUNT, _max11611Data);
      //         for (uint8_t i = 0; i < PRESSURE_SENSOR_COUNT; i++)
      //         {
      //             _forceSensor.SetAnalogData(i, _max11611Data[i]);
      //         }
      //         printf("\nCalibration Start ...");
      //         const uint8_t maxIterations = 5; //5s instead of 10s to acelerate tests
      //         _forceSensor.CalibrateForceSensor(_max11611, _max11611Data, maxIterations);
      //
      //         printf("\nINITIAL OFFSET VALUES\n");
      //         printf("Sensor Number\tAnalog Offset\n");
      //         pressure_mat_offset_t forceSensor = _forceSensor.GetOffsets();
      //         for (uint8_t i = 0; i < PRESSURE_SENSOR_COUNT; i++)
      //         {
      //             printf("Sensor No: %i \t %u \n", i + 1, forceSensor.analogOffset[i]);
      //         }
      //         printf("\nSensor mean from calibration : \t %f \n", forceSensor.totalSensorMean);
      //         printf("Detection Threshold : %f \n", forceSensor.detectionThreshold);
      //
      //         printf("\nENTER to repeat test\n");
      //         printf("ESC+ENTER to exit test\n");
      //         printf(".-.--..---.-.-.--.--.--.---.--.-\n");
      //         loopTest = getchar();
      //     }
      //     getchar();
      //     break;
      // }
      // case '7':
      // {
      //     printf("\n.-.--..---.-.-.--.--.--.---.--.-\n");
      //     printf("TEST NO. : %c\n", testNoID);
      //     printf("Presence detection validation\n");
      //     while (loopTest != 27)
      //     {
      //         _max11611.GetData(PRESSURE_SENSOR_COUNT, _max11611Data);
      //         for (uint8_t i = 0; i < PRESSURE_SENSOR_COUNT; i++)
      //         {
      //             _forceSensor.SetAnalogData(i, _max11611Data[i]);
      //         }
      //         uint16_t sensedPresence = 0;
      //         for (uint8_t i = 0; i < PRESSURE_SENSOR_COUNT; i++)
      //         {
      //             sensedPresence += _forceSensor.GetAnalogData(i);
      //         }
      //         if (PRESSURE_SENSOR_COUNT != 0)
      //         {
      //             sensedPresence /= PRESSURE_SENSOR_COUNT;
      //         }
      //         pressure_mat_offset_t forceSensor = _forceSensor.GetOffsets();
      //         printf("\nSensed presence (mean(Analog Value)) = %i\n", sensedPresence);
      //         printf("Detection Threshold set to : %f \n", forceSensor.detectionThreshold);
      //         printf("Presence detection result : ");
      //         if (_forceSensor.IsUserDetected())
      //         {
      //             printf("User detected \n");
      //         }
      //         else
      //         {
      //             printf("No user detected \n");
      //         }
      //         printf("\nENTER to repeat test\n");
      //         printf("ESC+ENTER to exit test\n");
      //         printf(".-.--..---.-.-.--.--.--.---.--.-\n");
      //         loopTest = getchar();
      //     }
      //     getchar();
      //     break;
      // }
      // case '8':
      // {
      //     printf("\n.-.--..---.-.-.--.--.--.---.--.-\n");
      //     printf("TEST NO. : %c\n", testNoID);
      //     printf("Force plates validation\n");
      //     while (loopTest != 27)
      //     {
      //         _max11611.GetData(PRESSURE_SENSOR_COUNT, _max11611Data);
      //         for (uint8_t i = 0; i < PRESSURE_SENSOR_COUNT; i++)
      //         {
      //             _forceSensor.SetAnalogData(i, _max11611Data[i]);
      //         }
      //         _forcePlates.UpdateForcePlateData();
      //         pressure_mat_data_t forcePlates = _forcePlates.GetPressureMatData();
      //         printf("\nFORCE PLATES CENTER OF PRESSURES\n");
      //         printf("Relative position of the center of pressure for each quadrants (inches) \n");
      //         printf("COP Axis \t forcePlate1 \t forcePlate2 \t forcePlate3 \t forcePlate4 \n");
      //         printf("COP (X): \t %f \t %f \t %f \t %f \n",
      //                forcePlates.quadrantPressure[1].x,
      //                forcePlates.quadrantPressure[2].x,
      //                forcePlates.quadrantPressure[3].x,
      //                forcePlates.quadrantPressure[4].x);
      //         printf("COP (Y): \t %f \t\%f \t %f \t %f \n",
      //                forcePlates.quadrantPressure[1].y,
      //                forcePlates.quadrantPressure[2].y,
      //                forcePlates.quadrantPressure[3].y,
      //                forcePlates.quadrantPressure[4].y);
      //
      //         printf("\nENTER to repeat test\n");
      //         printf("ESC+ENTER to exit test\n");
      //         printf(".-.--..---.-.-.--.--.--.---.--.-\n");
      //         loopTest = getchar();
      //     }
      //     getchar();
      //     break;
      // }
      // case 'q':
      // {
      //     return true;
      // }
      // default:
      // {
      //     printf("\nInvalid testNoID = %i\n", testNoID);
      // }
      // }
      break;
    }
    case '5':
    {
      printf("\nModule Camera\n");
      break;
    }
    case 'q':
    {
        return true;
    }
    default:
    {
        printf("\nInvalid Module ID = %i\n", moduleID);
    }
  }
    return false;
  }


    // //printf("\n\t g \t Check force sensors centers of pressure");
    // printf("\n\t i \t Activate IMU calibration");
    // //printf("\n\t j \t Detect relative pressure in quadrants");
    // printf("\n\t k \t Print date and time");


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
