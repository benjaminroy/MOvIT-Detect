#ifndef CHAIR_MANAGER_H
#define CHAIR_MANAGER_H

#include "MosquittoBroker.h"
#include "Utils.h"
#include "Timer.h"
#include "DeviceManager.h"
#include "SecondsCounter.h"

#include <string>
#include <unistd.h>
#include <chrono>

class ChairManager
{
  public:
    ChairManager(MosquittoBroker *mosquittoBroker, DeviceManager *deviceManager);
    ~ChairManager();

    void UpdateDevices();
    void ReadFromServer();
    void CheckNotification();
    void SetVibrationsActivated(bool isVibrationsActivated);
    std::thread ReadVibrationsThread();

  private:
    static constexpr auto CENTER_OF_PRESSURE_EMISSION_PERIOD = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::seconds(10));
    static constexpr auto FAILED_TILT_TIME = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::minutes(2));
    static constexpr auto CHAIR_ANGLE_EMISSION_PERIOD = std::chrono::milliseconds(1000);
    static constexpr auto WIFI_VALIDATION_PERIOD = std::chrono::seconds(10);
    static constexpr auto HEARTBEAT_PERIOD = std::chrono::milliseconds(1000);

    static constexpr int MINIMUM_ANGLE = 15; // degrees

    Alarm *_alarm;
    MosquittoBroker *_mosquittoBroker;
    DeviceManager *_deviceManager;

    SecondsCounter _secondsCounter;
    uint8_t _state = 0;

    int _currentChairAngle = 0;
    int _prevChairAngle = 0;
    float _snoozeTime = 600.0f; // Default snoozetime = 10 minutes
    std::string _currentDatetime = "";

    bool _isSomeoneThere = false;
    bool _prevIsSomeoneThere = false;
    bool _isMoving = false;
    bool _isChairInclined = false;
    bool _isWifiChanged = false;
    bool _setAlarmOn = false;
    bool _isVibrationsActivated = true;
    bool _isIMUCalibrationChanged = false;
    bool _isPressureMatCalibrationChanged = false;
    bool _overrideNotification = false;

    pressure_mat_data_t _pressureMatData;
    tilt_settings_t _tiltSettings;

    Timer _centerOfPressureTimer;
    Timer _wifiChangedTimer;
    Timer _chairAngleTimer;
    Timer _heartbeatTimer;
    Timer _failedTiltTimer;

    void CheckIfUserHasBeenSittingForRequiredTime();
    void CheckIfBackRestIsRequired();
    void NotificationSnoozed();
    void CheckIfRequiredBackSeatAngleIsReached();
    void CheckIfRequiredBackSeatAngleIsMaintained();
    void CheckIfBackSeatIsBackToInitialPosition();
    void OverrideNotification();
    void ReadVibrations();
};

#endif // CHAIR_MANAGER_H
