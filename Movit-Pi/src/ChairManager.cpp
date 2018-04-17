#include "ChairManager.h"

#define REQUIRED_SITTING_TIME 5
#define DELTA_ANGLE_THRESHOLD 5

#define CENTER_OF_PRESSURE_EMISSION_PERIOD 5 * 60
#define MINIMUM_BACK_REST_ANGLE 2

ChairManager::ChairManager(MosquittoBroker *mosquittoBroker, DeviceManager *devicemgr)
    : _mosquittoBroker(mosquittoBroker), _devicemgr(devicemgr)
{
    _copCoord.x = 0;
    _copCoord.y = 0;
}

void ChairManager::UpdateDevices()
{
    if (_mosquittoBroker->CalibPressureMatRequired())
    {
        printf("Debut de la calibration du tapis de pression\n");
        _devicemgr->CalibratePressureMat();
        printf("FIN de la calibration du tapis de pression\n");
    }

    if (_mosquittoBroker->CalibIMURequired())
    {
        printf("Debut de la calibration des IMU\n");
        _devicemgr->CalibrateIMU();
        printf("FIN de la calibration des IMU\n");
    }

    _prevIsSomeoneThere = _isSomeoneThere;
    _devicemgr->Update();
    _currentDatetime = std::to_string(_devicemgr->GetTimeSinceEpoch());
    _isSomeoneThere = _devicemgr->IsSomeoneThere();
    Coord_t tempCoord = _devicemgr->GetCenterOfPressure();
    _copCoord.x += tempCoord.x;
    _copCoord.y += tempCoord.y;
    _prevChairAngle = _currentChairAngle;
    _currentChairAngle = _devicemgr->GetBackSeatAngle();
    _isMoving = _devicemgr->GetIsMoving();

#ifdef DEBUG_PRINT
    printf("getDateTime = %s \n", _currentDatetime.c_str());
    printf("isSomeoneThere = %i \n", _devicemgr->IsSomeoneThere());
    printf("getCenterOfPressure x = %f, y = %f \n", _copCoord.x, _copCoord.y);
    printf("_currentChairAngle = %i \n", _currentChairAngle);
    printf("_prevChairAngle = %i \n", _prevChairAngle);
    printf("Is Moving = %d \n", _isMoving);
#endif

    // Envoi de la moyenne de la position dans les 5 dernieres minutes.
    // TODO Ceci est temporaire, il va falloir envoyer le centre de pression quand il y a un changement majeur.
    // Ceci sera revue en même temps que tous le scheduling
    if (_timer.Elapsed() >= CENTER_OF_PRESSURE_EMISSION_PERIOD * 1000)
    {
        _timer.Reset();
        // Tester cette ligne avec la chaise
        // _mosquittoBroker->SendCenterOfPressure(_copCoord.x/CENTER_OF_PRESSURE_EMISSION_PERIOD, _copCoord.y/CENTER_OF_PRESSURE_EMISSION_PERIOD, _currentDatetime);

        _copCoord.x = 0;
        _copCoord.y = 0;
    }

    if (_currentChairAngle != _prevChairAngle)
    {
        _mosquittoBroker->SendBackRestAngle(_currentChairAngle, _currentDatetime);
    }

    if (_prevIsSomeoneThere != _isSomeoneThere)
    {
        _mosquittoBroker->SendIsSomeoneThere(_isSomeoneThere, _currentDatetime);
    }

    // A rajouter quand le moment sera venu
    //_mosquittoBroker->SendSpeed(0, _currentDatetime);
}

void ChairManager::ReadFromServer()
{
    if (_mosquittoBroker->IsSetAlarmOnNew())
    {
        _overrideNotificationPattern = true;
        _setAlarmOn = _mosquittoBroker->GetSetAlarmOn();
        printf("Something new for setAlarmOn = %i\n", _setAlarmOn);
    }
    if (_mosquittoBroker->IsRequiredBackRestAngleNew())
    {
        _requiredBackRestAngle = _mosquittoBroker->GetRequiredBackRestAngle();
        _secondsCounter = 0;
        // TODO valider que c'est le bon _state
        _state = 1;
        printf("Something new for _requiredBackRestAngle = %i\n", _requiredBackRestAngle);
    }
    if (_mosquittoBroker->IsRequiredPeriodNew())
    {
        _requiredPeriod = _mosquittoBroker->GetRequiredPeriod();
        _secondsCounter = 0;
        // TODO valider que c'est le bon _state
        _state = 1;
        printf("Something new for _requiredPeriod = %i\n", _requiredPeriod);
    }
    if (_mosquittoBroker->IsRequiredDurationNew())
    {
        _requiredDuration = _mosquittoBroker->GetRequiredDuration();
        _secondsCounter = 0;
        // TODO valider que c'est le bon _state
        _state = 1;
        printf("Something new for _requiredDuration = %i\n", _requiredDuration);
    }
}

void ChairManager::CheckNotification()
{
    if (_overrideNotificationPattern)
    {
        OverrideNotificationPattern();
        return;
    }

    if (_requiredDuration == 0 || _requiredPeriod == 0 || _requiredBackRestAngle == 0)
    {
        _state = 1;
        _secondsCounter = 0;
        _devicemgr->GetAlarm()->TurnOffAlarm();
        return;
    }

    switch (_state)
    {
    case 1:
        CheckIfUserHasBeenSittingForFiveSeconds();
        break;
    case 2:
        CheckIfBackRestIsRequired();
        break;
    case 3:
        CheckIfRequiredBackSeatAngleIsReached();
        break;
    case 4:
        CheckIfRequiredBackSeatAngleIsMaintained();
        break;
    case 5:
        CheckIfBackSeatIsBackToInitialPosition();
        break;
    default:
        break;
    }
}

void ChairManager::CheckIfUserHasBeenSittingForFiveSeconds()
{
    printf("State 1\t_secondsCounter: %i\n", _secondsCounter);

    if (++_secondsCounter >= REQUIRED_SITTING_TIME)
    {
        _state = 2;
        _secondsCounter = 0;
    }
}

void ChairManager::CheckIfBackRestIsRequired()
{
    printf("State 2\t_secondsCounter: %i\n", _secondsCounter);

    if (++_secondsCounter >= _requiredPeriod)
    {
        _devicemgr->GetAlarm()->TurnOnRedAlarmThread().detach();
        _state = 3;
        _secondsCounter = 0;
    }
}

void ChairManager::CheckIfRequiredBackSeatAngleIsReached()
{
    printf("State 3\t abs(requiredBackRestAngle - _currentChairAngle): %i\n", abs(int(_requiredBackRestAngle) - int(_currentChairAngle)));

    if (_currentChairAngle > (_requiredBackRestAngle - DELTA_ANGLE_THRESHOLD))
    {
        _devicemgr->GetAlarm()->TurnOnGreenAlarm();
        _state = 4;
    }
}

void ChairManager::CheckIfRequiredBackSeatAngleIsMaintained()
{
    printf("State 4\n");

    if (_currentChairAngle > (_requiredBackRestAngle - DELTA_ANGLE_THRESHOLD))
    {
        printf("State 4\t_secondsCounter: %i\n", _secondsCounter);

        if (++_secondsCounter >= _requiredDuration)
        {
            _devicemgr->GetAlarm()->TurnOnBlinkLedsAlarmThread().detach();
            _state = 5;
        }
    }
    else
    {
        printf("State 4 il faut remonter\n");
        _devicemgr->GetAlarm()->TurnOnRedAlarmThread().detach();
        _state = 3;
        _secondsCounter = 0;
    }
}

void ChairManager::CheckIfBackSeatIsBackToInitialPosition()
{
    printf("State 5\t_secondsCounter: %i\n", ++_secondsCounter);

    if (_currentChairAngle < MINIMUM_BACK_REST_ANGLE)
    {
        _state = 2;
        _secondsCounter = 0;
    }
}

void ChairManager::OverrideNotificationPattern()
{
    if (_setAlarmOn)
    {
        _devicemgr->GetAlarm()->TurnOnRedAlarmThread().detach();
    }
    else
    {
        _devicemgr->GetAlarm()->TurnOffAlarm();
    }
    _overrideNotificationPattern = false;
}
