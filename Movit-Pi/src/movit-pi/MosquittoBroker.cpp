#include <iostream>
#include <cstdio>
#include <string>
#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"
#include "MosquittoBroker.h"

using rapidjson::Document;
using rapidjson::StringBuffer;
using rapidjson::Value;
using rapidjson::Writer;

// Embarqué à back-end
// data/current_back_rest_angle: entier (angle en degrés) ce qui est envoyé est l'angle de début et l'angle de fin de la bascule
// data/current_center_of_pressure: Coordonées en entier. Format: X:posX,Y:posy
// data/current_is_someone_there: bool (1 = on, 0 = off)
// data/current_chair_speed: float (m/s)

// Back-end à embarqué
// data/required_back_rest_angle: entier (angle en degrés)
// data/required_period: entier (minutes)
// data/required_duration: entier (seconde)
// data/set_alarm: entier (1 = on, 0 = off)

const char *ALARM_TOPIC = "data/set_alarm";
const char *REQUIRED_ANGLE_TOPIC = "data/required_back_rest_angle";
const char *REQUIRED_PERIOD_TOPIC = "data/required_period";
const char *REQUIRED_DURATION_TOPIC = "data/required_duration";
const char *CALIB_PRESSURE_MAT_TOPIC = "config/calib_pressure_mat";
const char *CALIB_IMU_TOPIC = "config/calib_imu";
const char *NOTIFICATIONS_SETTINGS_TOPIC = "config/notifications_settings";
const char *SELECT_WIFI_TOPIC = "config/wifi";

const char *CURRENT_BACK_REST_ANGLE_TOPIC = "data/current_back_rest_angle";
const char *CURRENT_PRESSURE_MAT_DATA_TOPIC = "data/current_pressure_mat_data";
const char *CURRENT_IS_SOMEONE_THERE_TOPIC = "data/current_is_someone_there";
const char *CURRENT_IS_WIFI_CONNECTED_TOPIC = "data/current_is_wifi_connected";
const char *CURRENT_CHAIR_SPEED_TOPIC = "data/current_chair_speed";
const char *HEARTBEAT_TOPIC = "heartbeat/embedded";
const char *VIBRATION_TOPIC = "data/vibration";
const char *IS_MOVING_TOPIC = "data/is_moving";
const char *TILT_INFO_TOPIC = "data/tilt_info";

const char *SENSORS_STATUS_TOPIC = "status/sensors";

const char *EXCEPTION_MESSAGE = "Exception thrown by %s()\n";

MosquittoBroker::MosquittoBroker(const char *id) : mosquittopp(id)
{
    mosqpp::lib_init();

    const char *username = "admin";
    const char *password = "movitplus";

    const char *host = "localhost";
    const int keepAlive = 60;
    const int port = 1883;

    if (username_pw_set(username, password) != MOSQ_ERR_SUCCESS)
    {
        printf("Failed to configure username and password for a mosquitto instance.\n");
        throw;
    }

    connect_async(host, port, keepAlive);

    if (loop_start() != MOSQ_ERR_SUCCESS)
    {
        printf("Failed to to start a new thread to process network traffic.\n");
        throw;
    }
}

MosquittoBroker::~MosquittoBroker()
{
    disconnect();

    if (loop_stop() != MOSQ_ERR_SUCCESS)
    {
        printf("Failed to stop the network thread previously created.\n");
    }

    mosqpp::lib_cleanup();
}

void MosquittoBroker::on_connect(int rc)
{
    printf("Connected with code %d.\n", rc);
    if (rc == 0)
    {
        subscribe(NULL, ALARM_TOPIC);
        subscribe(NULL, REQUIRED_ANGLE_TOPIC);
        subscribe(NULL, REQUIRED_PERIOD_TOPIC);
        subscribe(NULL, REQUIRED_DURATION_TOPIC);
        subscribe(NULL, CALIB_PRESSURE_MAT_TOPIC);
        subscribe(NULL, CALIB_IMU_TOPIC);
        subscribe(NULL, NOTIFICATIONS_SETTINGS_TOPIC);
        subscribe(NULL, SELECT_WIFI_TOPIC);
    }
}

void MosquittoBroker::on_publish(int mid)
{
    // printf("Message published with id %d.\n", mid);
    // uncomment for debug
}

void MosquittoBroker::on_subcribe(int mid, int qos_count, const int *granted_qos)
{
    printf("Subscription succeeded with id %d.\n", mid);
}

void MosquittoBroker::on_message(const mosquitto_message *msg)
{
    std::string message;
    std::string topic;

    if (msg->payload != NULL)
    {
        message = reinterpret_cast<char *>(msg->payload);
    }
    else
    {
        message = "";
    }
    topic = msg->topic;

    if (topic == ALARM_TOPIC)
    {
        try
        {
            _setAlarmOn = std::stoi(message);
            _setAlarmOnNew = true;
        }
        catch (const std::exception &e)
        {
            printf(EXCEPTION_MESSAGE, e.what());
            printf("Setting _setAlarmOn and _setAlarmOnNew to false\n");
            _setAlarmOn = false;
            _setAlarmOnNew = false;
        }
    }
    else if (topic == SELECT_WIFI_TOPIC)
    {
        _wifiChanged = true;
        _wifiInformation = message;
    }
    else if (topic == REQUIRED_ANGLE_TOPIC)
    {
        try
        {
            _tiltSettings.requiredBackRestAngle = std::stoi(message);
            _isTiltSettingsChanged = true;
        }
        catch (const std::exception &e)
        {
            printf(EXCEPTION_MESSAGE, e.what());
            printf("Setting _requiredBackRestAngle to 0 and _requiredBackRestAngleNew to false\n");
            _tiltSettings.requiredBackRestAngle = 0;
            _isTiltSettingsChanged = false;
        }
    }
    else if (topic == REQUIRED_PERIOD_TOPIC)
    {
        try
        {
            _tiltSettings.requiredPeriod = std::stoi(message);
            _isTiltSettingsChanged = true;
        }
        catch (const std::exception &e)
        {
            printf(EXCEPTION_MESSAGE, e.what());
            printf("Setting _requiredPeriod to 0 and _requiredPeriodNew to false\n");
            _tiltSettings.requiredPeriod = 0;
            _isTiltSettingsChanged = false;
        }
    }
    else if (topic == REQUIRED_DURATION_TOPIC)
    {
        try
        {
            _tiltSettings.requiredDuration = std::stoi(message);
            _isTiltSettingsChanged = true;
        }
        catch (const std::exception &e)
        {
            printf(EXCEPTION_MESSAGE, e.what());
            printf("Setting _requiredDuration to 0 and _requiredDurationNew to false\n");
            _tiltSettings.requiredDuration = 0;
            _isTiltSettingsChanged = false;
        }
    }
    else if (topic == CALIB_PRESSURE_MAT_TOPIC)
    {
        try
        {
            _calibPressureMatRequired = std::stoi(message);
        }
        catch (const std::exception &e)
        {
            printf(EXCEPTION_MESSAGE, e.what());
            printf("Setting _calibPressureMatRequired to false\n");
            _calibPressureMatRequired = false;
        }
    }
    else if (topic == CALIB_IMU_TOPIC)
    {
        try
        {
            _calibIMURequired = std::stoi(message);
        }
        catch (const std::exception &e)
        {
            printf(EXCEPTION_MESSAGE, e.what());
            printf("Setting _requiredDuration to 0\n");
            _calibIMURequired = false;
        }
    }
    else if (topic == NOTIFICATIONS_SETTINGS_TOPIC)
    {
        Document document;
        document.Parse(message.c_str());

        // Pour une certaine raison obscure, je ne suis pas capable de gerer le cas
        // ou il n'y a pas d'object notifications_settings. Pas cool rapidjson
        if (document["notifications_settings"].IsObject())
        {
            Value &object = document["notifications_settings"];
            _notificationsSettings.isLedBlinkingEnabled = object["isLedBlinkingEnabled"].GetBool();
            _notificationsSettings.isVibrationEnabled = object["isVibrationEnabled"].GetBool();
            _notificationsSettings.snoozeTime = object["snoozeTime"].GetFloat();
        }

        _isNotificationsSettingsChanged = true;
    }
}

void MosquittoBroker::SendBackRestAngle(const int angle, const std::string datetime)
{
    std::string strAngle = std::to_string(angle);
    std::string strMsg = "{\"datetime\":" + datetime + ",\"angle\":" + strAngle + "}";

    PublishMessage(CURRENT_BACK_REST_ANGLE_TOPIC, strMsg);
}

void MosquittoBroker::SendPressureMatData(const pressure_mat_data_t data, const std::string datetime)
{
    StringBuffer strBuff;
    Writer<StringBuffer> writer(strBuff);

    writer.StartObject();
    writer.Key("datetime");
    writer.String(datetime.c_str());
    writer.Key("center");
    writer.StartObject();
    writer.Key("x");
    writer.Double(data.centerOfPressure.x);
    writer.Key("y");
    writer.Double(data.centerOfPressure.y);
    writer.EndObject();
    writer.Key("quadrants");
    writer.StartArray();
    for (int i = 0; i < 4; i++)
    {
        writer.StartObject();
        writer.Key("x");
        writer.Double(data.quadrantPressure[i].x);
        writer.Key("y");
        writer.Double(data.quadrantPressure[i].y);
        writer.EndObject();
    }
    writer.EndArray();

    writer.EndObject();

    PublishMessage(CURRENT_PRESSURE_MAT_DATA_TOPIC, strBuff.GetString());
}

void MosquittoBroker::SendIsSomeoneThere(const bool state, const std::string datetime)
{
    std::string strState = std::to_string(state);
    std::string strMsg = "{\"datetime\":" + datetime + ",\"IsSomeoneThere\":" + strState + "}";

    PublishMessage(CURRENT_IS_SOMEONE_THERE_TOPIC, strMsg);
}

void MosquittoBroker::SendIsWifiConnected(const bool state, const std::string datetime)
{
    std::string strState = std::to_string(state);
    std::string strMsg = "{\"datetime\":" + datetime + ",\"IsWifiConnected\":" + strState + "}";

    PublishMessage(CURRENT_IS_WIFI_CONNECTED_TOPIC, strMsg);
}

void MosquittoBroker::SendIsPressureMatCalib(const bool state, const std::string datetime)
{
    std::string strState = std::to_string(state);
    std::string strMsg = "{\"datetime\":" + datetime + ",\"IsPressureMatCalib\":" + strState + "}";

    PublishMessage(CALIB_PRESSURE_MAT_TOPIC, strMsg);
}

void MosquittoBroker::SendIsIMUCalib(const bool state, const std::string datetime)
{
    std::string strState = std::to_string(state);
    std::string strMsg = "{\"datetime\":" + datetime + ",\"IsIMUCalib\":" + strState + "}";

    PublishMessage(CALIB_IMU_TOPIC, strMsg);
}

void MosquittoBroker::SendSpeed(const float speed, const std::string datetime)
{
    std::string strSpeed = std::to_string(speed);
    std::string strMsg = "{\"datetime\":" + datetime + ",\"vitesse\":" + strSpeed + "}";

    PublishMessage(CURRENT_CHAIR_SPEED_TOPIC, strMsg);
}

void MosquittoBroker::SendVibration(double acceleration, const std::string datetime)
{
    std::string strAcceleration = std::to_string(acceleration);
    std::string strMsg = "{\"datetime\":" + datetime + ",\"vibration\":" + strAcceleration + "}";

    PublishMessage(VIBRATION_TOPIC, strMsg);
}

void MosquittoBroker::SendIsMoving(const bool state, const std::string datetime)
{
    std::string strState = std::to_string(state);
    std::string strMsg = "{\"datetime\":" + datetime + ",\"isMoving\":" + strState + "}";

    PublishMessage(IS_MOVING_TOPIC, strMsg);
}

void MosquittoBroker::SendTiltInfo(const int info, const std::string datetime)
{
    std::string strInfo = std::to_string(info);
    std::string strMsg = "{\"datetime\":" + datetime + ",\"info\":" + strInfo + "}";

    PublishMessage(TILT_INFO_TOPIC, strMsg);
}

void MosquittoBroker::SendHeartbeat(const std::string datetime)
{
    std::string strMsg = "{\"datetime\":" + datetime + "}";

    PublishMessage(HEARTBEAT_TOPIC, strMsg);
}

void MosquittoBroker::SendSensorsState(sensor_state_t sensorState, const std::string datetime)
{
    StringBuffer strBuff;
    Writer<StringBuffer> writer(strBuff);
    writer.StartObject();

    writer.Key("notificationModule");
    writer.Bool(sensorState.notificationModuleValid);
    writer.Key("fixedAccelerometer");
    writer.Bool(sensorState.fixedAccelerometerValid);
    writer.Key("mobileAccelerometer");
    writer.Bool(sensorState.mobileAccelerometerValid);
    writer.Key("pressureMat");
    writer.Bool(sensorState.pressureMatValid);
    writer.Key("datetime");
    writer.String(datetime.c_str());

    writer.EndObject();

    PublishMessage(SENSORS_STATUS_TOPIC, strBuff.GetString());
}

bool MosquittoBroker::GetSetAlarmOn()
{
    _setAlarmOnNew = false;
    return _setAlarmOn;
}

tilt_settings_t MosquittoBroker::GetTiltSettings()
{
    _isTiltSettingsChanged = false;
    return _tiltSettings;
}

std::string MosquittoBroker::GetWifiInformation()
{
    _wifiChanged = false;
    return _wifiInformation;
}

notifications_settings_t MosquittoBroker::GetNotificationsSettings()
{
    _isNotificationsSettingsChanged = false;
    return _notificationsSettings;
}

bool MosquittoBroker::CalibPressureMatRequired()
{
    if (_calibPressureMatRequired)
    {
        _calibPressureMatRequired = false;
        return true;
    }
    else
    {
        return false;
    }
}

bool MosquittoBroker::CalibIMURequired()
{
    if (_calibIMURequired)
    {
        _calibIMURequired = false;
        return true;
    }
    else
    {
        return false;
    }
}

void MosquittoBroker::PublishMessage(const char *topic, const std::string message)
{
    if (message.empty())
    {
        printf("Error: Empty mosquitto message\n");
        return;
    }
    publish(NULL, topic, message.length(), message.c_str());
}
