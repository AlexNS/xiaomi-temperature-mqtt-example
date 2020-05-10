#include <HardwareSerial.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <string>

#include <WiFi.h>
#include <PubSubClient.h>

const char* WifiSsid = "*** SID ***";
const char* WifiPassword = "*** PASSWORD ***";
const char* MqttServer = "*** MQTT BROKER ADDRESS ***";
const char* MqttClientName = "esp32-ble-bridge";

enum EventType {
    Temperature = 0x01,
    Humidity = 0x02,
    Battery = 0x03,
    VisibleDevices = 0x04,
    SensorDevices = 0x05
};

enum XiaomiEventType {
    XiaomiTemperatureAndHumidity = 0x0D,
    XiaomiBattery = 0x0A,
    XiaomiHumidity = 0x06,
    XiaomiTemperature = 0x04
};

enum WifiActivityState {
    WifiConnecting = 1,
    WifiConnected = 2,
    MqttReconnect = 3,
    MqttConnecting = 4,
    MqttConnected = 5,
    WifiReconnect = 6,
    Error = 77,
};

class Event;

int g_scanTime = 30; // BLE scan time in seconds
QueueHandle_t g_eventsBufferMutex; // synchronization object for BLE events buffer
std::vector<Event*> g_eventsBuffer; // event buffer for received BLE events 

WiFiClient g_wifiClient; // wifi client object 
PubSubClient g_pubsubClient(g_wifiClient); // MQTT server client object
std::vector<Event*> g_eventsSendBuffer; // event send buffer for MQTT sending task
int g_eventsDeliverInterval = 30; // interval in seconds of MQTT events delivery

class Event {
    public:
        Event(std::string deviceAddress, EventType type, float value) {
            _deviceAddress = deviceAddress;
            _eventType = type;
            _value = value;
        };

        const std::string getDeviceAddress() {
            return _deviceAddress;
        }

        const EventType getEventType() {
            return _eventType;
        }

        const float getValue() {
            return _value;
        }
    private:
        std::string _deviceAddress;
        EventType _eventType;
        float _value;        
};



void printBuffer(uint8_t* buf, int len) {
    for(int i = 0; i < len; i++) {
        Serial.printf("%02x", buf[i]);
    }
    Serial.printf("  Len: %d", len);
    Serial.print("\n");
}

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {

    uint8_t* findServiceData(uint8_t* data, size_t length, uint8_t* foundBlockLength) {
        uint8_t* rightBorder = data + length;
        while (data < rightBorder) {
            uint8_t blockLength = *data;
            if (blockLength < 5) {
                data += (blockLength+1);
                continue;
            }
            uint8_t blockType = *(data+1);
            uint16_t serviceType = *(uint16_t*)(data + 2);
            if (blockType == 0x16 && serviceType == 0xfe95 && *(data + 4) == 0x50) {
                *foundBlockLength = blockLength-3;
                return data+4;
            }
            data += (blockLength+1);
        }   
        return nullptr;
    }

    void onResult(BLEAdvertisedDevice advertisedDevice) {

        uint8_t* payload = advertisedDevice.getPayload();
        size_t payloadLength = advertisedDevice.getPayloadLength();
        std::string deviceAddress = advertisedDevice.getAddress().toString();
        // Serial.printf("%s (%s): ", deviceAddress.c_str(), advertisedDevice.haveName() ? advertisedDevice.getName().c_str() : "");
        // printBuffer(payload, payloadLength);
        uint8_t serviceDataLength=0;
        uint8_t* serviceData = findServiceData(payload, payloadLength, &serviceDataLength);
        if (serviceData == nullptr) {
            return;
        }
        xSemaphoreTake(g_eventsBufferMutex, portMAX_DELAY);
        // 11th byte is an event type
        switch (serviceData[11])
        {
            case XiaomiEventType::XiaomiTemperatureAndHumidity:
            {
                float temp = *(uint16_t*)(serviceData + 11 + 3) / 10.0;
                float humidity = *(uint16_t*)(serviceData + 11 + 5) / 10.0;
                Serial.printf("Temp: %f Humidity: %f\n", temp, humidity);
                g_eventsBuffer.push_back(new Event(deviceAddress, EventType::Temperature, temp));
                g_eventsBuffer.push_back(new Event(deviceAddress, EventType::Humidity, humidity));
            }
            break;
            case XiaomiEventType::XiaomiTemperature:
            {
                float temp = *(uint16_t*)(serviceData + 11 + 3) / 10.0;
                Serial.printf("Temp: %f\n", temp);
                g_eventsBuffer.push_back(new Event(deviceAddress, EventType::Temperature, temp));
            }
            break;
            case XiaomiEventType::XiaomiHumidity:
            {
                float humidity = *(uint16_t*)(serviceData + 11 + 3) / 10.0;
                Serial.printf("Humidity: %f\n", humidity);
                g_eventsBuffer.push_back(new Event(deviceAddress, EventType::Humidity, humidity));
            }
            break;
            case XiaomiEventType::XiaomiBattery:
            {
                int battery = *(serviceData + 11 + 3);
                Serial.printf("Battery: %d\n", battery);
                g_eventsBuffer.push_back(new Event(deviceAddress, EventType::Battery, (float)battery));
            }
            break;
        }
        xSemaphoreGive(g_eventsBufferMutex);
    }
};

void setup() {
    Serial.begin(115200);
    g_eventsBufferMutex = xSemaphoreCreateMutex();

    BLEDevice::init("ble-mqtt-bridge");
    BLEScan * pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks(), true);
    pBLEScan->setActiveScan(false);
    pBLEScan->setInterval(0x50);
    pBLEScan->setWindow(0x30);

    TaskHandle_t bleScanHandle = nullptr;
    xTaskCreate(taskScanBleDevices, "ble-scan", 2048, nullptr, tskIDLE_PRIORITY, &bleScanHandle);

    TaskHandle_t wifiActivityHandle = nullptr;
    xTaskCreate(taskWifiActivity, "wifi-activity", 2048*8, nullptr, tskIDLE_PRIORITY, &wifiActivityHandle);
}

void loop() {
}


void taskScanBleDevices(void* pvParameters) {
    while (true) {
        BLEScan * pBLEScan = BLEDevice::getScan();
        BLEScanResults foundDevices = pBLEScan->start(g_scanTime, false);
        Serial.print("Devices found: ");
        int visibleDevices = foundDevices.getCount();
        Serial.println(visibleDevices);
        g_eventsBuffer.push_back(new Event("", EventType::VisibleDevices, (float)visibleDevices));
        Serial.println("Scan done!");
        vTaskDelay(2000 / portTICK_RATE_MS);
    }
}

void prepareSendBuffer() {
    Serial.println("Send buffer prepare started");
    xSemaphoreTake(g_eventsBufferMutex, portMAX_DELAY);
    Serial.println("Semaphore taken");
    if (!g_eventsBuffer.empty()) {
        Serial.println("Found events");
        for(std::vector<Event*>::reverse_iterator i = g_eventsBuffer.rbegin(); i != g_eventsBuffer.rend(); ++i) {
            Event* e = *i;
            std::string address = e->getDeviceAddress();
            Serial.printf("Trying to add event for address %s\n", address.c_str());

            // we should check if we already added that event type for that deviceAddress
            bool found = false;
            if (!g_eventsSendBuffer.empty()) {
                
                for(std::vector<Event*>::iterator i = g_eventsSendBuffer.begin(); i != g_eventsSendBuffer.end(); ++i) { 
                    if ((*i)->getDeviceAddress() == address && (*i)->getEventType() == e->getEventType()) {
                        found = true;
                        break;
                    }
                }
            }
            if (!found) {
                g_eventsSendBuffer.push_back(e);
                Serial.println("Event added");
            } else {
                delete e; // we don't need this event anymore
            }
        }
    }
    g_eventsBuffer.clear();
    xSemaphoreGive(g_eventsBufferMutex);
    Serial.println("Send buffer prepared");
}

void publishEvents() {
    Serial.println("Publish events started");
    const int bufferSize = 1000;
    char* topicStringBuffer = new char[bufferSize];
    char* payloadStringBuffer = new char[bufferSize];

    if (!g_eventsSendBuffer.empty()) {
        for(std::vector<Event*>::iterator i = g_eventsSendBuffer.begin(); i != g_eventsSendBuffer.end(); ++i) { 
            Event* e = *i;
            std::string address = e->getDeviceAddress();
            Serial.printf("Publishing event for %s\n", address.c_str());
            switch (e->getEventType())
            {
                case EventType::Temperature:
                    snprintf(topicStringBuffer, bufferSize, "sensor/%s/temperature", address.c_str());
                    break;
                case EventType::Humidity:
                    snprintf(topicStringBuffer, bufferSize, "sensor/%s/humidity", address.c_str());
                    break;
                case EventType::Battery:
                    snprintf(topicStringBuffer, bufferSize, "sensor/%s/battery", address.c_str());
                    break;
                case EventType::VisibleDevices:
                    snprintf(topicStringBuffer, bufferSize, "sensor/devices");
                    break;
                case EventType::SensorDevices:
                    snprintf(topicStringBuffer, bufferSize, "sensor/sensors");
                    break;
                default:
                    continue;
                    break;
            }
            snprintf(payloadStringBuffer, bufferSize, "%f", e->getValue());
            Serial.printf("Event: %s %s\n", topicStringBuffer, payloadStringBuffer);

            delete e;
            g_pubsubClient.publish(topicStringBuffer, payloadStringBuffer);
        }
    }
    Serial.println("Publish events DONE");
    g_eventsSendBuffer.clear();

    delete[] topicStringBuffer;
    delete[] payloadStringBuffer;
}

void taskWifiActivity(void* pvParameters) {
    Serial.println("Wifi Task Started");
    WifiActivityState state = WifiActivityState::WifiReconnect;
    bool shouldStop = false;
    while (!shouldStop)
    {
        wl_status_t wifiStatus = WiFi.status();
        Serial.printf("Wifi status: %d State: %d\n", wifiStatus, state);
        switch (state)
        {
            case WifiActivityState::WifiReconnect:
                vTaskDelay(5000 / portTICK_RATE_MS); // reconnect delay, just in case
                WiFi.begin(WifiSsid, WifiPassword);
                state = WifiActivityState::WifiConnecting;
                Serial.println("Connecting...");
            break;
            
            case WifiActivityState::WifiConnecting:
                switch (wifiStatus)
                {
                    case WL_CONNECTED:
                        state = WifiActivityState::WifiConnected;
                        Serial.println("Wifi Connected");
                        break;
                    case WL_CONNECT_FAILED:
                        state = WifiActivityState::Error;
                        break;
                    default:
                        vTaskDelay(1000 / portTICK_RATE_MS); // active waiting delay
                        break;
                }
                break;

            case WifiActivityState::WifiConnected:
                if (wifiStatus == WL_CONNECTED) {
                    Serial.println(WiFi.localIP());
                    state = WifiActivityState::MqttReconnect;
                } else {
                    state = WifiActivityState::WifiReconnect;
                }
                break;

            case WifiActivityState::MqttReconnect:
                if (wifiStatus == WL_CONNECTED) {
                    Serial.println("Mqtt server connecting");
                    g_pubsubClient.setServer(MqttServer, 1883);
                    g_pubsubClient.connect(MqttClientName);
                    state = WifiActivityState::MqttConnecting;
                } else {
                    state = WifiActivityState::WifiReconnect;
                }
                break;

            case WifiActivityState::MqttConnecting:
                if (wifiStatus == WL_CONNECTED) {
                    if (g_pubsubClient.connected()) {
                        Serial.println("Mqtt server connected");
                        state = WifiActivityState::MqttConnected;
                    }
                    vTaskDelay(1000 / portTICK_RATE_MS); // active waiting delay
                } else {
                    state = WifiActivityState::WifiReconnect;
                }
                break;

            case WifiActivityState::MqttConnected:
                if (wifiStatus == WL_CONNECTED) {
                    Serial.println("...Activity...");
                    if (g_pubsubClient.connected()) {
                        prepareSendBuffer();
                        publishEvents();
                        vTaskDelay( g_eventsDeliverInterval * 1000 / portTICK_RATE_MS);
                    } else {
                        Serial.println("Client Disconnected");
                        state = WifiActivityState::MqttReconnect;
                        vTaskDelay(5000/portTICK_RATE_MS);
                    }
                } else {
                    state = WifiActivityState::WifiReconnect;
                }
                break;

            case WifiActivityState::Error:
                Serial.println("Connection error");
                shouldStop = true; // end task
                // TODO add code for connection retry with increasing time intervals
                break;
            default:
                break;
        }
    }
    vTaskDelete(NULL);
}
