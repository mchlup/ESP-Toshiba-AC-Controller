#include "ToshibaCarrierHvac.h"

#define BUADRATE 9600                       // buadrate
#define MAX_RX_BYTE_READ 250                // max bytes read
#define RX_READ_TIMEOUT 250                 // read timeout(ms)
#define IDLE_TIMEOUT 1                      // max timeout(minutes) after received data, try to query temperature to check the connection (should not exceed than 3 minutes)
#define CONNECTION_TIMEOUT 2                // max timeout(minutes) after sent some query or command but no reply in time, that's mean connection break or disconnected, try to send new handshake
#define START_DELAY 10                      // after connected delay x second before query all data
#define SETTINGS_SEND_DELAY 600             // delay x ms before send next setting (do not decrease too much, your hvac may not parse a setting correctly)
#define SINGLE_QUEUE_TIMEOUT 800            // when timeout(ms) reached and has only one callback in queue just do a callback
#define MULTI_QUEUE_TIMEOUT 1500            // when queue > 1, wait for other data until timeout(ms) then do a callback
#define MAX_FEEDBACK_COUNT 5                // when received x feedbacks then query temperature once to avoid front panel blinking, this value should not exceed 20.

extern HardwareSerial Serial;
// Optional RX hook – definováno ve vašem .ino
extern void HvacRawAnalyzer_onFrame(bool tx, uint8_t cmd, const uint8_t* payload, uint8_t plen);

#if defined(HVAC_USE_SW_SERIAL)
ToshibaCarrierHvac::ToshibaCarrierHvac(uint8_t rxPin, uint8_t txPin) {
    #if defined(__AVR__)
    CustomSoftwareSerial *port = new CustomSoftwareSerial(rxPin, txPin);
    port->begin(BUADRATE, CSERIAL_8E1);
    #endif
    #if defined(ESP8266)
    SoftwareSerial *port = new SoftwareSerial(rxPin, txPin);
    port->begin(BUADRATE, SWSERIAL_8E1);
    #endif
    this->_serial = port;
    this->_swSerial = true;
}
#endif

ToshibaCarrierHvac::ToshibaCarrierHvac(HardwareSerial* port) {
    #if defined(ESP8266) || defined(ESP32)
    port->setRxBufferSize(MAX_RX_BYTE_READ);
    #endif
    port->begin(BUADRATE, SERIAL_8E1);
    #if defined(ESP8266)
    // port->swap();
    #endif
    this->_serial = port;
    this->_swSerial = false;
}

ToshibaCarrierHvac::~ToshibaCarrierHvac() {
    if (_swSerial) delete this->_serial;
}

// callback
void ToshibaCarrierHvac::setStatusUpdatedCallback(STATUS_UPDATED_CALLBACK_SIGNATURE) {
    this->statusUpdatedCallback = statusUpdatedCallback;
}
void ToshibaCarrierHvac::setSettingsUpdatedCallback(SETTINGS_UPDATED_CALLBACK_SIGNATURE) {
    this->settingsUpdatedCallback = settingsUpdatedCallback;
}
void ToshibaCarrierHvac::setUpdateCallback(UPDATE_CALLBACK_SIGNATURE) {
    this->updateCallback = updateCallback;
}
void ToshibaCarrierHvac::setWhichFunctionUpdatedCallback(WHICH_FUNCTION_UPDATED_CALLBACK_SIGNATURE) {
    this->whichFunctionUpdatedCallback = whichFunctionUpdatedCallback;
}

// normal function
void ToshibaCarrierHvac::sendPacket(byte data[], size_t dataLen) {
    _serial->write(data, dataLen);
    _lastSendWake = millis();
    _sendWake = true;
    #ifdef HVAC_DEBUG
    DEBUG_PORT.print(F("HVAC> Sending data-> "));
    for (uint8_t i=0; i<dataLen; i++) {
        DEBUG_PORT.print(data[i]);
        DEBUG_PORT.print(" ");
    }
    DEBUG_PORT.println("");
    #endif
}

void ToshibaCarrierHvac::sendPacket(const byte data[], size_t dataLen) {
    _serial->write(data, dataLen);
    _lastSendWake = millis();
    _sendWake = true;
    #ifdef HVAC_DEBUG
    DEBUG_PORT.print(F("HVAC> Sending data-> "));
    for (uint8_t i=0; i<dataLen; i++) {
        DEBUG_PORT.print(data[i]);
        DEBUG_PORT.print(" ");
    }
    DEBUG_PORT.println("");
    #endif
}

byte ToshibaCarrierHvac::getByteByName(const byte byteMap[], const char* valMap[], size_t byteLen, const char* name) {
    yield();
    for (uint8_t i=0; i<byteLen; i++) {
        if(name && valMap[i] && strcasecmp(valMap[i], name) == 0) {
            return byteMap[i];
        }
    }
    return 255;
}

const char* ToshibaCarrierHvac::getNameByByte(const char* valMap[], const byte byteMap[], size_t byteLen, byte byteVal) {
    yield();
    for (uint8_t i=0; i<byteLen; i++) {
        if (byteMap[i] == byteVal) {
            return valMap[i];
        }
    }
    return valMap[byteLen];
}

byte ToshibaCarrierHvac::checksum(uint16_t baseKey, byte data[], size_t dataLen) {
    int16_t result=0;
    uint16_t key = baseKey - (dataLen * 2);
    result = key;
    for (int8_t i=0; i<dataLen; i++) {
        result -= data[i];
    }
    if (result > 255) return result - 256;
    else if (result < 0) return result + 256;
    else return result;
}

int8_t ToshibaCarrierHvac::temperatureCorrection(byte val) {
    if (val > 127) return ((256 - val) * (-1));
    else return val;
}

bool ToshibaCarrierHvac::createPacket(const byte header[], size_t headerLen, byte packetType, byte data[],byte dataLen) {
    if (packetType == getByteByName(PACKET_TYPE, PACKET_TYPE_MAP, sizeof(PACKET_TYPE), "COMMAND")) {    // type: command
        byte packet[12 + dataLen + 1];    // base + dataLen + checksum
        memset(packet, 0, sizeof(packet));  // set all index to 0
        memcpy(packet, header, headerLen);  // add header
        packet[3] = packetType; // add packet type
        packet[6] = sizeof(packet) - 8; // add packet size
        // add unknown byte
        packet[7] = 1;
        packet[8] = 48;
        packet[9] = 1;
        packet[11] = dataLen;  // add data type
        memcpy(packet + 12, data, dataLen);   // add data
        packet[sizeof(packet) - 1] = checksum(438, data, dataLen);  // add checksum

        #ifdef HVAC_DEBUG
        DEBUG_PORT.println(F("HVAC> Command/Query packet was created"));
        #endif

        // send created packet
        sendPacket(packet, sizeof(packet));
        return true;
    } else if (packetType == getByteByName(PACKET_TYPE, PACKET_TYPE_MAP, sizeof(PACKET_TYPE), "REPLY")) {     // type: reply
        byte packet[14 + dataLen + 1];    // base + dataLen + checksum
        memset(packet, 0, sizeof(packet));  // set all index to 0
        memcpy(packet, header, headerLen);  // add header
        packet[3] = packetType; // add packet type
        packet[6] = sizeof(packet) - 8; // add packet size
        // add unknown byte
        packet[7] = 1;
        packet[8] = 48;
        packet[9] = 1;
        packet[13] = dataLen;  // add data type
        memcpy(packet + 14, data, dataLen);   // add data
        packet[sizeof(packet) - 1] = checksum(308, data, dataLen);  // add checksum

        #ifdef HVAC_DEBUG
        DEBUG_PORT.println(F("HVAC> Reply packet was created"));
        #endif

        // send created packet
        sendPacket(packet, sizeof(packet));
        return true;
    }
    return false;
}

void ToshibaCarrierHvac::sendHandshake(void) {
    if (_firstRun && !_connected) {
        ++_handshakeAttempts;
        // Restore the six SYN telegrams and the original 200ms spacing.
        sendPacket(HANDSHAKE_SYN_PACKET_1, sizeof(HANDSHAKE_SYN_PACKET_1));
        delay(200);
        sendPacket(HANDSHAKE_SYN_PACKET_2, sizeof(HANDSHAKE_SYN_PACKET_2));
        delay(200);
        sendPacket(HANDSHAKE_SYN_PACKET_3, sizeof(HANDSHAKE_SYN_PACKET_3));
        delay(200);
        sendPacket(HANDSHAKE_SYN_PACKET_4, sizeof(HANDSHAKE_SYN_PACKET_4));
        delay(200);
        sendPacket(HANDSHAKE_SYN_PACKET_5, sizeof(HANDSHAKE_SYN_PACKET_5));
        delay(200);
        sendPacket(HANDSHAKE_SYN_PACKET_6, sizeof(HANDSHAKE_SYN_PACKET_6));
        delay(200);
        _sendWake = true;
        _lastSendWake = millis();
        #ifdef HVAC_DEBUG
        DEBUG_PORT.println(F("HVAC> Handshake sent; waiting for SYN/ACK"));
        #endif
    } else if (_handshake && !_connected && !_ready) {
        sendPacket(HANDSHAKE_ACK_PACKET_1, sizeof(HANDSHAKE_ACK_PACKET_1));
        delay(200);
        sendPacket(HANDSHAKE_ACK_PACKET_2, sizeof(HANDSHAKE_ACK_PACKET_2));
        delay(100);
        #ifdef HVAC_DEBUG
        DEBUG_PORT.println(F("HVAC> ACK sent; waiting for READY"));
        #endif
        _handshake = false;
        _ready = _sendWake = true;
        _lastSendWake = millis();
    }
}

bool ToshibaCarrierHvac::syncUserSettings(void) {
    if (strcasecmp(wantedSettings.state, userSettings.state) != 0) {    // state
        wantedSettings.state = userSettings.state;
        byte data[2];
        data[0] = getByteByName(FUNCTION_BYTE, FUNCTION_BYTE_MAP, sizeof(FUNCTION_BYTE), "STATE");
        data[1] = getByteByName(STATE_BYTE, OFF_ON_MAP, sizeof(STATE_BYTE), wantedSettings.state);
        createPacket(PACKET_HEADER, sizeof(PACKET_HEADER), 16, data, sizeof(data));
        #ifdef HVAC_DEBUG
        DEBUG_PORT.print(F("HVAC> User wanted State-> "));
        DEBUG_PORT.println(wantedSettings.state);
        #endif
        _lastSyncSettings = millis();
        return true;
    }
    if (wantedSettings.setpoint != userSettings.setpoint) {    // setpoint
        if ((userSettings.setpoint >= 17) && (userSettings.setpoint <= 30)) {
            wantedSettings.setpoint = userSettings.setpoint;
            byte data[2];
            data[0] = getByteByName(FUNCTION_BYTE, FUNCTION_BYTE_MAP, sizeof(FUNCTION_BYTE), "SETPOINT");
            data[1] = wantedSettings.setpoint;
            createPacket(PACKET_HEADER, sizeof(PACKET_HEADER), 16, data, sizeof(data));
            #ifdef HVAC_DEBUG
            DEBUG_PORT.print(F("HVAC> User wanted Setpoint-> "));
            DEBUG_PORT.println(wantedSettings.setpoint);
            #endif
            _lastSyncSettings = millis();
            return true;
        } else if (userSettings.setpoint < 17) {    // if value lower then minimun set to minimun
            #ifdef HVAC_DEBUG
            DEBUG_PORT.print(F("HVAC> error: User wanted out of range, set to minimum - User wanted Setpoint-> "));
            DEBUG_PORT.println(userSettings.setpoint);
            #endif
            userSettings.setpoint = 17;
            return false;
        } else if (userSettings.setpoint > 30) {    // if value higher then maximun set to maximum
            #ifdef HVAC_DEBUG
            DEBUG_PORT.print(F("HVAC> error: User wanted out of range, set to maximum - User wanted Setpoint-> "));
            DEBUG_PORT.println(userSettings.setpoint);
            #endif
            userSettings.setpoint = 30;
            return false;
        }
    }
    if (strcasecmp(wantedSettings.mode, userSettings.mode) != 0) {    // mode
        wantedSettings.mode = userSettings.mode;
        byte data[2];
        data[0] = getByteByName(FUNCTION_BYTE, FUNCTION_BYTE_MAP, sizeof(FUNCTION_BYTE), "MODE");
        data[1] = getByteByName(MODE_BYTE, MODE_BYTE_MAP, sizeof(MODE_BYTE), wantedSettings.mode);
        createPacket(PACKET_HEADER, sizeof(PACKET_HEADER), 16, data, sizeof(data));
        #ifdef HVAC_DEBUG
        DEBUG_PORT.print(F("HVAC> User wanted Mode-> "));
        DEBUG_PORT.println(wantedSettings.mode);
        #endif
        _lastSyncSettings = millis();
        return true;
    }
    if (strcasecmp(wantedSettings.swing, userSettings.swing) != 0) {    // swing
        wantedSettings.swing = userSettings.swing;
        byte data[2];
        data[0] = getByteByName(FUNCTION_BYTE, FUNCTION_BYTE_MAP, sizeof(FUNCTION_BYTE), "SWING");
        data[1] = getByteByName(SWING_BYTE, SWING_BYTE_MAP, sizeof(SWING_BYTE), wantedSettings.swing);
        createPacket(PACKET_HEADER, sizeof(PACKET_HEADER), 16, data, sizeof(data));
        #ifdef HVAC_DEBUG
        DEBUG_PORT.print(F("HVAC> User wanted Swing-> "));
        DEBUG_PORT.println(wantedSettings.swing);
        #endif
        _lastSyncSettings = millis();
        return true;
    }
    if (strcasecmp(wantedSettings.fanMode, userSettings.fanMode) != 0) {    // fan mode
        wantedSettings.fanMode = userSettings.fanMode;
        byte data[2];
        data[0] = getByteByName(FUNCTION_BYTE, FUNCTION_BYTE_MAP, sizeof(FUNCTION_BYTE), "FANMODE");
        data[1] = getByteByName(FANMODE_BYTE, FANMODE_BYTE_MAP, sizeof(FANMODE_BYTE), wantedSettings.fanMode);
        createPacket(PACKET_HEADER, sizeof(PACKET_HEADER), 16, data, sizeof(data));
        #ifdef HVAC_DEBUG
        DEBUG_PORT.print(F("HVAC> User wanted FanMode-> "));
        DEBUG_PORT.println(wantedSettings.fanMode);
        #endif
        _lastSyncSettings = millis();
        return true;
    }
    if (strcasecmp(wantedSettings.pure, userSettings.pure) != 0) {    // pure
        wantedSettings.pure = userSettings.pure;
        byte data[2];
        data[0] = getByteByName(FUNCTION_BYTE, FUNCTION_BYTE_MAP, sizeof(FUNCTION_BYTE), "PURE");
        data[1] = getByteByName(PURE_BYTE, OFF_ON_MAP, sizeof(PURE_BYTE), wantedSettings.pure);
        createPacket(PACKET_HEADER, sizeof(PACKET_HEADER), 16, data, sizeof(data));
        #ifdef HVAC_DEBUG
        DEBUG_PORT.print(F("HVAC> User wanted Pure-> "));
        DEBUG_PORT.println(wantedSettings.pure);
        #endif
        _lastSyncSettings = millis();
        return true;
    }
    if (strcasecmp(wantedSettings.powerSelect, userSettings.powerSelect) != 0) {    // power select
        wantedSettings.powerSelect = userSettings.powerSelect;
        byte data[2];
        data[0] = getByteByName(FUNCTION_BYTE, FUNCTION_BYTE_MAP, sizeof(FUNCTION_BYTE), "PSEL");
        data[1] = getByteByName(PSEL_BYTE, PSEL_BYTE_MAP, sizeof(PSEL_BYTE), wantedSettings.powerSelect);
        createPacket(PACKET_HEADER, sizeof(PACKET_HEADER), 16, data, sizeof(data));
        #ifdef HVAC_DEBUG
        DEBUG_PORT.print(F("HVAC> User wanted PowerSelect-> "));
        DEBUG_PORT.println(wantedSettings.powerSelect);
        #endif
        _lastSyncSettings = millis();
        return true;
    }
    if (strcasecmp(wantedSettings.operation, userSettings.operation) != 0) {    // operation
        wantedSettings.operation = userSettings.operation;
        byte data[2];
        data[0] = getByteByName(FUNCTION_BYTE, FUNCTION_BYTE_MAP, sizeof(FUNCTION_BYTE), "OP");
        data[1] = getByteByName(OP_BYTE, OP_BYTE_MAP, sizeof(OP_BYTE), wantedSettings.operation);
        createPacket(PACKET_HEADER, sizeof(PACKET_HEADER), 16, data, sizeof(data));
        #ifdef HVAC_DEBUG
        DEBUG_PORT.print(F("HVAC> User wanted Operation-> "));
        DEBUG_PORT.println(wantedSettings.operation);
        #endif
        _lastSyncSettings = millis();
        return true;
    }
    if (strcasecmp(wantedSettings.wifiLed, userSettings.wifiLed) != 0) {    // wifi led
        wantedSettings.wifiLed = userSettings.wifiLed;
        byte data[2];
        if (!_wifiled) {    // wifi led 1
            data[0] = getByteByName(FUNCTION_BYTE, FUNCTION_BYTE_MAP, sizeof(FUNCTION_BYTE), "WIFILED1");
            data[1] = getByteByName(WIFILED1_BYTE, OFF_ON_MAP, sizeof(WIFILED1_BYTE), wantedSettings.wifiLed);
        } else {    // wifi led 2
            data[0] = getByteByName(FUNCTION_BYTE, FUNCTION_BYTE_MAP, sizeof(FUNCTION_BYTE), "WIFILED2");
            data[1] = getByteByName(WIFILED2_BYTE, OFF_ON_MAP, sizeof(WIFILED2_BYTE), wantedSettings.wifiLed);
        }
        
        createPacket(PACKET_HEADER, sizeof(PACKET_HEADER), 16, data, sizeof(data));
        #ifdef HVAC_DEBUG
        DEBUG_PORT.print(F("HVAC> User wanted Wifi LED-> "));
        DEBUG_PORT.println(wantedSettings.wifiLed);
        #endif
        _lastSyncSettings = millis();
        return true;
    }
    return false;
}

bool ToshibaCarrierHvac::processData(byte data[], size_t dataLen) {
    if (dataLen == 5) {     // process data group 1 - basic (mode, setpoint, fanmode, operation)
        hvacSettings receiveSettings;
        if (data[0] == getByteByName(FUNCTION_BYTE, FUNCTION_BYTE_MAP, sizeof(FUNCTION_BYTE), "FN_GROUP_1")) {
            receiveSettings.mode = getNameByByte(MODE_BYTE_MAP, MODE_BYTE, sizeof(MODE_BYTE), data[1]);
            receiveSettings.setpoint = data[2];
            receiveSettings.fanMode = getNameByByte(FANMODE_BYTE_MAP, FANMODE_BYTE, sizeof(FANMODE_BYTE), data[3]);
            receiveSettings.operation = getNameByByte(OP_BYTE_MAP, OP_BYTE, sizeof(OP_BYTE), data[4]);
            if ((currentSettings.mode != receiveSettings.mode)) {
                if (settingsUpdatedCallback) {
                    wantedSettings.mode = userSettings.mode = currentSettings.mode = receiveSettings.mode;
                    _settingsCallbackBucket++;
                    _lastSettingsCallback = millis();
                } else if (updateCallback) {
                    wantedSettings.mode = userSettings.mode = currentSettings.mode = receiveSettings.mode;
                    _updateCallbackBucket++;
                    _lastUpdateCallback = millis();
                } else if (whichFunctionUpdatedCallback) {
                    wantedSettings.mode = userSettings.mode = currentSettings.mode = receiveSettings.mode;
                    whichFunctionUpdatedCallback(getNameByByte(FUNCTION_BYTE_MAP, FUNCTION_BYTE, sizeof(FUNCTION_BYTE), 176));
                } else {
                    wantedSettings.mode = userSettings.mode = currentSettings.mode = receiveSettings.mode;
                }
            }
            if (currentSettings.setpoint != receiveSettings.setpoint) {
                if (settingsUpdatedCallback) {
                    wantedSettings.setpoint = userSettings.setpoint = currentSettings.setpoint = receiveSettings.setpoint;
                    _settingsCallbackBucket++;
                    _lastSettingsCallback = millis();
                } else if (updateCallback) {
                    wantedSettings.setpoint = userSettings.setpoint = currentSettings.setpoint = receiveSettings.setpoint;
                    _updateCallbackBucket++;
                    _lastUpdateCallback = millis();
                } else if (whichFunctionUpdatedCallback) {
                    wantedSettings.setpoint = userSettings.setpoint = currentSettings.setpoint = receiveSettings.setpoint;
                    whichFunctionUpdatedCallback(getNameByByte(FUNCTION_BYTE_MAP, FUNCTION_BYTE, sizeof(FUNCTION_BYTE), 179));
                } else {
                    wantedSettings.setpoint = userSettings.setpoint = currentSettings.setpoint = receiveSettings.setpoint;
                }
            }
            if (currentSettings.fanMode != receiveSettings.fanMode) {
                if (settingsUpdatedCallback) {
                    wantedSettings.fanMode = userSettings.fanMode = currentSettings.fanMode = receiveSettings.fanMode;
                    _settingsCallbackBucket++;
                    _lastSettingsCallback = millis();
                } else if (updateCallback) {
                    wantedSettings.fanMode = userSettings.fanMode = currentSettings.fanMode = receiveSettings.fanMode;
                    _updateCallbackBucket++;
                    _lastUpdateCallback = millis();
                } else if (whichFunctionUpdatedCallback) {
                    wantedSettings.fanMode = userSettings.fanMode = currentSettings.fanMode = receiveSettings.fanMode;
                    whichFunctionUpdatedCallback(getNameByByte(FUNCTION_BYTE_MAP, FUNCTION_BYTE, sizeof(FUNCTION_BYTE), 160));
                } else {
                    wantedSettings.fanMode = userSettings.fanMode = currentSettings.fanMode = receiveSettings.fanMode;
                }
            }
            if (currentSettings.operation != receiveSettings.operation) {
                if (settingsUpdatedCallback) {
                    wantedSettings.operation = userSettings.operation = currentSettings.operation = receiveSettings.operation;
                    _settingsCallbackBucket++;
                    _lastSettingsCallback = millis();
                } else if (updateCallback) {
                    wantedSettings.operation = userSettings.operation = currentSettings.operation = receiveSettings.operation;
                    _updateCallbackBucket++;
                    _lastUpdateCallback = millis();
                } else if (whichFunctionUpdatedCallback) {
                    wantedSettings.operation = userSettings.operation = currentSettings.operation = receiveSettings.operation;
                    whichFunctionUpdatedCallback(getNameByByte(FUNCTION_BYTE_MAP, FUNCTION_BYTE, sizeof(FUNCTION_BYTE), 247));
                } else {
                    wantedSettings.operation = userSettings.operation = currentSettings.operation = receiveSettings.operation;
                }
            }
            #ifdef HVAC_DEBUG
            DEBUG_PORT.print(F("HVAC> Process data group 1 result: Mode-> "));
            DEBUG_PORT.print(currentSettings.mode);
            DEBUG_PORT.print(F(", Setpoint-> "));
            DEBUG_PORT.print(currentSettings.setpoint);
            DEBUG_PORT.print(F(", Fanmode-> "));
            DEBUG_PORT.print(currentSettings.fanMode);
            DEBUG_PORT.print(F(", Operation-> "));
            DEBUG_PORT.println(currentSettings.operation);
            #endif
            return true;
        } else {
            #ifdef HVAC_DEBUG
            DEBUG_PORT.println(F("HVAC> Received unknown data group."));
            #endif
            return false;
        }
    } else if (dataLen == 2) {    // process single data
        #ifdef HVAC_DEBUG
        DEBUG_PORT.print(F("HVAC> Process data result: "));
        #endif
        // connection status
        if (data[0] == getByteByName(FUNCTION_BYTE, FUNCTION_BYTE_MAP, sizeof(FUNCTION_BYTE), "STATUS")) {  // status
            #ifdef HVAC_DEBUG
            DEBUG_PORT.print(F("Status-> "));
            #endif
            if (data[1] == getByteByName(STATUS_BYTE, STATUS_BYTE_MAP, sizeof(STATUS_BYTE), "READY")) {
                #ifdef HVAC_DEBUG
                DEBUG_PORT.println(F("READY"));
                #endif
                if (!_connected) {
                    _connected = true;
                    _ready = _handshake = false;
                }
                return true;
            } else {
                #ifdef HVAC_DEBUG
                DEBUG_PORT.println(getNameByByte(STATUS_BYTE_MAP, STATUS_BYTE, sizeof(STATUS_BYTE), data[1]));
                #endif
                return false;
            }
        }
        // data
        if (_connected) {   // process data when connected
            if ((data[0] == 187) || (data[0] == 190) || (data[0] == 144) || (data[0] == 148)) {    // status
                hvacStatus receiveStatus;
                if (data[0] == getByteByName(FUNCTION_BYTE, FUNCTION_BYTE_MAP, sizeof(FUNCTION_BYTE), "ROOMTEMP")) {    // room temperature
                    #ifdef HVAC_DEBUG
                    DEBUG_PORT.print(F("RoomTemp-> "));
                    #endif
                    receiveStatus.roomTemperature = temperatureCorrection(data[1]);
                    #ifdef HVAC_DEBUG
                    DEBUG_PORT.println(receiveStatus.roomTemperature);
                    #endif
                    if (currentStatus.roomTemperature != receiveStatus.roomTemperature) {
                        if (statusUpdatedCallback) {
                            currentStatus.roomTemperature = receiveStatus.roomTemperature;
                            _statusCallbackBucket++;
                            _lastStatusCallback = millis();
                            return true;
                        } else if (updateCallback) {
                            currentStatus.roomTemperature = receiveStatus.roomTemperature;
                            _updateCallbackBucket++;
                            _lastUpdateCallback = millis();
                            return true;
                        } else if (whichFunctionUpdatedCallback) {
                            currentStatus.roomTemperature = receiveStatus.roomTemperature;
                            whichFunctionUpdatedCallback(getNameByByte(FUNCTION_BYTE_MAP, FUNCTION_BYTE, sizeof(FUNCTION_BYTE), data[0]));
                            return true;
                        } else {
                            currentStatus.roomTemperature = receiveStatus.roomTemperature;
                            return true;
                        }
                    }
                    return false;
                }
                if (data[0] == getByteByName(FUNCTION_BYTE, FUNCTION_BYTE_MAP, sizeof(FUNCTION_BYTE), "OUTSIDETEMP")) { // outside temperature and cdu state
                    #ifdef HVAC_DEBUG
                    DEBUG_PORT.print(F("OutsideTemp-> "));
                    #endif
                    receiveStatus.outsideTemperature = temperatureCorrection(data[1]);
                    #ifdef HVAC_DEBUG
                    DEBUG_PORT.println(receiveStatus.outsideTemperature);
                    #endif
                    if (receiveStatus.outsideTemperature == 127) {  // cdu not running, not update outside temperature and update cdu state
                        #ifdef HVAC_DEBUG
                        DEBUG_PORT.println(F("HVAC> Not update outside temperature, condensing unit not running"));
                        #endif
                        receiveStatus.running = false;
                        if (currentStatus.running != receiveStatus.running) {
                            if (statusUpdatedCallback) {
                                currentStatus.running = receiveStatus.running;
                                _statusCallbackBucket++;
                                _lastStatusCallback = millis();
                                return true;
                            } else if (updateCallback) {
                                currentStatus.running = receiveStatus.running;
                                _updateCallbackBucket++;
                                _lastUpdateCallback = millis();
                                return true;
                            } else if (whichFunctionUpdatedCallback) {
                                currentStatus.running = receiveStatus.running;
                                whichFunctionUpdatedCallback("CDU_STATE");
                                return true;
                            } else {
                                currentStatus.running = receiveStatus.running;
                                return true;
                            }
                        }
                        return false;
                    } else if (currentStatus.outsideTemperature != receiveStatus.outsideTemperature) {
                        currentStatus.running = true;
                        if (statusUpdatedCallback) {
                            currentStatus.outsideTemperature = receiveStatus.outsideTemperature;
                            _statusCallbackBucket++;
                            _lastStatusCallback = millis();
                            return true;
                        } else if (updateCallback) {
                            currentStatus.outsideTemperature = receiveStatus.outsideTemperature;
                            _updateCallbackBucket++;
                            _lastUpdateCallback = millis();
                            return true;
                        } else if (whichFunctionUpdatedCallback) {
                            currentStatus.outsideTemperature = receiveStatus.outsideTemperature;
                            whichFunctionUpdatedCallback(getNameByByte(FUNCTION_BYTE_MAP, FUNCTION_BYTE, sizeof(FUNCTION_BYTE), data[0]));
                            whichFunctionUpdatedCallback("CDU_STATE");
                        } else {
                            currentStatus.outsideTemperature = receiveStatus.outsideTemperature;
                            return true;
                        }
                    }
                    return false;
                }
                if (data[0] == getByteByName(FUNCTION_BYTE, FUNCTION_BYTE_MAP, sizeof(FUNCTION_BYTE), "OFFTIMER")) {   // off timer
                    #ifdef HVAC_DEBUG
                    DEBUG_PORT.print(F("OffTimer-> "));
                    #endif
                    receiveStatus.offTimer = getNameByByte(OFF_ON_MAP, TIMER_BYTE, sizeof(TIMER_BYTE), data[1]);
                    #ifdef HVAC_DEBUG
                    DEBUG_PORT.println(receiveStatus.offTimer);
                    #endif
                    if (currentStatus.offTimer != receiveStatus.offTimer) {
                        if (statusUpdatedCallback) {
                            currentStatus.offTimer = receiveStatus.offTimer;
                            _statusCallbackBucket++;
                            _lastStatusCallback = millis();
                            return true;
                        } else if (updateCallback) {
                            currentStatus.offTimer = receiveStatus.offTimer;
                            _updateCallbackBucket++;
                            _lastUpdateCallback = millis();
                            return true;
                        } else if (whichFunctionUpdatedCallback) {
                            currentStatus.offTimer = receiveStatus.offTimer;
                            whichFunctionUpdatedCallback(getNameByByte(FUNCTION_BYTE_MAP, FUNCTION_BYTE, sizeof(FUNCTION_BYTE), data[0]));
                            return true;
                        } else {
                            currentStatus.offTimer = receiveStatus.offTimer;
                            return true;
                        }
                    }
                    return false;
                }
                if (data[0] == getByteByName(FUNCTION_BYTE, FUNCTION_BYTE_MAP, sizeof(FUNCTION_BYTE), "ONTIMER")) {   // on timer
                    #ifdef HVAC_DEBUG
                    DEBUG_PORT.print(F("OnTimer-> "));
                    #endif
                    receiveStatus.onTimer = getNameByByte(OFF_ON_MAP, TIMER_BYTE, sizeof(TIMER_BYTE), data[1]);
                    #ifdef HVAC_DEBUG
                    DEBUG_PORT.println(receiveStatus.onTimer);
                    #endif
                    if (currentStatus.onTimer != receiveStatus.onTimer) {
                        if (statusUpdatedCallback) {
                            currentStatus.onTimer = receiveStatus.onTimer;
                            _statusCallbackBucket++;
                            _lastStatusCallback = millis();
                            return true;
                        } else if (updateCallback) {
                            currentStatus.onTimer = receiveStatus.onTimer;
                            _updateCallbackBucket++;
                            _lastUpdateCallback = millis();
                            return true;
                        } else if (whichFunctionUpdatedCallback) {
                            currentStatus.onTimer = receiveStatus.onTimer;
                            whichFunctionUpdatedCallback(getNameByByte(FUNCTION_BYTE_MAP, FUNCTION_BYTE, sizeof(FUNCTION_BYTE), data[0]));
                            return true;
                        } else {
                            currentStatus.onTimer = receiveStatus.onTimer;
                            return true;
                        }
                    }
                    return false;
                }
                return false;
            } else {    // setting
                hvacSettings receiveSettings;
                if (data[0] == getByteByName(FUNCTION_BYTE, FUNCTION_BYTE_MAP, sizeof(FUNCTION_BYTE), "STATE")) {   // state
                    #ifdef HVAC_DEBUG
                    DEBUG_PORT.print(F("State-> "));
                    #endif
                    receiveSettings.state = getNameByByte(OFF_ON_MAP, STATE_BYTE, sizeof(STATE_BYTE), data[1]);
                    #ifdef HVAC_DEBUG
                    DEBUG_PORT.println(receiveSettings.state);
                    #endif
                    if (currentSettings.state != receiveSettings.state) {
                        if (settingsUpdatedCallback) {
                            wantedSettings.state = userSettings.state = currentSettings.state = receiveSettings.state;
                            _settingsCallbackBucket++;
                            _lastSettingsCallback = millis();
                            return true;
                        } else if (updateCallback) {
                            wantedSettings.state = userSettings.state = currentSettings.state = receiveSettings.state;
                            _updateCallbackBucket++;
                            _lastUpdateCallback = millis();
                            return true;
                        } else if (whichFunctionUpdatedCallback) {
                            wantedSettings.state = userSettings.state = currentSettings.state = receiveSettings.state;
                            whichFunctionUpdatedCallback(getNameByByte(FUNCTION_BYTE_MAP, FUNCTION_BYTE, sizeof(FUNCTION_BYTE), data[0]));
                            return true;
                        } else {
                            wantedSettings.state = userSettings.state = currentSettings.state = receiveSettings.state;
                            return true;
                        }
                    }
                    return false;
                }
                if (data[0] == getByteByName(FUNCTION_BYTE, FUNCTION_BYTE_MAP, sizeof(FUNCTION_BYTE), "SETPOINT")) {    // setpoint
                    #ifdef HVAC_DEBUG
                    DEBUG_PORT.print(F("Setpoint-> "));
                    #endif
                    receiveSettings.setpoint = data[1];
                    #ifdef HVAC_DEBUG
                    DEBUG_PORT.println(receiveSettings.setpoint);
                    #endif
                    if (currentSettings.setpoint != receiveSettings.setpoint) {
                        if (settingsUpdatedCallback) {
                            wantedSettings.setpoint = userSettings.setpoint = currentSettings.setpoint = receiveSettings.setpoint;
                            _settingsCallbackBucket++;
                            _lastSettingsCallback = millis();
                            return true;
                        } else if (updateCallback) {
                            wantedSettings.setpoint = userSettings.setpoint = currentSettings.setpoint = receiveSettings.setpoint;
                            _updateCallbackBucket++;
                            _lastUpdateCallback = millis();
                            return true;
                        } else if (whichFunctionUpdatedCallback) {
                            wantedSettings.setpoint = userSettings.setpoint = currentSettings.setpoint = receiveSettings.setpoint;
                            whichFunctionUpdatedCallback(getNameByByte(FUNCTION_BYTE_MAP, FUNCTION_BYTE, sizeof(FUNCTION_BYTE), data[0]));
                            return true;
                        } else {
                            wantedSettings.setpoint = userSettings.setpoint = currentSettings.setpoint = receiveSettings.setpoint;
                            return true;
                        }
                    }
                    return false;
                }
                if (data[0] == getByteByName(FUNCTION_BYTE, FUNCTION_BYTE_MAP, sizeof(FUNCTION_BYTE), "MODE")) {    // mode
                    #ifdef HVAC_DEBUG
                    DEBUG_PORT.print(F("Mode-> "));
                    #endif
                    receiveSettings.mode = getNameByByte(MODE_BYTE_MAP, MODE_BYTE, sizeof(MODE_BYTE), data[1]);
                    #ifdef HVAC_DEBUG
                    DEBUG_PORT.println(receiveSettings.mode);
                    #endif
                    if (currentSettings.mode != receiveSettings.mode) {
                        if (settingsUpdatedCallback) {
                            wantedSettings.mode = userSettings.mode = currentSettings.mode = receiveSettings.mode;
                            _settingsCallbackBucket++;
                            _lastSettingsCallback = millis();
                            return true;
                        } else if (updateCallback) {
                            wantedSettings.mode = userSettings.mode = currentSettings.mode = receiveSettings.mode;
                            _updateCallbackBucket++;
                            _lastUpdateCallback = millis();
                            return true;
                        } else if (whichFunctionUpdatedCallback) {
                            wantedSettings.mode = userSettings.mode = currentSettings.mode = receiveSettings.mode;
                            whichFunctionUpdatedCallback(getNameByByte(FUNCTION_BYTE_MAP, FUNCTION_BYTE, sizeof(FUNCTION_BYTE), data[0]));
                            return true;
                        } else {
                            wantedSettings.mode = userSettings.mode = currentSettings.mode = receiveSettings.mode;
                            return true;
                        }
                    }
                    return false;
                }
                if (data[0] == getByteByName(FUNCTION_BYTE, FUNCTION_BYTE_MAP, sizeof(FUNCTION_BYTE), "SWING")) {   // swing
                    #ifdef HVAC_DEBUG
                    DEBUG_PORT.print(F("Swing-> "));
                    #endif
                    receiveSettings.swing = getNameByByte(SWING_BYTE_MAP, SWING_BYTE, sizeof(SWING_BYTE), data[1]);
                    #ifdef HVAC_DEBUG
                    DEBUG_PORT.println(receiveSettings.swing);
                    #endif
                    if (currentSettings.swing != receiveSettings.swing) {
                        if (settingsUpdatedCallback) {
                            wantedSettings.swing = userSettings.swing = currentSettings.swing = receiveSettings.swing;
                            _settingsCallbackBucket++;
                            _lastSettingsCallback = millis();
                            return true;
                        } else if (updateCallback) {
                            wantedSettings.swing = userSettings.swing = currentSettings.swing = receiveSettings.swing;
                            _updateCallbackBucket++;
                            _lastUpdateCallback = millis();
                            return true;
                        } else if (whichFunctionUpdatedCallback) {
                            wantedSettings.swing = userSettings.swing = currentSettings.swing = receiveSettings.swing;
                            whichFunctionUpdatedCallback(getNameByByte(FUNCTION_BYTE_MAP, FUNCTION_BYTE, sizeof(FUNCTION_BYTE), data[0]));
                            return true;
                        } else {
                            wantedSettings.swing = userSettings.swing = currentSettings.swing = receiveSettings.swing;
                            return true;
                        }
                    }
                    return false;
                }
                if (data[0] == getByteByName(FUNCTION_BYTE, FUNCTION_BYTE_MAP, sizeof(FUNCTION_BYTE), "FANMODE")) { // fan mode
                    #ifdef HVAC_DEBUG
                    DEBUG_PORT.print(F("Fanmode-> "));
                    #endif
                    receiveSettings.fanMode = getNameByByte(FANMODE_BYTE_MAP, FANMODE_BYTE, sizeof(FANMODE_BYTE), data[1]);
                    #ifdef HVAC_DEBUG
                    DEBUG_PORT.println(receiveSettings.fanMode);
                    #endif
                    if (currentSettings.fanMode != receiveSettings.fanMode) {
                        if (settingsUpdatedCallback) {
                            wantedSettings.fanMode = userSettings.fanMode = currentSettings.fanMode = receiveSettings.fanMode;
                            _settingsCallbackBucket++;
                            _lastSettingsCallback = millis();
                            return true;
                        } else if (updateCallback) {
                            wantedSettings.fanMode = userSettings.fanMode = currentSettings.fanMode = receiveSettings.fanMode;
                            _updateCallbackBucket++;
                            _lastUpdateCallback = millis();
                            return true;
                        } else if (whichFunctionUpdatedCallback) {
                            wantedSettings.fanMode = userSettings.fanMode = currentSettings.fanMode = receiveSettings.fanMode;
                            whichFunctionUpdatedCallback(getNameByByte(FUNCTION_BYTE_MAP, FUNCTION_BYTE, sizeof(FUNCTION_BYTE), data[0]));
                            return true;
                        } else {
                            wantedSettings.fanMode = userSettings.fanMode = currentSettings.fanMode = receiveSettings.fanMode;
                            return true;
                        }
                    }
                    return false;
                }
                if (data[0] == getByteByName(FUNCTION_BYTE, FUNCTION_BYTE_MAP, sizeof(FUNCTION_BYTE), "PURE")) {    // pure
                    #ifdef HVAC_DEBUG
                    DEBUG_PORT.print(F("Pure-> "));
                    #endif
                    receiveSettings.pure = getNameByByte(OFF_ON_MAP, PURE_BYTE, sizeof(PURE_BYTE), data[1]);
                    #ifdef HVAC_DEBUG
                    DEBUG_PORT.println(receiveSettings.pure);
                    #endif
                    if (currentSettings.pure != receiveSettings.pure) {
                        if (settingsUpdatedCallback) {
                            wantedSettings.pure = userSettings.pure = currentSettings.pure = receiveSettings.pure;
                            _settingsCallbackBucket++;
                            _lastSettingsCallback = millis();
                            return true;
                        } else if (updateCallback) {
                            wantedSettings.pure = userSettings.pure = currentSettings.pure = receiveSettings.pure;
                            _updateCallbackBucket++;
                            _lastUpdateCallback = millis();
                            return true;
                        } else if (whichFunctionUpdatedCallback) {
                            wantedSettings.pure = userSettings.pure = currentSettings.pure = receiveSettings.pure;
                            whichFunctionUpdatedCallback(getNameByByte(FUNCTION_BYTE_MAP, FUNCTION_BYTE, sizeof(FUNCTION_BYTE), data[0]));
                            return true;
                        } else {
                            wantedSettings.pure = userSettings.pure = currentSettings.pure = receiveSettings.pure;
                            return true;
                        }
                    }
                    return false;
                }
                if (data[0] == getByteByName(FUNCTION_BYTE, FUNCTION_BYTE_MAP, sizeof(FUNCTION_BYTE), "PSEL")) {    // power select
                    #ifdef HVAC_DEBUG
                    DEBUG_PORT.print(F("PowerSelect-> "));
                    #endif
                    receiveSettings.powerSelect = getNameByByte(PSEL_BYTE_MAP, PSEL_BYTE, sizeof(PSEL_BYTE), data[1]);
                    #ifdef HVAC_DEBUG
                    DEBUG_PORT.println(receiveSettings.powerSelect);
                    #endif
                    if (currentSettings.powerSelect != receiveSettings.powerSelect) {
                        if (settingsUpdatedCallback) {
                            wantedSettings.powerSelect = userSettings.powerSelect = currentSettings.powerSelect = receiveSettings.powerSelect;
                            _settingsCallbackBucket++;
                            _lastSettingsCallback = millis();
                            return true;
                        } else if (updateCallback) {
                            wantedSettings.powerSelect = userSettings.powerSelect = currentSettings.powerSelect = receiveSettings.powerSelect;
                            _updateCallbackBucket++;
                            _lastUpdateCallback = millis();
                            return true;
                        } else if (whichFunctionUpdatedCallback) {
                            wantedSettings.powerSelect = userSettings.powerSelect = currentSettings.powerSelect = receiveSettings.powerSelect;
                            whichFunctionUpdatedCallback(getNameByByte(FUNCTION_BYTE_MAP, FUNCTION_BYTE, sizeof(FUNCTION_BYTE), data[0]));
                            return true;
                        } else {
                            wantedSettings.powerSelect = userSettings.powerSelect = currentSettings.powerSelect = receiveSettings.powerSelect;
                            return true;
                        }
                    }
                    return false;
                }
                if (data[0] == getByteByName(FUNCTION_BYTE, FUNCTION_BYTE_MAP, sizeof(FUNCTION_BYTE), "OP")) {    // operation
                    #ifdef HVAC_DEBUG
                    DEBUG_PORT.print(F("Operation-> "));
                    #endif
                    receiveSettings.operation = getNameByByte(OP_BYTE_MAP, OP_BYTE, sizeof(OP_BYTE), data[1]);
                    #ifdef HVAC_DEBUG
                    DEBUG_PORT.println(receiveSettings.operation);
                    #endif
                    if (currentSettings.operation != receiveSettings.operation) {
                        if (settingsUpdatedCallback) {
                            wantedSettings.operation = userSettings.operation = currentSettings.operation = receiveSettings.operation;
                            _settingsCallbackBucket++;
                            _lastSettingsCallback = millis();
                            return true;
                        } else if (updateCallback) {
                            wantedSettings.operation = userSettings.operation = currentSettings.operation = receiveSettings.operation;
                            _updateCallbackBucket++;
                            _lastUpdateCallback = millis();
                            return true;
                        } else if (whichFunctionUpdatedCallback) {
                            wantedSettings.operation = userSettings.operation = currentSettings.operation = receiveSettings.operation;
                            whichFunctionUpdatedCallback(getNameByByte(FUNCTION_BYTE_MAP, FUNCTION_BYTE, sizeof(FUNCTION_BYTE), data[0]));
                            return true;
                        } else {
                            wantedSettings.operation = userSettings.operation = currentSettings.operation = receiveSettings.operation;
                            return true;
                        }
                    }
                    return false;
                }
                if (data[0] == getByteByName(FUNCTION_BYTE, FUNCTION_BYTE_MAP, sizeof(FUNCTION_BYTE), "WIFILED1")) {    // wifi led 1
                    _wifiled = false;
                    #ifdef HVAC_DEBUG
                    DEBUG_PORT.print(F("WifiLED1-> "));
                    #endif
                    receiveSettings.wifiLed = getNameByByte(OFF_ON_MAP, WIFILED1_BYTE, sizeof(WIFILED1_BYTE), data[1]);
                    #ifdef HVAC_DEBUG
                    DEBUG_PORT.println(receiveSettings.wifiLed);
                    #endif
                    if (currentSettings.wifiLed != receiveSettings.wifiLed) {
                        if (settingsUpdatedCallback) {
                            wantedSettings.wifiLed = userSettings.wifiLed = currentSettings.wifiLed = receiveSettings.wifiLed;
                            _settingsCallbackBucket++;
                            _lastSettingsCallback = millis();
                            return true;
                        } else if (updateCallback) {
                            wantedSettings.wifiLed = userSettings.wifiLed = currentSettings.wifiLed = receiveSettings.wifiLed;
                            _updateCallbackBucket++;
                            _lastUpdateCallback = millis();
                            return true;
                        } else if (whichFunctionUpdatedCallback) {
                            wantedSettings.wifiLed = userSettings.wifiLed = currentSettings.wifiLed = receiveSettings.wifiLed;
                            whichFunctionUpdatedCallback(getNameByByte(FUNCTION_BYTE_MAP, FUNCTION_BYTE, sizeof(FUNCTION_BYTE), data[0]));
                            return true;
                        } else {
                            wantedSettings.wifiLed = userSettings.wifiLed = currentSettings.wifiLed = receiveSettings.wifiLed;
                            return true;
                        }
                    }
                    return false;
                }
                if (data[0] == getByteByName(FUNCTION_BYTE, FUNCTION_BYTE_MAP, sizeof(FUNCTION_BYTE), "WIFILED2")) {    // wifi led 2
                    _wifiled = true;
                    #ifdef HVAC_DEBUG
                    DEBUG_PORT.print(F("WifiLED2-> "));
                    #endif
                    receiveSettings.wifiLed = getNameByByte(OFF_ON_MAP, WIFILED2_BYTE, sizeof(WIFILED2_BYTE), data[1]);
                    #ifdef HVAC_DEBUG
                    DEBUG_PORT.println(receiveSettings.wifiLed);
                    #endif
                    if (currentSettings.wifiLed != receiveSettings.wifiLed) {
                        if (settingsUpdatedCallback) {
                            wantedSettings.wifiLed = userSettings.wifiLed = currentSettings.wifiLed = receiveSettings.wifiLed;
                            _settingsCallbackBucket++;
                            _lastSettingsCallback = millis();
                            return true;
                        } else if (updateCallback) {
                            wantedSettings.wifiLed = userSettings.wifiLed = currentSettings.wifiLed = receiveSettings.wifiLed;
                            _updateCallbackBucket++;
                            _lastUpdateCallback = millis();
                            return true;
                        } else if (whichFunctionUpdatedCallback) {
                            wantedSettings.wifiLed = userSettings.wifiLed = currentSettings.wifiLed = receiveSettings.wifiLed;
                            whichFunctionUpdatedCallback(getNameByByte(FUNCTION_BYTE_MAP, FUNCTION_BYTE, sizeof(FUNCTION_BYTE), data[0]));
                            return true;
                        } else {
                            wantedSettings.wifiLed = userSettings.wifiLed = currentSettings.wifiLed = receiveSettings.wifiLed;
                            return true;
                        }
                    }
                    return false;
                }
                #ifdef HVAC_DEBUG
                DEBUG_PORT.println(F("error: Received unknown function, skipped"));
                #endif
                return false;
            }
        } else {    // received data when not connected
            #ifdef HVAC_DEBUG
            DEBUG_PORT.println(F("error: Received data when not connected, skipped"));
            #endif
            return false;
        }
    } else if (dataLen == 1) {    // setting changed reply
        #ifdef HVAC_DEBUG
        DEBUG_PORT.print(F("HVAC> Received setting changed reply-> "));
        #endif
        if (data[0] == getByteByName(FUNCTION_BYTE, FUNCTION_BYTE_MAP, sizeof(FUNCTION_BYTE), "STATE")) {   // state
            #ifdef HVAC_DEBUG
            DEBUG_PORT.println(F("STATE"));
            #endif
            return createPacket(PACKET_HEADER, sizeof(PACKET_HEADER), 16, data, 1);
        }
        if (data[0] == getByteByName(FUNCTION_BYTE, FUNCTION_BYTE_MAP, sizeof(FUNCTION_BYTE), "SETPOINT")) {    // setpoint
            #ifdef HVAC_DEBUG
            DEBUG_PORT.println(F("SETPOINT"));
            #endif
            return createPacket(PACKET_HEADER, sizeof(PACKET_HEADER), 16, data, 1);
        }
        if (data[0] == getByteByName(FUNCTION_BYTE, FUNCTION_BYTE_MAP, sizeof(FUNCTION_BYTE), "MODE")) {    // mode
            #ifdef HVAC_DEBUG
            DEBUG_PORT.println(F("MODE"));
            #endif
            return createPacket(PACKET_HEADER, sizeof(PACKET_HEADER), 16, data, 1);
        }
        if (data[0] == getByteByName(FUNCTION_BYTE, FUNCTION_BYTE_MAP, sizeof(FUNCTION_BYTE), "SWING")) {   // swing
            #ifdef HVAC_DEBUG
            DEBUG_PORT.println(F("SWING"));
            #endif
            return createPacket(PACKET_HEADER, sizeof(PACKET_HEADER), 16, data, 1);
        }
        if (data[0] == getByteByName(FUNCTION_BYTE, FUNCTION_BYTE_MAP, sizeof(FUNCTION_BYTE), "FANMODE")) { //fan mode
            #ifdef HVAC_DEBUG
            DEBUG_PORT.println(F("FANMODE"));
            #endif
            return createPacket(PACKET_HEADER, sizeof(PACKET_HEADER), 16, data, 1);
        }
        if (data[0] == getByteByName(FUNCTION_BYTE, FUNCTION_BYTE_MAP, sizeof(FUNCTION_BYTE), "PURE")) {    // pure
            #ifdef HVAC_DEBUG
            DEBUG_PORT.println(F("PURE"));
            #endif
            return createPacket(PACKET_HEADER, sizeof(PACKET_HEADER), 16, data, 1);
        }
        if (data[0] == getByteByName(FUNCTION_BYTE, FUNCTION_BYTE_MAP, sizeof(FUNCTION_BYTE), "PSEL")) {    // power select
            #ifdef HVAC_DEBUG
            DEBUG_PORT.println(F("PSEL"));
            #endif
            return createPacket(PACKET_HEADER, sizeof(PACKET_HEADER), 16, data, 1);
        }
        if (data[0] == getByteByName(FUNCTION_BYTE, FUNCTION_BYTE_MAP, sizeof(FUNCTION_BYTE), "OP")) {    // operation
            #ifdef HVAC_DEBUG
            DEBUG_PORT.println(F("OP"));
            #endif
            return createPacket(PACKET_HEADER, sizeof(PACKET_HEADER), 16, data, 1);
        }
        if (data[0] == getByteByName(FUNCTION_BYTE, FUNCTION_BYTE_MAP, sizeof(FUNCTION_BYTE), "WIFILED1") ||
            data[0] == getByteByName(FUNCTION_BYTE, FUNCTION_BYTE_MAP, sizeof(FUNCTION_BYTE), "WIFILED2")) {    // wifi led 1 and 2
            #ifdef HVAC_DEBUG
            DEBUG_PORT.println(F("WifiLED"));
            #endif
            return createPacket(PACKET_HEADER, sizeof(PACKET_HEADER), 16, data, 1);
        }
        return false;
    } else {    // received unknown data
        #ifdef HVAC_DEBUG
        DEBUG_PORT.println(F("HVAC> Received unknown data, ignored"));
        #endif
        return false;
    }
    return false;
}

bool ToshibaCarrierHvac::readPacket(byte data[], size_t dataLen) {
    if (dataLen < 8 || dataLen != static_cast<size_t>(data[6]) + 8) return false;
    if (data[3] == getByteByName(PACKET_TYPE, PACKET_TYPE_MAP, sizeof(PACKET_TYPE), "FEEDBACK")) {  // feedback
        if (dataLen < 13 || data[11] == 0 || data[11] > dataLen-13 || data[11] > 16) return false;
        byte newData[16];
        memcpy(newData, data + 12, data[11]);   // copy only data to newData
        #ifdef HVAC_DEBUG
        DEBUG_PORT.print(F("HVAC> Received feedback from hvac with data: "));
        for (uint8_t i=0; i<data[11]; i++) {
            DEBUG_PORT.print(newData[i]);
            DEBUG_PORT.print(" ");
        }
        DEBUG_PORT.println("");
        #endif
        byte reply_data[1] = {136};
        if (data[4] > MAX_FEEDBACK_COUNT) { // query temperature after received x feedback(s) to avoid front panel blinking.
            #ifdef HVAC_DEBUG
            DEBUG_PORT.println(F("HVAC> Max feedback count reached, sending temperature query."));
            #endif
            queryTemperature();
        }
        
        if (data[11] >= 1) {
            uint8_t cmd = newData[0];
            const uint8_t* payload = (data[11] > 1) ? (newData + 1) : nullptr;
            uint8_t plen = (data[11] > 1) ? (data[11] - 1) : 0;
            HvacRawAnalyzer_onFrame(false, cmd, payload, plen);
        }
        
        return processData(newData, data[11]);
    } else if (data[3] == getByteByName(PACKET_TYPE, PACKET_TYPE_MAP, sizeof(PACKET_TYPE), "REPLY")) {  // reply
        if (dataLen < 15 || data[13] == 0 || data[13] > dataLen-15 || data[13] > 16) return false;
        byte newData[16];
        memcpy(newData, data + 14, data[13]);   // copy only data to newData
        #ifdef HVAC_DEBUG
        DEBUG_PORT.print(F("HVAC> Received reply from hvac with data: "));
        for (uint8_t i=0; i<data[13]; i++) {
            DEBUG_PORT.print(newData[i]);
            DEBUG_PORT.print(" ");
        }
        DEBUG_PORT.println("");
        #endif
        
        // >>> RX hook pro analyzér (REPLY)
        if (data[13] >= 1) {
            uint8_t cmd = newData[0];
            const uint8_t* payload = (data[13] > 1) ? (newData + 1) : nullptr;
            uint8_t plen = (data[13] > 1) ? (data[13] - 1) : 0;
            HvacRawAnalyzer_onFrame(false, cmd, payload, plen);
        }
        
        return processData(newData, data[13]);
    } else if (data[3] == getByteByName(PACKET_TYPE, PACKET_TYPE_MAP, sizeof(PACKET_TYPE), "SYN/ACK")) {    // syn/ack
        #ifdef HVAC_DEBUG
        DEBUG_PORT.println(F("HVAC> Received handshake SYN/ACK"));
        #endif
        _handshake = true;
        return true;
    } else if (data[3] == getByteByName(PACKET_TYPE, PACKET_TYPE_MAP, sizeof(PACKET_TYPE), "ACK")) {    // ack
        #ifdef HVAC_DEBUG
        DEBUG_PORT.println(F("HVAC> Received confirm handshake, your code may crash or hw problem cause node mcu restarted"));
        #endif
        _ready = _handshake = false;
        _connected = true;
        return true;
    } else {   // unknown
        #ifdef HVAC_DEBUG
        DEBUG_PORT.print(F("HVAC> Received "));
        DEBUG_PORT.print(getNameByByte(PACKET_TYPE_MAP, PACKET_TYPE, sizeof(PACKET_TYPE_MAP), data[3]));
        DEBUG_PORT.println(F(" packet from hvac"));
        #endif
        return false;
    }
}

bool ToshibaCarrierHvac::findHeader(byte data[], size_t dataLen, const byte header[], size_t headerLen) {
    return data && header && headerLen && dataLen >= headerLen &&
           memcmp(data, header, headerLen) == 0;
}
uint8_t ToshibaCarrierHvac::findHeaderStart(byte data[], size_t dataLen, const byte header[], size_t headerLen) {
    if (!data || !header || !headerLen || dataLen < headerLen) return 255;
    for (size_t pos = 0; pos + headerLen <= dataLen; ++pos)
        if (memcmp(data + pos, header, headerLen) == 0) return static_cast<uint8_t>(pos);
    return 255;
}
bool ToshibaCarrierHvac::chunkSplit(byte data[], size_t dataLen) {
    if (!data || dataLen < 8) return false;
    size_t pos = 0;
    bool processed = false;
    while (pos + 8 <= dataLen) {
        // 02 00 00, 02 00 01, 02 00 02 and 02 00 03 are
        // known variants of the protocol header.
        if (data[pos] != 2 || data[pos+1] != 0 || data[pos+2] > 3) {
            ++pos;
            continue;
        }
        size_t len = static_cast<size_t>(data[pos+6]) + 8;
        if (len < 8 || len > MAX_RX_BYTE_READ) {
            ++pos;
            continue;
        }
        if (len > dataLen - pos) break;
        bool accepted = readPacket(data + pos, len);
        processed = accepted || processed;
        pos += len;
        yield();
    }
    return processed;
}

bool ToshibaCarrierHvac::packetMonitor(void) {
    if (!_serial || !_serial->available()) return false;
    byte rxBuffer[MAX_RX_BYTE_READ];
    size_t dataLen = 0;
    uint32_t start = millis();
    // The original Toshiba protocol collector waits up to 250ms for the
    // remaining bytes. This handles frames even when the main loop is busy.
    while ((millis() - start) < RX_READ_TIMEOUT && dataLen < sizeof(rxBuffer)) {
        if (_serial->available()) {
            int value = _serial->read();
            if (value >= 0) rxBuffer[dataLen++] = static_cast<byte>(value);
        } else {
            yield();
        }
    }
    if (!dataLen) return false;
    #ifdef HVAC_DEBUG
    DEBUG_PORT.print(F("HVAC> Received bytes: "));
    DEBUG_PORT.println(dataLen);
    #endif
    _lastReceive = millis();
    _sendWake = !_connected;
    bool simpleFrame = (dataLen == 15 || dataLen == 17)
        && findHeader(rxBuffer, dataLen, PACKET_HEADER, sizeof(PACKET_HEADER));
    bool accepted = simpleFrame ? readPacket(rxBuffer, dataLen) : chunkSplit(rxBuffer, dataLen);
    if (accepted) {
        ++_validPackets;
        _lastValidFrameMs = millis();
    } else ++_invalidPackets;
    return accepted;
}

void ToshibaCarrierHvac::queryall(void) {
    // Toshiba bus: one query at a time, then collect its complete reply.
    const byte fn[11] = {128,135,144,148,163,187,190,199,222,223,248};
    for (uint8_t i=0; i<11; ++i) {
        byte data[1] = {fn[i]};
        createPacket(PACKET_HEADER, sizeof(PACKET_HEADER), 16, data, 1);
        delay(200);
        packetMonitor();
        yield();
    }
}

void ToshibaCarrierHvac::queryTemperature(void) {
    const byte fn[2] = {187,190};
    for (uint8_t i=0; i<2; ++i) {
        byte data[1] = {fn[i]};
        createPacket(PACKET_HEADER, sizeof(PACKET_HEADER), 16, data, 1);
        delay(200);
        packetMonitor();
        yield();
    }
}

void ToshibaCarrierHvac::handleHvac(void) {
    if (!_connected) {
        sendHandshake();
    }

    if (_firstRun) {
        _idleTimeout = IDLE_TIMEOUT * 60 * 1000;
        _connectionTimeout = CONNECTION_TIMEOUT * 60 * 1000;
        _queryallDelay = START_DELAY * 1000;
        _firstRun = false;
    }

    // query all data once after connected
    if (((millis() - _lastReceive) >= _queryallDelay) && !_init && _connected) {
        queryall();
        _init = true;
    }

    packetMonitor(); // original time-bounded RX

    // idle timeout
    if (((millis() - _lastReceive) >= _idleTimeout) && !_sendWake && _init) {
        #ifdef HVAC_DEBUG
        DEBUG_PORT.println(F("HVAC> Max idle timeout reached"));
        #endif
        _sendWake = true;
        // try to query temperature
        queryTemperature();
        _lastSendWake = millis();
    }

    // connection timeout
    if (((millis() - _lastSendWake) >= _connectionTimeout) && _sendWake) {
        #ifdef HVAC_DEBUG
        DEBUG_PORT.println(F("HVAC> Connection timeout, try to send new handshake"));
        #endif
        _firstRun = true;
        _handshake = _ready = _connected = _sendWake = _init = false;
        _lastReceive = _lastSendWake = millis();
    }

    if(_init && ((millis() - _lastSyncSettings) >= SETTINGS_SEND_DELAY)) {
        if (syncUserSettings()) {
            #ifdef HVAC_DEBUG
            DEBUG_PORT.println(F("HVAC> New setting has been sync, waiting for reply"));
            #endif
        }
    }
    // settings callback limit
    if ((_settingsCallbackBucket == 1) && ((millis() - _lastSettingsCallback) >= SINGLE_QUEUE_TIMEOUT)) {
        settingsUpdatedCallback(currentSettings);
        _settingsCallbackBucket = 0;
    } else if ((_settingsCallbackBucket > 1) && ((millis() - _lastSettingsCallback) >= MULTI_QUEUE_TIMEOUT)) {
        settingsUpdatedCallback(currentSettings);
        _settingsCallbackBucket = 0;
    }
    // status callback limit
    if ((_statusCallbackBucket == 1) && ((millis() - _lastStatusCallback) >= SINGLE_QUEUE_TIMEOUT)) {
        statusUpdatedCallback(currentStatus);
        _statusCallbackBucket = 0;
    } else if ((_statusCallbackBucket > 1) && ((millis() - _lastStatusCallback) >= MULTI_QUEUE_TIMEOUT)) {
        statusUpdatedCallback(currentStatus);
        _statusCallbackBucket = 0;
    }
    // update callback limit
    if ((_updateCallbackBucket == 1) && ((millis() - _lastUpdateCallback) >= SINGLE_QUEUE_TIMEOUT)) {
        updateCallback();
        _updateCallbackBucket = 0;
    } else if ((_updateCallbackBucket > 1) && ((millis() - _lastUpdateCallback) >= MULTI_QUEUE_TIMEOUT)) {
        updateCallback();
        _updateCallbackBucket = 0;
    }
}

bool ToshibaCarrierHvac::sendCustomPacket(byte data[], size_t length) {
    if ((length >= 8) && (length <= 17)) {
        sendPacket(data, length);
        return true;
    } else {
        return false;
    }
}

void ToshibaCarrierHvac::applyPreset(hvacSettings newSettings) {
    userSettings = newSettings;
}

void ToshibaCarrierHvac::setState(const char* newState) {
    userSettings.state = newState;
}

void ToshibaCarrierHvac::setSetpoint(uint8_t newSetpoint) {
    userSettings.setpoint = newSetpoint;
}

void ToshibaCarrierHvac::setMode(const char* newMode) {
    userSettings.mode = newMode;
}

void ToshibaCarrierHvac::setSwing(const char* newSwing) {
    userSettings.swing = newSwing;
}

void ToshibaCarrierHvac::setFanMode(const char* newFanMode) {
    userSettings.fanMode = newFanMode;
}

void ToshibaCarrierHvac::setPure(const char* newPure) {
    userSettings.pure = newPure;
}

void ToshibaCarrierHvac::setPowerSelect(const char* newPowerSelect) {
    userSettings.powerSelect = newPowerSelect;
}

void ToshibaCarrierHvac::setOperation(const char* newOperation) {
    userSettings.operation = newOperation;
}

void ToshibaCarrierHvac::setWifiLed(const char* newWifiLed) {
    userSettings.wifiLed = newWifiLed;
}

hvacStatus ToshibaCarrierHvac::getStatus(void) {
    return currentStatus;
}

hvacSettings ToshibaCarrierHvac::getSettings(void) {
    return currentSettings;
}

int8_t ToshibaCarrierHvac::getRoomTemperature(void) {
    return currentStatus.roomTemperature;
}

int8_t ToshibaCarrierHvac::getOutsideTemperature(void) {
    return currentStatus.outsideTemperature;
}

const char* ToshibaCarrierHvac::getState(void) {
    return currentSettings.state;
}

uint8_t ToshibaCarrierHvac::getSetpoint(void) {
    return currentSettings.setpoint;
}

const char* ToshibaCarrierHvac::getMode(void) {
    return currentSettings.mode;
}

const char* ToshibaCarrierHvac::getSwing(void) {
    return currentSettings.swing;
}

const char* ToshibaCarrierHvac::getFanMode(void) {
    return currentSettings.fanMode;
}

const char* ToshibaCarrierHvac::getPure(void) {
    return currentSettings.pure;
}

const char* ToshibaCarrierHvac::getOffTimer(void) {
    return currentStatus.offTimer;
}

const char* ToshibaCarrierHvac::getOnTimer(void) {
    return currentStatus.onTimer;
}

const char* ToshibaCarrierHvac::getPowerSelect(void) {
    return currentSettings.powerSelect;
}

const char* ToshibaCarrierHvac::getWifiLed(void) {
    return currentSettings.wifiLed;
}

const char* ToshibaCarrierHvac::getOperation(void) {
    return currentSettings.operation;
}

bool ToshibaCarrierHvac::isCduRunning(void) {
    return currentStatus.running;
}

bool ToshibaCarrierHvac::isConnected(void) {
    return _connected;
}

void ToshibaCarrierHvac::forceQueryAllData(void) {
    _init = false;
}

const char* ToshibaCarrierHvac::getSwingRaw() const {
    return currentSettings.swing;
}

uint8_t ToshibaCarrierHvac::getSwingFixPosition() const {
    if (!currentSettings.swing) return 0;
    // oèekávané tvary: "fix_pos_1" až "fix_pos_5"
    if (strncasecmp(currentSettings.swing, "fix_pos_", 8) == 0) {
        int n = atoi(currentSettings.swing + 8);
        if (n >= 1 && n <= 5) return (uint8_t)n;
    }
    return 0; // 0 = není fixní poloha
}

bool ToshibaCarrierHvac::isSwingVertical() const {
    return currentSettings.swing && strcasecmp(currentSettings.swing, "v_swing") == 0;
}

bool ToshibaCarrierHvac::isSwingHorizontal() const {
    return currentSettings.swing && strcasecmp(currentSettings.swing, "h_swing") == 0;
}

bool ToshibaCarrierHvac::isSwingBoth() const {
    return currentSettings.swing && strcasecmp(currentSettings.swing, "hv_swing") == 0;
}
