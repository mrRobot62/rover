#include <Arduino.h>
#include <ArduinoJson.h>
#include <Dynamixel2Arduino.h>
#include "rover.h"
#include "rover_utils.h"
#include "led_info.h"

#include <Arduino.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include "i2c_helper.h"

//--------------------------------------------------------
// I2C & Dynamixe-Konfigurationen in rover.h
//--------------------------------------------------------

#define RAINER_ROVER


// -----------------------------
// Command-IDs per Enum
// -----------------------------
enum CommandID : uint8_t
{
    CMD_NONE = 0,
    CMD_SERVO_WRITE,
    CMD_SERVO_READ,
    CMD_DIGITAL_WRITE,
    CMD_DIGITAL_READ,
    CMD_ANALOG_WRITE,
    CMD_ANALOG_READ,
    CMD_ESP32_STATE,
    CMD_MAX // nicht entfernen, muss das letzte Element sein, steht für die Größe des Arrays
};

enum SubCommandID : uint8_t
{
    SCMD_NONE = 0,
    SCMD_SERVO_SPEED,
    SCMD_SERVO_POSITION,
    SCMD_SERVO_SPEED_POSITION,
    SCMD_SERVO_TORQUE_ENABLE,
    SCMD_SERVO_LED_ON,
    SUB_MAX
};

enum DIGITALPIN : uint8_t
{
    ONBOARD_LED = 2,
    LED1 = 19,
    LED2 = 18,
    IO1 = 5,
    IO2 = 4,
    IO3 = 15,
    IO4 = 23,
    ADC0 = 36,
    ADC1 = 39,
    ADC2 = 34,
    ADC3 = 35 
};


// Callback-Funktionstyp
// typedef void (*CommandCallback)();
// Callback-Typen
typedef void (*WriteCallback)(I2C_CommandPacket &payload);
typedef void (*ReadCallback)(I2C_CommandPacket &payload);
// Callback-Arrays
WriteCallback writeCallbacks[CMD_MAX];
ReadCallback readCallbacks[CMD_MAX];

I2C_CommandPacket rxPacket;



bool isValidPin(uint8_t pin) {
    switch(pin) {
        case ONBOARD_LED:
        case LED1:
        case LED2:
        case IO1:
        case IO2:
        case IO3:
        case IO4:
        case ADC0:
        case ADC1:
        case ADC2:
        case ADC3:
            return true;
        default:
            return false;
    }
}

uint8_t on_off(uint8_t state) {
    return state > 0 ? 1 : 0;
}
// -----------------------------
// CRC8 Dallas/Maxim
// -----------------------------
uint8_t calcCRC8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++)
        {
            crc = (crc & 0x80) ? (crc << 1) ^ 0x31 : (crc << 1);
        }
    }
    return crc;
}

// -----------------------------
// WRITE callback Funktionen
// -----------------------------
/**
 *
 */
void cmdServoWrite(I2C_CommandPacket &payload)
{
    //
    // der Host sendet negative Werte mit Modulo 65536 als uint16_t integer zahl
    // um den negativen Wert wieder zu bekommen wird der Wert in int16_t gecastet und 
    // anschließend in ein float konvertiert.
    float speed = float(((int16_t)payload.data[0]) / 100.0);
    float curve = float(((int16_t)payload.data[1]) / 100.0);
    uint8_t scmd = payload.subcmd;
    DEBUG_SERIAL.printf("[cmdServoWrite] SubCMD: %d, Speed: %3.2f; Curve: %3.2f \n", scmd, speed, curve);

    //
    // Der Host kann die Befehle für Speed und Curve einzeln senden oder gemeinsam.
    // Über das SubCommand wird festgelegt, welche Art er sendet.
    switch (scmd) {
        case SCMD_SERVO_POSITION:
        setServoCurvature(&dxl, curve);
        break;
        case SCMD_SERVO_SPEED:
        setRoverVelocity(&dxl, speed);
        break;
        case SCMD_SERVO_SPEED_POSITION:
        setRoverVelocity(&dxl, speed);
        setServoCurvature(&dxl, curve);
        break;
        default:
        DEBUG_SERIAL.printf("[cmdServoWrite] => unbekannter SubCommand (%d) \n",scmd);
       
    }
}

/**
 *
 */
void cmdDigitalWrite(I2C_CommandPacket &payload)
{
    uint8_t pins[2] = {(uint8_t)payload.data[0], (uint8_t)payload.data[1]};
    uint8_t states[2] = {(uint8_t)payload.data[2], (uint8_t)payload.data[3]};

    if (isValidPin(pins[0])) {
        pinMode(pins[0],OUTPUT);
        digitalWrite(pins[0], on_off(states[0]));
    }
    if (isValidPin(pins[1])) {
        pinMode(pins[1],OUTPUT);
        digitalWrite(pins[1], on_off(states[1]));
    }
    DEBUG_SERIAL.printf("[cmdDigitalWrite] PINS: [%d,%d], STATES: [%d,%d]\n", pins[0], pins[1], states[0],states[1] );
}

/**
 *
 */
void cmdAnalogWrite(I2C_CommandPacket &payload)
{
    DEBUG_SERIAL.printf("[cmdAnalogWrite] => not implemented yet \n");

    DEBUG_SERIAL.printf("[cmdAnalogWrite]  \n");
}

//-------------------------------------------------------------
// Read callbacks
//-------------------------------------------------------------

/**
 *
 */
void cmdServoRead(I2C_CommandPacket &payload)
{
    DEBUG_SERIAL.printf("[cmdServoRead] => not implemented yet \n");

    DEBUG_SERIAL.printf("[cmdServoRead]  \n");
}

/**
 *
 */
void cmdESP32State(I2C_CommandPacket &data)
{
    DEBUG_SERIAL.printf("[cmdESP32State] => not implemented yet \n");
    DEBUG_SERIAL.printf("[cmdESP32State]  \n");
}

/**
 *
 */
void cmdDigitalRead(I2C_CommandPacket &payload)
{
    DEBUG_SERIAL.printf("[cmdDigitalRead] => not implemented yet \n");

    DEBUG_SERIAL.printf("[cmdDigitalRead]  \n");
}

/**
 *
 */
void cmdAnalogRead(I2C_CommandPacket &payload)
{
    DEBUG_SERIAL.printf("[cmdAnalogRead] => not implemented yet \n");

    DEBUG_SERIAL.printf("[cmdAnalogRead]  \n");
}

//-------------------------------------------------------------
// I2C Receive Event
//-------------------------------------------------------------
void receiveEvent(int len)
{
    //DEBUG_SERIAL.printf("receiveEvent - DataSize: %d\n", len);

    if (len != sizeof(I2C_CommandPacket))
    {
        DEBUG_SERIAL.printf("[I2C] rxBuffer ungültige Länge (%d) statt (%d)\n", len, sizeof(I2C_CommandPacket));
        return;
    }
    Wire.readBytes((uint8_t *)&rxPacket, sizeof(rxPacket));
    if (rxPacket.header != 0xFEEF)
    {
        DEBUG_SERIAL.printf("[I2C] rxBuffer ungülter Header (%d)\n", rxPacket.header);
        return;
    }

    uint8_t crcCalc = calcCRC8((uint8_t *)&rxPacket, sizeof(rxPacket) - 1);
    if (crcCalc != rxPacket.crc)
    {
        DEBUG_SERIAL.printf("[I2C] rxBuffer CRC Error (%d) statt (%d)\n", crcCalc, rxPacket.crc);
        return;
    }

    // Packet korrekt empfangen
    // DEBUG_SERIAL.printf("[I2C] rxBuffer-Struktur gültig\n");
    //
    // Auswertung des Commands und Aufruf der dazugehörigen Funktion über die CallBack-Funktion
    switch (rxPacket.cmd)
    {
    case CMD_SERVO_WRITE:
    case CMD_DIGITAL_WRITE:
    case CMD_ANALOG_WRITE:
        writeCallbacks[rxPacket.cmd](rxPacket);
        break;
    case CMD_SERVO_READ:
    case CMD_DIGITAL_READ:
    case CMD_ANALOG_READ:
    case CMD_ESP32_STATE:
        readCallbacks[rxPacket.cmd](rxPacket);
        break;
    default:
        DEBUG_SERIAL.printf("[I2C] rxBuffer unbekanntes Command-Flag (%d)\n", rxPacket.cmd);
        return;
    }
}

//-------------------------------------------------------------
// I2C Request Event
//-------------------------------------------------------------
void requestEvent()
{
    // Antwort wird immer direkt in handleReadCommand() erzeugt
}



//-------------------------------------------------------------
// Setup
//-------------------------------------------------------------
void setup()
{
    Serial.begin(115200);
    DEBUG_SERIAL.printf("[setup]  ESP32 I2C slave startet, FastMode (400kHz)...\n");

    DEBUG_SERIAL.println("[setup] Initialisiere I2C-Slave...");

    bool i2c_ready = false;
    int timeout = 0;
    while (!i2c_ready)
    {

        if (!initI2CSlave(DEFAULT_I2C_PORT, DEFAULT_I2C_SDA, DEFAULT_I2C_SCL,
                          DEFAULT_I2C_ADDR, DEFAULT_I2C_BUF_SIZE,
                          receiveEvent, requestEvent))
        {
            DEBUG_SERIAL.println("I2C-Initialisierung fehlgeschlagen!");
            i2c_ready = false;
            timeout++;
            if (timeout > 5)
            {
                DEBUG_SERIAL.println("I2C Error nach 5 Fehlversuchen - reboot");
                while (true)
                {
                }
            }
            delay(1000);
        }
        else
        {
            i2c_ready = true;
        }
    }

    DEBUG_SERIAL.println("I2C-Slave bereit");

    for (uint8_t i = 0; i < CMD_MAX; i++)
    {
        writeCallbacks[i] = nullptr;
        readCallbacks[i] = nullptr;
    }

    // Write-Callbacks
    writeCallbacks[CMD_SERVO_WRITE] = cmdServoWrite;
    writeCallbacks[CMD_DIGITAL_WRITE] = cmdDigitalWrite;
    writeCallbacks[CMD_ANALOG_WRITE] = cmdAnalogWrite;

    // Read-Callbacks
    readCallbacks[CMD_SERVO_READ] = cmdServoRead;
    readCallbacks[CMD_DIGITAL_READ] = cmdDigitalRead;
    readCallbacks[CMD_ANALOG_READ] = cmdAnalogRead;
    readCallbacks[CMD_ESP32_STATE] = cmdESP32State;

    DEBUG_SERIAL.println("I2C Kommandos erstellt");

    // Dynamixel starten
    DXL_SERIAL.begin(DXL_BAUDRATE, SERIAL_8N1, 16, 17);
    dxl.begin(DXL_BAUDRATE);
    dxl.setPortProtocolVersion(DXL_PROTOCOL);
    delay(2000);

    // Dynamixel-Scan
    // alle gefundenen Servos erst einmal auf MODE: OP_POSTION setzen (Winkel 0-300Grad)
    // scanDynamixels(dynaList, OP_POSITION);
    if (!scanAllDynamixels(&dxl))
    {
        // DEBUG_SERIAL.println("SERVO-Liste falsch konfiguriert. Angegebene Servos nicht gefunden.");
        while (true)
        {
            delay(1000);
            DEBUG_SERIAL.print("ERROR ");
            setLEDPattern(LED1, LED2, LED_DYNA_NOSERVOS, 500);

        }
    }

    // vier Lenk-Servos erstellen
    // ID=ServoID, Type=Steering oder Wheel, POS=VL(vorneLinks), ...
    // CW=CW_ANGLE_LIMIT, CCW = CCW_ANGLE_LIMIT
    // MID= Mittelposition
    // START = Startposition - Rover fährt in gerader Linie

    #ifdef RAINER_ROVER
        // Konfiguration von Bernd
        // vier Wheel-Servos erstellen
        //                                        ID,Type,POS,CW,CCW,MID,START
        velocityServos.add(createRoverServo(&dxl, 7, WHEEL, VL, 0, 0, 0, 0));
        velocityServos.add(createRoverServo(&dxl, 3, WHEEL, VR, 0, 0, 0, 0));
        velocityServos.add(createRoverServo(&dxl, 1, WHEEL, HL, 0, 0, 0, 0));
        velocityServos.add(createRoverServo(&dxl, 5, WHEEL, HR, 0, 0, 0, 0));

        // Lenk-Servos
        //                                        ID,Type,POS,CW,CCW,MID,StartAngle, backward, forward, base_velo, TorqueOn
        steeringServos.add(createRoverServo(&dxl, 8, STEERING, VL, 285, 673, 511, 356, 0, 1024, 200, true));  // ok
        steeringServos.add(createRoverServo(&dxl, 4, STEERING, VR, 358, 746, 511, 675, 1024, 0, 200, true));  // ok
        steeringServos.add(createRoverServo(&dxl, 2, STEERING, HL, 358, 746, 511, 675, 0, 1024, 200, true)); // ok
        steeringServos.add(createRoverServo(&dxl, 6, STEERING, HR, 285, 673, 511, 356, 1024, 0, 200, true)); // ok
    #else
        // Konfiguration von Bernd
        // vier Wheel-Servos erstellen
        //                                        ID,Type,POS,CW,CCW,MID,START
        velocityServos.add(createRoverServo(&dxl, 4, WHEEL, VL, 0, 0, 0, 0));
        velocityServos.add(createRoverServo(&dxl, 11, WHEEL, VR, 0, 0, 0, 0));
        velocityServos.add(createRoverServo(&dxl, 3, WHEEL, HL, 0, 0, 0, 0));
        velocityServos.add(createRoverServo(&dxl, 1, WHEEL, HR, 0, 0, 0, 0));

        // Lenk-Servos
        //                                        ID,Type,POS,CW,CCW,MID,StartAngle, backward, forward, base_velo, TorqueOn
        steeringServos.add(createRoverServo(&dxl, 8, STEERING, VL, 285, 673, 511, 356, 0, 1024, 200, true));  // ok
        steeringServos.add(createRoverServo(&dxl, 7, STEERING, VR, 358, 746, 511, 675, 1024, 0, 200, true));  // ok
        steeringServos.add(createRoverServo(&dxl, 17, STEERING, HL, 358, 746, 511, 675, 0, 1024, 200, true)); // ok
        steeringServos.add(createRoverServo(&dxl, 18, STEERING, HR, 285, 673, 511, 356, 1024, 0, 200, true)); // ok

    #endif


    setAllServosToStandardPosition(&dxl, steeringServos, SteeringPosition::REGULAR);
    setRoverVelocity(&dxl, 0.0, 0.0, 0);

    DEBUG_SERIAL.println("Steering Servos in Startposition");

    DEBUG_SERIAL.printf("[setup]  ESP32 I2C slave ready\n");
}

// -----------------------------
// Hauptloop
// -----------------------------
void loop()
{
    // Hintergrund-Task / Nicht blockierend!
    delay(1); // Minimale Entlastung
}