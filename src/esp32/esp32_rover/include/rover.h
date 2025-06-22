
#ifndef _ROVER_H_
#define _ROVER_H_
#include <Arduino.h>
#include "DynamicArray.h"
#include <Vector.h>
#include <Ticker.h>
//------------------------------------------------------------------------------
// I2C Pins & Dynamix-Pin konfigurieren
//------------------------------------------------------------------------------
// I2C Konfiguration
// Default-Werte
#define DEFAULT_I2C_PORT I2C_NUM_0
#define DEFAULT_I2C_SDA 21
#define DEFAULT_I2C_SCL 22
#define DEFAULT_I2C_ADDR 0x12
#define DEFAULT_I2C_BUF_SIZE 512
//
// Dynamixel-Konfiguration
#define DXL_SERIAL Serial1  // Dynamixel auf Serial1
#define DEBUG_SERIAL Serial // Debug auf Serial0
#define DXL_DIR_PIN 4       // GPIO für Dynamixel-Richtung
#define DXL_BROADCAST_ID 254

const unsigned long DXL_BAUDRATE = 1000000;
const float DXL_PROTOCOL = 1.0;
//------------------------------------------------------------------------------


int store_elements[DXL_BROADCAST_ID - 1];
typedef Vector<int> TDynaList;
TDynaList dynaList(store_elements);
char buffer[200];

// Dynamixel-Objekt
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);


/**
 * verfügbare SSID zur Kopplung definieren
 */
//define WIFI_SSID "FRITZ!Box 7590 UL"        // WLAN-SSID
//define WIFI_PASSWORD "95493765313007016133" // WLAN-Passwort


enum RoverParams
{
    MIN_RADIUS = 250, // kleinster Wendekreisradius in mm
    SPURWEITE = 175,  // Abstand L <-> R in mm
    RADSTAND = 185,   // Abstand V <-> H in mm
    RAD_DIA = 120,    // Durchmesser Rad in mm (für Geschwindigkeitsberechnung)
    MIN_DEG = 0,      // kleinster Winkel in Grad des Servos
    MAX_DEG = 300     // maximaler Winkel in Grad des Servos

};

/*
 * Wo ist der Servo verbaut (RoverPosition)
 */
enum ServoPosition
{
    VL = 0,
    VR = 1,
    HL = 2,
    HR = 3
};

/**
 * Lenkservo oder Antriebsservo
 */
enum ServoType
{
    STEERING = 0,
    WHEEL = 1
};

enum SteeringPosition
{
    REGULAR = 0, // entspricht der START_ANGLE_POSITION
    TURN = 1,    // entspricht der MID_ANGLE_POSITION, Rover dreht sich um seine eignee Achse
    SIDEWAY = 2  // entspricht der Stellung aller Räder, das der Rover seitwärts fahren kann (VL auf CW_ANGLE, VR auf CCW_ANGLE, HL auf CCW_ANGLE, HR=CW_ANGLE)
};

enum SteeringDirection
{
    STRAIGHT = 0,
    LEFT = 1,
    RIGHT = 2,
};

/**
 * @brief verschiedene Lenk/Drehbewegungen des Rovers
 *
 * Ackermann
 * Lenkung nach Ackerman Beispiel Linksdrehung: VL und VR drehen nach Links (CCW), HL & HR drehen nach rechts (CW).
 * Typische Autolenkung (hier aber mit vier lenkbaren Rädern)
 *
 * CRAB
 * Lenkung ähnlich ener Krabbe Rover soll nach links ausweichen: VL, VR, HL, HR drehen alle CCW.
 * Heißt der Rover macht direkt eine Bewegung nach links
 */
enum RoverTurnMode
{
    ACKERMANN = 0,
    CRAB = 1
};

/**
 * Ein RoverServo
 */
typedef struct
{
    uint8_t id;
    ServoPosition servo_position;
    ServoType servo_type;

    // nur bei ServoType STEERING wichtig
    int START_ANGLE_POSITION = 0;
    int MID_ANGLE_POSITION = 511;
    int CW_ANGLE_LIMIT = 0;
    int CCW_ANGLE_LIMIT = 1023;

    // nur by ServoType WHEEL wichtig
    int BACKWARD_START_VELOCITY = 1024;
    int FORWARD_START_VOLOCITY = 0;
    int BASE_VELOCITY;
} RoverServo;

DynamicArray<RoverServo *> velocityServos;
DynamicArray<RoverServo *> steeringServos;

#pragma pack(push,1)
struct I2C_CommandPacket {
    uint16_t header;
    uint8_t cmd;
    uint8_t subcmd;
    uint8_t flags;
    uint16_t data[5];
    uint8_t reserved;
    uint8_t crc;
};
#pragma pack(pop)

#endif