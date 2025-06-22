
#ifndef _ROVER_H_ 
#define _ROVER_H_
#include <Arduino.h>

 /** 
  * verfügbare SSID zur Kopplung definieren
  */
 #define WIFI_SSID "FRITZ!Box 7590 UL"        // WLAN-SSID
 #define WIFI_PASSWORD "95493765313007016133" // WLAN-Passwort
 
 // Dynamixel-Konfiguration
 #define DXL_SERIAL Serial1  // Dynamixel auf Serial1
 #define DEBUG_SERIAL Serial // Debug auf Serial0
 #define DXL_DIR_PIN 4       // GPIO für Dynamixel-Richtung
 #define DXL_BROADCAST_ID 254
 
#define MOVING_SPEED 32 // das ist die Adresse für Beschleunigung in der DynamixelControlTable

uint8_t dxl_ids_steering[4] = {14, 15, 0, 0};    // VL, VR, HL, HR
uint8_t dxl_ids_velocity[4] = {7, 10, 0, 0};     // VL, VR, HL, HR

// 2D Array für alle vier Lenk-Servos
// Min/Max Winkel und Startposition (Winkel)
float dxl_id_steering_range[4][4] =
{
    // MIN, MAX, START, CENTER
    {60.0,240.0,135.0,150.0},       // VL
    {60.0,240.0, 45.0,150.0},        // VR
    {60.0,240.0, 45.0,150.0},       // HL
    {60.0,240.0,135.0,150.0}       // HR
};

#endif