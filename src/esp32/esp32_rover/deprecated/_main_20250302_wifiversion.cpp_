#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>
#include <Dynamixel2Arduino.h>
#include <Vector.h>
#include "rover.h"
#include "rover_utils.h"

/************************************************************************************
 * 
 * ROS2 ESP32 Komponente
 * 
 * Der ESP32 baut einen Wifi  UDP-Socket Verbindung auf und stellt diese dann
 * dem Python-Rover modul für die Kommunikation zur Verfügung
 * 
 * Später soll die Verbindung statt UDP über RS485 stattfinden, da die Möglichkeit
 * geschaffen werden soll, an einen Python-Node mehrere ESP32 andocken zu können
 * 
 * die Datenübertragung muss sehr schnell sein, daher ist bei Wifi UDP eine gute wahl
 * ein HTTP-Socket ist zu langsam
 * 
 * 
 * 
 ***********************************************************************************/

#define TEST_VL
//#define TEST_HL
//#define TEST_VR
//#define TEST_HR




// Dynamixel Commands
#define DXL_

const unsigned long DXL_BAUDRATE = 1000000;
const float DXL_PROTOCOL = 1.0;

int servo_angle = 0; //

/* statische IP-Adresse, damit sichergestellt wird, das der ESP32 grundsätzlich immer mit der gleichen IP angesprochen werden kann*/
IPAddress staticIP(192, 168, 0, 199); // Statische IP-Adresse
IPAddress gateway(192, 168, 0, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress primaryDNS(192, 168, 0, 1);
IPAddress secondDNS(0, 0, 0, 0);

// UDP-Konfiguration
WiFiUDP udp;
const unsigned int localUdpPort = 12345; // Port des ESP32
char incomingPacket[512];                // Puffer für eingehende UDP-Pakete

int store_elements[DXL_BROADCAST_ID - 1];
typedef Vector<int> TDynaList;

TDynaList dynaList(store_elements);
char buffer[200];

// Dynamixel-Objekt
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

//void scanDynamixels(TDynaList &list, uint8_t opMode = OP_POSITION);
bool scanDynamixels();
void handleIncomingMessage(String msg);
void publishSensorData();
void goStartPosition();

void setup()
{
  // Debug starten
  DEBUG_SERIAL.begin(115200);
  while (!DEBUG_SERIAL)
    ;

  delay(2000) ; // warten um VSCode zeit zu geben den Monitor zu öffnen

  // Dynamixel starten
  DXL_SERIAL.begin(DXL_BAUDRATE, SERIAL_8N1, 16, 17);
  dxl.begin(DXL_BAUDRATE);
  dxl.setPortProtocolVersion(DXL_PROTOCOL);

  delay(500);

  resetAllServos(&dxl);

  // Dynamixel-Scan
  // alle gefundenen Servos erst einmal auf MODE: OP_POSTION setzen (Winkel 0-300Grad)
  //scanDynamixels(dynaList, OP_POSITION);
  if (!scanDynamixels()) {
    DEBUG_SERIAL.println("SERVO-Liste falsch konfiguriert. Angegebene Servos nicht gefunden.");
    while(true) {delay(1000);DEBUG_SERIAL.print("ERROR ");}
  }

  DEBUG_SERIAL.println("Dynamixel gestartet!");

  // die Servo-Limits werden imm EEPROM des Servos gespeichert um zu vermeiden,
  // das versehentlich das Limit überschritten wird und der Servo beschädigt wird (oder anderes)
  // for (uint8_t id = 0; id < ARRAY_SIZE(dxl_ids_steering); id++) {
  //   saveAngleLimits(&dxl, 
  //     dxl_ids_steering[id], 0, 300
  //     // dxl_id_steering_range[id][0], // min
  //     // dxl_id_steering_range[id][1]  // max
  //   );
  //   DEBUG_SERIAL.printf("Limit Servo (%02d) MIN(%3.1f), MAX(%3.1f)\n", 
  //     dxl_ids_steering[id],
  //     dxl_id_steering_range[id][0], // min
  //     dxl_id_steering_range[id][1]  // max
  //   );
  // }

  goStartPosition();
  DEBUG_SERIAL.println("Antriebseinheiten in Startposition");

  printCurrentAngles(&dxl);

  // WLAN verbinden
  WiFi.config(staticIP, gateway, subnet, primaryDNS, secondDNS); // Statische IP einstellen
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    DEBUG_SERIAL.print(".");
  }
  DEBUG_SERIAL.println("\nWLAN verbunden!");
  DEBUG_SERIAL.println(WiFi.localIP());

  // UDP starten
  udp.begin(localUdpPort);
  DEBUG_SERIAL.printf("UDP Server gestartet auf Port %d\n", localUdpPort);


}

void loop()
{
  int packetSize = udp.parsePacket();
  if (packetSize)
  {
    // Daten empfangen
    int len = udp.read(incomingPacket, sizeof(incomingPacket) - 1);
    if (len > 0)
    {
      incomingPacket[len] = 0; // Null-terminieren
    }
    DEBUG_SERIAL.printf("Empfangen: %s\n", incomingPacket);

    // Nachricht verarbeiten
    handleIncomingMessage(String(incomingPacket));
  }

  // Sensordaten senden (optional, falls du regelmäßig Daten senden willst)
  publishSensorData();
}

// /**
//  * @brief Scan für Dynamixels. Wenn gefunden, werden die IDs in eine Liste aufgenommen.
//  */
// void scanDynamixels(TDynaList &list, uint8_t opMode)
// {
//   if (dynaList.size() < DXL_BROADCAST_ID)
//   {
//     for (uint8_t id = 0; id < DXL_BROADCAST_ID; id++)
//     {
//       if (dxl.ping(id))
//       {
//         DEBUG_SERIAL.printf("ID (%3d), Model: %3d\n", id, dxl.getModelNumber(id));
//         dynaList.push_back(id);
//         // um den MODE zu setzen muss vorher TORQUE deaktiviert werden
//         dxl.torqueOff(id);
//         dxl.setOperatingMode(id, opMode);
//         // um den Servo zu bewegen muss TORQUE wieder aktiviert werden
//         dxl.torqueOn(id);
//       }
//     }
//     DEBUG_SERIAL.printf("Number of Dynamixels found: {%3d}\n", dynaList.size());
//   }
// }

/**
 * @brief Scannt die Dynamixel-Kette und sucht ob die definierten IDs auch verfügbar sind.
 * Wenn nein liefert die Funktion false zurück. Wurde ein Serve gefunden, wird auch direkt der
 * Mode für diesen Servo gesetzt. OP_VELOCITY (für Antriebsservos), OP_POSITION (für lenkservos)
 * 
 */
bool scanDynamixels() {
  size_t s = ARRAY_SIZE(dxl_ids_steering);
  bool found = true;
  for (uint8_t id=0; id < s; id++) {
    // Steering Servos
    if (dxl_ids_steering[id] > 0 && dxl.ping(dxl_ids_steering[id])) {
      DEBUG_SERIAL.printf("STEERING-Servo ID:(%2d) MODE: (%d) MODEL:(%3d)\n", 
        dxl_ids_steering[id], OP_POSITION, dxl.getModelNumber(id));
      dxl.torqueOff(id); // Torque abschalten um den Mode zu setzen
      dxl.setOperatingMode(id, OP_POSITION);
      dxl.torqueOn(id);  // Troque wieder aktivieren um den Servo nutzen zu können
    }
    else {
      if (dxl_ids_steering[id] == 0) continue;
      DEBUG_SERIAL.printf("STEERING-Servo ID:(%2d) Servo not found - error - check dxl_ids_steering\n", 
        dxl_ids_steering[id]);
      found = false;

    }
    // Antriebsservos
    if (dxl_ids_velocity[id] > 0 && dxl.ping(dxl_ids_velocity[id])) {
      DEBUG_SERIAL.printf("VELOCITY-Servo ID:(%2d) MODE: (%d) MODEL:(%3d)\n", 
        dxl_ids_velocity[id], OP_VELOCITY, dxl.getModelNumber(id));
      dxl.torqueOff(id); // Torque abschalten um den Mode zu setzen
      dxl.setOperatingMode(id, OP_VELOCITY);
      dxl.torqueOn(id);  // Troque wieder aktivieren um den Servo nutzen zu können
    }
    else {
      if (dxl_ids_velocity[id] == 0) continue;
      DEBUG_SERIAL.printf("VELOCITY-Servo ID:(%2d) Servo not found - error - check dxl_ids_velocity\n", 
        dxl_ids_velocity[id]);
      found = false;

    }
  }
  return found;
}

/**
 * @brief Setzt für jeden Antriebsstrang den Servo auf die Ausgangsstellung
 * Schritt 1 : Servo auf 150°, dann für den jeweiligen Antriebsstrang in seine Postion
 * für vorwärts/rückwärts
 * 
 */
void goStartPosition() {
  size_t s = ARRAY_SIZE(dxl_ids_steering);
  for (uint8_t id=0; id < s; id++) {
    // Fahre Servo in die Mittelposition - langsam
    if (dxl_ids_steering[id] == 0) continue;
    DEBUG_SERIAL.printf("ID(%2d) move to center....\n", dxl_ids_steering[id]);
    setAngle(&dxl, 
        dxl_ids_steering[id],
        dxl_id_steering_range[id][3], 
        dxl_id_steering_range[id][0], 
        dxl_id_steering_range[id][1], 
        UNIT_DEGREE,
        100
    );
  }
  delay(5000);
  for (uint8_t id=0; id < s; id++) {
    // Fahre Servo in die Startposition (max beschleunigung)
    if (dxl_ids_steering[id] == 0) continue;
    DEBUG_SERIAL.printf("ID(%2d) move to start pos (%3.1.f)°...\n", dxl_ids_steering[id],dxl_id_steering_range[id][2]);
    setAngle(&dxl, 
      dxl_ids_steering[id], dxl_id_steering_range[id][2], 
      dxl_id_steering_range[id][0], 
      dxl_id_steering_range[id][1], 
      UNIT_DEGREE,
      0 
    );
  }

}

/**
 * @brief Eingehende UDP-Nachricht verarbeiten
 * IncommingMessage ist ein Steuerbefehl für die Motoren
 * und wird vom Python-Node versendet.
 * 
 */
void handleIncomingMessage(String msg)
{
  DynamicJsonDocument doc(1024);
  DeserializationError error = deserializeJson(doc, msg);

  if (error)
  {
    DEBUG_SERIAL.println("JSON-Parsing fehlgeschlagen!");
    return;
  }
  DEBUG_SERIAL.println("Parsing done");

  //  const char *command = doc["command"];
  const char *command = doc["cmd"];
  if (strcmp(command, "move_motor") == 0)
  {
    DEBUG_SERIAL.println("set DEGREES");
    int motor_id = doc["id"];
    float position = doc["position"];
    int unit = UNIT_DEGREE;
    dxl.setGoalPosition(motor_id, position, UNIT_DEGREE); // Position setzen
    DEBUG_SERIAL.printf("Motor %d auf Winkel %f° bewegt\n", motor_id, position);
  }
}

/**
 * @brief Sensordaten an den letzten Sender senden
 */
void publishSensorData()
{
  DynamicJsonDocument doc(1024);
  doc["sensor"] = random(0, 1024);                 // Beispiel: Zufallswert
  doc["motor_status"] = dxl.getPresentPosition(1); // Beispiel: Motorposition für Motor 1
  String jsonStr;
  serializeJson(doc, jsonStr);

  // Sensordaten an den Sender senden
  udp.beginPacket(udp.remoteIP(), udp.remotePort());
  udp.print(jsonStr);
  udp.endPacket();
}
