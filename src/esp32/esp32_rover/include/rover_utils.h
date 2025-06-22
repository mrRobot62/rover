#ifndef _ROVER_UTILS_H_
#define _ROVER_UTILS_H_

#include <Arduino.h>
#include <Dynamixel2Arduino.h>
#include "rover.h"

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))
#define SPEED_MULTIPLIER 1000

// Funktion zur Überprüfung, ob eine ID im Array vorhanden ist
/**
 * @brief prüft ob ein Wert im übergebenen Array enthalten ist
 * Wird z.B. benötigt um herauszufinden ob eine Servo-ID in der
 * Liste der benutzten Servos steht
 *
 * @param id DynamixelID
 * @param *array Zeiger auf das zu prüfende Array
 * @param size Größe des zu prüfenden Arrays
 */
uint8_t id_exists(uint8_t id, const uint8_t *array, uint8_t size)
{
    for (uint8_t i = 0; i < size; i++)
    {
        if (array[i] == id)
        {
            return 1; // ID gefunden
        }
    }
    return 0; // ID nicht gefunden
}

/**
 * @brief Konvertiert einen Winkelwert in einen Positionswert zw. 0..1023 und setzt die Position des Servos
 * Der übergebene Wert kann ein Prozentwert zwischen 0.0..100.0, ein Winkel 0.0..300.0 oder ein Rohwert von 0..1023 sein
 * Unterschieden wird dies über den Paramter unit. Die Werte für min/max müssen an den übergabe Wert (value) entsprechend angepasst werden beim
 * Aufruf der funktion
 *
 * @param dxl Zeiger auf das dxl-objekt
 * @param id Dynamixel ID
 * @param min/max Min/Max bezogen auf den Value
 * @param float wenn UNIT_DEGREE dann Wert zwischen 0..300,
 *          wenn UNIT_RAW, dann Wert zwischen -1.0 .. 0 .. 1.0,
 *          wenn UNIT_PERCENT, dann value zwischen -100.0 .. 0 .. 100.0,
 *          default = UNIT_RAW
 * @param velocity kann genutzt werden um die Beschleunigung anzupassen. Default 0 (max) 1=sehr langsam CW, 1023 max CW, 1024 langsam CCW, 2047 max CCW
 */
void setAngle(Dynamixel2Arduino *dxl, uint8_t id, float value, float min = -1.0, float max = 1.0, uint8_t unit = UNIT_RAW, uint16_t velocity = 0)
{
}

/**
 * @brief gibt den aktuellen Winkel der Servostellung zurück für Servo id zurück
 */
float getPostionInDegree(Dynamixel2Arduino *dxl, uint8_t id)
{
    float raw = dxl->getPresentPosition(id, UNIT_RAW);
    return map(raw, 0, 1023, 0, 300);
}

void printCurrentAngles(Dynamixel2Arduino *dxl)
{
    // size_t s = ARRAY_SIZE(dxl_ids_steering);
    // float angle;
    // for (uint8_t id = 0; id < s; id++)
    // {
    //     if (dxl_ids_steering[id] == 0)
    //         continue;
    //     angle = getPostionInDegree(dxl, dxl_ids_steering[id]);
    //     DEBUG_SERIAL.printf("SERVO(%02d) - Winkel: (%3.2f)° \n", dxl_ids_steering[id], angle);
    // }
}

/**
 * @brief setzt eine physikalisches Winkel-Limit für den Servo. Wird im EEPROM gespeichert und ist auch nach einem Neustart vorhanden !
 * Reset kann nur mit min=0, max=300 auf den maximal Winkelausschlag zurück gesetzt werden !
 */
void saveAngleLimits(Dynamixel2Arduino *dxl, uint8_t id, uint16_t min = 0, uint16_t max = 300)
{
    dxl->torqueOff(id);
    min = map(min, 0, 300, 0, 1023);
    max = map(max, 0, 300, 0, 1023);
    dxl->writeControlTableItem(6, id, min);
    dxl->writeControlTableItem(8, id, max);
    dxl->torqueOn(id);
}

/**
 * @brief setzt für den Servo den Wheel Modus. Wird im EEPROM des Servos gespeichert
 */
void setWheelMode(Dynamixel2Arduino *dxl, uint8_t id)
{
    dxl->writeControlTableItem(ControlTableItem::CW_ANGLE_LIMIT, id, 0);
    dxl->writeControlTableItem(ControlTableItem::CCW_ANGLE_LIMIT, id, 0);
}

/**
 * @brief führt ein PING über die komplette Chain durch
 */
bool scanAllDynamixels(Dynamixel2Arduino *dxl, uint8_t from = 1, uint8_t to = 253)
{
    bool found = false;
    for (int id = from; id < to; id++)
    {
        if (dxl->ping(id))
        {
            DEBUG_SERIAL.printf("scan found ID: %3d - model: (%2d)\n", id, dxl->getModelNumber(id));
            found = true;
        }
    }
    if (!found)
    {
        DEBUG_SERIAL.print("------ NO SERVOS FOUND ------");
    }
    return found;
}

/**
 * @brief Geschwindigkeit des Rovers für alle Antriebsservos setzen.
 * Die Geschwindigkeit wird in einem Bereich von 0-1023 angegeben. Forward/Backward wird über DIR angeben (0=Backward, 1=Forward(default))
 * Verstärkungs- oder Dämpfungswert über GAIN. Wertebereich von -1.0 (Dämpfen) bis +1.0 (Verstärken)
 * Nutzbar um den eigentlichen Geschwindigkeitswert beibehalten und trotzdem die Geschwindigkeit (z.B. in Kurven) anzupassen
 *
 * Gilt für ALLE Antriebsservos
 *
 * @param dxl Pointer auf das Dynamixel2Arduino Objekt
 * @param speed Geschwindigkeit zwischen 0-1023
 * @param gain Verstärkung-/Dämpfungsfaktor von -1.0(dämpfen) bis +1.0(Verstärken)
 * @param dir Forward(1), Backward(0)
 *
 */
void setRoverVelocity(Dynamixel2Arduino *dxl, uint16_t speed, float gain = 0.0, uint8_t dir = 1)
{
    uint16_t adjusted_speed;
    if (speed > 1023)
        speed = 1023; // Begrenzung des Speed-Werts
    if (gain < -1.0)
        gain = -1.0; // Begrenzung von gain
    if (gain > 1.0)
        gain = 1.0;

    // DEBUG_SERIAL.printf("[%6d] [VELOCITY] => ", millis());
    for (auto &servo : velocityServos)
    {
        uint8_t servo_id = servo->id;
        uint16_t adjusted_speed = speed;

        switch (servo->servo_position) {
            case VL:
            case HL:{
                if (dir == 1)
                    adjusted_speed += servo->FORWARD_START_VOLOCITY;
                else
                    adjusted_speed += servo->BACKWARD_START_VELOCITY;
                break;
            }
            case VR:
            case HR:{
                if (dir == 1)
                    adjusted_speed += servo->BACKWARD_START_VELOCITY;
                else
                    adjusted_speed += servo->FORWARD_START_VOLOCITY;
                break;
            }
        }

        if (adjusted_speed < 1024) {
            adjusted_speed += (speed * gain);
            // Grenzen einhalten (0-1023)
            adjusted_speed = (adjusted_speed < 0) ? 0 : adjusted_speed;
            adjusted_speed = (adjusted_speed > 1023) ? 1023 : adjusted_speed;
        }
        if (adjusted_speed > 1023) {
            adjusted_speed += (speed * gain);
            // Trenzen einhalten (1024-2047)
            adjusted_speed = (adjusted_speed < 1024) ? 1024 : adjusted_speed;
            adjusted_speed = (adjusted_speed > 2027) ? 2027 : adjusted_speed;

        }

        // Berechnung des angepassten Geschwindigkeitswerts
        //int32_t new_speed = speed + (speed * gain);

        // if (gain > 0.0)
        // {
        //     // Verstärkung: Obergrenze prüfen
        //     if (base_speed == 0)
        //     { // CCW
        //         adjusted_speed = (new_speed > 1023) ? 1023 : (base_speed + new_speed);
        //     }
        //     else
        //     { // CW
        //         adjusted_speed = (new_speed > 2047) ? 2047 : (base_speed + new_speed);
        //     }
        // }
        // else if (gain < 0.0)
        // {
        //     // Dämpfung: Untergrenze prüfen
        //     if (base_speed == 0)
        //     { // CCW
        //         adjusted_speed = (new_speed < 0) ? 0 : (base_speed + new_speed);
        //     }
        //     else
        //     { // CW
        //         adjusted_speed = (new_speed < 1024) ? 1024 : (base_speed + new_speed);
        //     }
        // }
        // else
        // {
        //     adjusted_speed = (base_speed + speed);
        // }

        // Richtung umkehren, wenn dir == 0 (Rückwärtsmodus)
        // if (dir == 0)
        // {
        //     adjusted_speed = (adjusted_speed < 1024) ? adjusted_speed + 1024 : adjusted_speed - 1024;
        // }
        // Geschwindigkeitswert an den Servo senden
        // DEBUG_SERIAL.printf("| (ID:%d)::(%4d) ", servo_id, adjusted_speed);
        dxl->setGoalVelocity(servo_id, adjusted_speed, UNIT_RAW);
        delay(10);
    }
    // DEBUG_SERIAL.println("|\n");
}

/**
 * @brief speed im Bereich von -1023 bis +1023. Wird intern in 0-1023 und dir 0 oder 1 umgewandelt. Negativ = dir=0, Positiv = dir=1. Gilt für ALLE Antriebsservos
 */
void setRoverVelocity(Dynamixel2Arduino *dxl, int speed, float gain = 0.0)
{
    // DEBUG_SERIAL.printf("[setRoverVelocity(int)] : %d\n", speed);

    if (speed < 0)
    {
        setRoverVelocity(dxl, abs(speed), gain, 0);
    }
    else
    {
        setRoverVelocity(dxl, abs(speed), gain, 1);
    }
}

/**
 * @brief Geschwindigkeit des Rovers für alle Antriebsservos setzen. Werte Bereich von -1.0 bis +1.0 (Gamepad Axis Values)
 * Die Geschwindigkeit bezieht sich auf Ausgaben des Gamepad-Controllers und wird in den passenden Wertebereich des Dynamixel-Servos
 * konvertiert.
 * Verstärkungs- oder Dämpfungswert über GAIN. Wertebereich von -1.0 (Dämpfen) bis +1.0 (Verstärken)
 * Nutzbar um den eigentlichen Geschwindigkeitswert beibehalten und trotzdem die Geschwindigkeit (z.B. in Kurven) anzupassen.alignas
 *
 * Gilt für ALLE Antriebsservos
 *
 * @param dxl Dynamixel2Arduino Objekt
 * @param speed Geschwindigkeitsbereich von -1.0 bis +1.0 (postiv = Forward, negativ = Backward)
 * @param gain Verstärungs-/Dämpfungsfaktor von -1.0 bis +1.0
 *
 */
void setRoverVelocity(Dynamixel2Arduino *dxl, float speed, float gain = 0.0)
{
    uint8_t dir = 1;
    int adjusted_speed = 0;
    int s_adj = 0;

    if (speed < -1.0)
        speed = -1.0;
    if (speed > 1.0)
        speed = 1.0;
    if (gain < -1.0)
        gain = -1.0;
    if (gain > 1.0)
        gain = 1.0;

    s_adj = long(speed * SPEED_MULTIPLIER);
    //
    // der Wertebereich von -1.0 bis +1.0 muss nun in einen Wertebereich von -1023 bis +1023 gemapped werden
    // der Float-Wert wird im mit 1000 multipliziert um eine möglichst gute Linearisierung zu erreichen
    adjusted_speed = map(s_adj, -SPEED_MULTIPLIER, SPEED_MULTIPLIER, -1023, 1023);
    // DEBUG_SERIAL.printf("[setRoverVelocity(float)] : %3.2f\n", speed);

    setRoverVelocity(dxl, adjusted_speed, gain);
}

/**
 * @brief generiert einen RoverServo basieren auf konfigurierte Parameter. Mit einerm Servo-Objkt arbeiten ist einfacher als sich die
 * Daten permament aus Array auszulesen.alignas
 *
 * @param dxl Dynamixel2Arduino Objekt
 * @param id ID des Servos
 * @param stype Type des Servos (Steering/Joint oder WHEEL)
 * @param spos Position des Servosm(entspricht dem ENUM VL/VR/HL/HR)
 * @param cw_angle Limit Angle CW
 * @param ccw_angle Limit Angle CCW
 * @param mid_angle Mittlere Position des Servos
 * @param start_angle Postion für gerade aus fahren
 * @param bck_velo Min-Value für Backward Drehen im Wheel-Modus (z.B ab 1024-2047)
 * @param fwd_velo Min-Value für Forward Drehen im Wheel-Modus (z.B. ab 0-1023)
 * @param torqueOn Default true, aktiviert den Servo direkt
 */
RoverServo *createRoverServo(Dynamixel2Arduino *dxl, uint8_t id, ServoType stype, ServoPosition spos, int cw_angle = 0, int ccw_angle = 1023, int mid_angle = 511, int start_angle = 0, int bck_velo = 1024, int fwd_velo = 0, int base_velo = 0, bool torqueOn = true)
{
    RoverServo *servo = new RoverServo();
    if (id > 0)
    {
        servo->id = id;
        servo->servo_type = stype;
        servo->servo_position = spos;
        dxl->torqueOff(servo->id);        // zum Eeprom schreiben muss Torque deaktiviert werden
        servo->BASE_VELOCITY = base_velo; // gilt für Velocity & Steering

        if (stype == ServoType::STEERING)
        {
            // Steering/Lenk Servos bekommen eine Begrenzung des Drehwinkels.
            servo->CW_ANGLE_LIMIT = constrain(cw_angle, 0, 1023);
            servo->CCW_ANGLE_LIMIT = constrain(ccw_angle, 0, 1023);
            servo->MID_ANGLE_POSITION = constrain(mid_angle, 0, 1023);
            servo->START_ANGLE_POSITION = constrain(start_angle, 0, 1023);
            dxl->setOperatingMode(servo->id, OP_POSITION);
        }
        else
        {
            // Antrieb/Wheel Servos haben keine Winkelbegrenzung, daher CW/CCW auf 0
            servo->CW_ANGLE_LIMIT = 0;
            servo->CCW_ANGLE_LIMIT = 0;
            servo->BACKWARD_START_VELOCITY = bck_velo;
            servo->FORWARD_START_VOLOCITY = fwd_velo;
            dxl->setOperatingMode(servo->id, OP_VELOCITY);
        }
        // speichere im ServoEEPRO ob JOINT oder WHEEL Modus
        dxl->writeControlTableItem(ControlTableItem::CW_ANGLE_LIMIT, id, servo->CW_ANGLE_LIMIT);
        dxl->writeControlTableItem(ControlTableItem::CCW_ANGLE_LIMIT, id, servo->CCW_ANGLE_LIMIT);

        if (torqueOn)
        {
            dxl->torqueOn(servo->id);
        }
    }
    else
    {
        return nullptr;
    }
    // DEBUG_SERIAL.printf("new RoverServo (%3d), Type: (%d), Pos:(%d), CW_Limit:(%04d), CCW_Limit(%04d), Start (%04d), MID (%04d)\n",
    //                     servo->id,
    //                     servo->servo_type,
    //                     servo->servo_position,
    //                     servo->CW_ANGLE_LIMIT,
    //                     servo->CCW_ANGLE_LIMIT,
    //                     servo->START_ANGLE_POSITION,
    //                     servo->MID_ANGLE_POSITION);
    return servo;
}

/**
 * @brief Dreht alle Steering-Servos in ihre Startposition
 *(getestet funktioniert)

 * @param rtm Welche Grundposition soll eingenommen werden REGULAR = Default = Servos in Position das der Rover sich in einer Linie bewegen kann
 *              TURN = Räder so, das der Rover sich um seine Achse drehen kann
 * @param speed Wie schnell soll die servos bewegt werden, Default=1/2 Leistung (0-1023, default=511)
 */
void setAllServosToStandardPosition(Dynamixel2Arduino *dxl, DynamicArray<RoverServo *> list, SteeringPosition rtm = SteeringPosition::REGULAR, int speed = -1)
{
    long position;

    // long position = servo->MID_ANGLE_POSITION;
    for (auto &servo : list)
    {
        if (speed < 0)
        {
            speed = servo->BASE_VELOCITY;
        }
        switch (rtm)
        {
        case SteeringPosition::REGULAR:
            position = servo->START_ANGLE_POSITION;
            break;

        case SteeringPosition::TURN:
            position = servo->MID_ANGLE_POSITION;
            break;

        default:
            position = servo->START_ANGLE_POSITION;
            break;
        }
        dxl->setGoalVelocity(servo->id, speed, UNIT_RAW);
        dxl->setGoalPosition(servo->id, position, UNIT_RAW);
        // DEBUG_SERIAL.printf("Start Position (%3d) to %4d - read RAW (%4d) Deg (%5.2f) - MovingSpeed (%4d)\n",
        //                     servo->id,
        //                     position,
        //                     long(dxl->getPresentPosition(servo->id, UNIT_RAW)),
        //                     dxl->getPresentPosition(servo->id, UNIT_DEGREE),
        //                     speed);
        delay(10);
    }
}

void setSteeringServo(Dynamixel2Arduino *dxl, uint8_t id, int raw_angle, int speed = 0)
{
}

/**
 * @brief dreht alle Servos für eine Krümmung nach links
 * Der raw_pos value wird basierend auf die Servoposition in den dazugehörigen tatsächlichen Servobereich gempapped
 * VL 0=CW linker Anschlag als CW_ANGLE_LIMIT, VL 1023=CCW rechter Anschlag also CCW_ANGLELIMIT
 *
 * folgenes Servo Setup (VL CWA=300, CCWA=665, VR CWA=360, CCWA=730, HL=CWA=330, CCWA=775, HR=CWA=265, CCWA670)
 * Beispiel: raw_pos = 400 (im Wertebereich 0-1023),
 * VL 400 wird nun in einen Bereich von 300-665 gemapped =>
 *
 * @param inner_pos innere Servos und deren Position ist dann immer VL+HL oder VR+HR
 * @param outer_pos äußeren Servos und deren Position ist dann imme VL+HL oder VR+HR
 * @param speed Steuert die Geschwindigkeit der Drehung, default=500
 */
void setAllSteeringServosToPosition(Dynamixel2Arduino *dxl, SteeringDirection dir, int inner_pos, int outer_pos, int speed = 500)
{
    uint16_t mapped_pos_inner, mapped_pos_outer;
    if (dir == SteeringDirection::STRAIGHT)
    {
        setAllServosToStandardPosition(dxl, steeringServos, SteeringPosition::REGULAR, speed);
    }
    for (auto &servo : steeringServos)
    {
        dxl->setGoalVelocity(servo->id, speed, UNIT_RAW);
        switch (dir)
        {
        case SteeringDirection::STRAIGHT:
            setAllServosToStandardPosition(dxl, steeringServos, SteeringPosition::REGULAR);
            break;
        case SteeringDirection::LEFT:
            switch (servo->servo_position)
            {
            case ServoPosition::VL:
            case ServoPosition::HL:
                dxl->setGoalPosition(servo->id, mapped_pos_inner, UNIT_RAW);
                break;
            case ServoPosition::VR:
            case ServoPosition::HR:
                dxl->setGoalPosition(servo->id, mapped_pos_outer, UNIT_RAW);
                break;
            }
            break;
        case SteeringDirection::RIGHT:
            switch (servo->servo_position)
            {
            case ServoPosition::VL:
            case ServoPosition::HL:
                dxl->setGoalPosition(servo->id, mapped_pos_outer, UNIT_RAW);
                break;
            case ServoPosition::VR:
            case ServoPosition::HR:
                dxl->setGoalPosition(servo->id, mapped_pos_inner, UNIT_RAW);
                break;
            }
            break;
        }
        delay(15);
    }
}

/**
 * @brief Dreht alle Räder im Ackermann-Modus auf einen imaginären Kreis-Radius
 *
 * @param curve Krümmung in einem Wertefeld von -1 (maximale Krümmung) - 0 (geradeaus) - +1.0(max Krümmung andere Seite)
 * @param speed Geschwindigkeitsbereich von -1.0 bis +1.0 (negativ rückwärts, poisitiv = vorwärts)
 */
void setServoCurvature(Dynamixel2Arduino *dxl, float curve, uint16_t speed = 500)
{
    // Begrenzung auf gültigen Bereich
    if (curve < -1.0f)
        curve = -1.0f;
    if (curve > 1.0f)
        curve = 1.0f;

    int target_pos;

    // DEBUG_SERIAL.printf("[%6d] [CURVATUR] => ", millis());

    // Faktor für äußere Lenkung (etwas flacher als innen), alternativ über Geometrie berechnen
    const float OUTER_CURVE_FACTOR = 0.65; // 0.6–0.8 realistisch

    for (auto &servo : steeringServos)
    {
        float factor, factor_outer = 0.0;
        int range = 0;
        //
        //
        // der Bewegungsradius der Servos wird ermittelt.
        // entweder von CW_ANGLE_LIMIT zur START_POS oder von START_POS zu CCW_ANGLE_LIMIT
        switch (servo->servo_position)
        {
        case ServoPosition::VL:
        case ServoPosition::HR:
            range = servo->START_ANGLE_POSITION - servo->CW_ANGLE_LIMIT;
            break;
        case ServoPosition::VR:
        case ServoPosition::HL:
            range = servo->CCW_ANGLE_LIMIT - servo->START_ANGLE_POSITION;
            break;
        }

        switch (servo->servo_position)
        {
        case ServoPosition::VR:
        case ServoPosition::VL:
            if (curve < 0.0)
            {
                factor = abs(curve);
                factor_outer = factor * OUTER_CURVE_FACTOR;
                target_pos = servo->START_ANGLE_POSITION - (int)(range)*factor;
            }
            else if (curve > 0.0)
            {
                factor = curve;
                factor_outer = factor * OUTER_CURVE_FACTOR;
                target_pos = servo->START_ANGLE_POSITION + (int)(range)*factor;
            }
            else { // 0.0
                factor = 0.0;
                factor_outer = factor * OUTER_CURVE_FACTOR;
                target_pos = servo->START_ANGLE_POSITION + (int)(range)*factor;

            }
            break;
        case ServoPosition::HL:
        case ServoPosition::HR:
            if (curve < 0.0)
            {
                factor = -curve;
                target_pos = servo->START_ANGLE_POSITION + (int)(range)*factor;
            }
            else if (curve > 0.0)
            {
                factor = curve;
                target_pos = servo->START_ANGLE_POSITION - (int)(range)*factor;
            }
            else { // 0.0
                factor = 0.0;
                factor_outer = factor * OUTER_CURVE_FACTOR;
                target_pos = servo->START_ANGLE_POSITION + (int)(range)*factor;

            }
            break;
        }

        // Setze Geschwindigkeit (optional – falls relevant)
        dxl->setGoalVelocity(servo->id, speed);

        // Sende Zielposition
        dxl->setGoalPosition(servo->id, target_pos, UNIT_RAW);
        switch (servo->id)
        {
        case 7: // VR
        {
            // DEBUG_SERIAL.printf(" |VR (%4d, %3d, %3d - %2.1f)", target_pos, servo->CW_ANGLE_LIMIT, servo->CCW_ANGLE_LIMIT, factor);
            break;
        }
        case 8: // VL
        {
            // DEBUG_SERIAL.printf(" |VL (%4d, %3d, %3d - %2.1f)", target_pos, servo->CW_ANGLE_LIMIT, servo->CCW_ANGLE_LIMIT, factor);
            break;
        }
        case 17: // HL
        {
            // DEBUG_SERIAL.printf(" |HL (%4d, %3d, %3d - %2.1f)", target_pos, servo->CW_ANGLE_LIMIT, servo->CCW_ANGLE_LIMIT, factor);
            break;
        }
        case 18: // HR
        {
            // DEBUG_SERIAL.printf(" |HR (%4d, %3d, %3d - %2.1f)", target_pos, servo->CW_ANGLE_LIMIT, servo->CCW_ANGLE_LIMIT, factor);
            break;
        }
        }
    }
    // DEBUG_SERIAL.println();
    // DEBUG_SERIAL.println("**********************************");
}

/**
 * @brief Rover setzt die Antriebseinheiten, so das man seitwärts fahren kann.
 * (getestet funktioniert)
 *
 * @param dxl Dynamixel2Arduino Objekt
 * @param speed Steuert die Geschwindigkeit der Drehung, default=500
 *
 */
void setServosToSidway(Dynamixel2Arduino *dxl, int speed = 500)
{
    for (auto &servo : steeringServos)
    {
        dxl->setGoalVelocity(servo->id, speed, UNIT_RAW);

        switch (servo->servo_position)
        {
        case ServoPosition::VL:
        case ServoPosition::HR:
            dxl->setGoalPosition(servo->id, servo->CCW_ANGLE_LIMIT, UNIT_RAW);
            break;
        case ServoPosition::VR:
        case ServoPosition::HL:
            dxl->setGoalPosition(servo->id, servo->CW_ANGLE_LIMIT, UNIT_RAW);
            break;
        }
        delay(10);
    }
}

#endif