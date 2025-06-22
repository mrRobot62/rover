#from src.rover.hardware.i2c_driver import ESP32Driver, ServoDriver, SERVOCOMMANDS, ESP32COMMANDS
from .i2c_driver import ESP32RawDriver, ServoDriver, CommandID, SubCommandID, dataModulo
from enum import Enum
from typing import List


IDX_AXES_SPEED = 1

class RoverDriver():
    """ 
        @brief RoverServoDriver
        Steuert den Rover über I2C in Geschwindigkeit (velocity) und Lenkung (Steering)
        
    """
    def __init__(self, logger):
        """
        Konstruktor, übergeben wird ein ROS2-Logger
        """
        self.logger = logger
        self.servo_driver = ServoDriver(self.logger)
        self.esp32_driver = ESP32RawDriver(self.logger)

    def set_led(self, pin, position=0, state=0):
        """
        setzt den Status eines LED-Pins

        @param pin = GPIO
        @position = postion im Array (0 oder 1), 0=default
        @state = 0 = off, 1=on, 0=default (off)
        """
        position = 0 if position != 1 else 1
        state = 0 if state != 1 else 1

        pin_list = [0,0]
        state_list = [0,0]
        pin_list[position] = pin
        state_list[position] = state

        self.digitalWrite(pin_list,state_list)


    def digitalWrite(self, pins: List[int]=[0,0], states:List[int]=[0,0]):
        """ 
        Sendet für pins den den jeweiligen state

        @return 0=0k, 1=fehler
        """
        self.logger.info(self.__msg(f"digitalWrite({pins}, {states})"))
        return self.esp32_driver.digitalWrite(pins, states)


    def set_velocity(self, value: float):
        """
        Steuerbefehl für die Drehgeschindigkeit der Antriebsservos
        """
        if value < -1.0:
            value = -1.0
        if value > 1.0:
            value = 1.0

        cmd = CommandID.SERVO_WRITE
        scmd = SubCommandID.SCMD_SERVO_POSITION
        data = [value, 0, 0,0,0]
        self.logger.info(self.__msg(f"set_velocity(CMD:{cmd}, SCMD:{scmd}, {data}) "))

        return self.servo_driver.write(cmd=cmd, scmd=scmd,servo_data=data)

    def set_steering(self, value: float):
        """
        Steuerbefehl für den Drehwinkel der Lenk-Servos
        """
        if value < -1.0:
            value = -1.0
        if value > 1.0:
            value = 1.0

        cmd = CommandID.SERVO_WRITE
        scmd = SubCommandID.SCMD_SERVO_POSITION
        data = [0, value, 0,0,0]
        self.logger.info(self.__msg(f"set_steering(CMD:{cmd}, SCMD:{scmd}, {data}) "))
        return self.servo_driver.write(cmd=cmd, scmd=scmd,servo_data=data)
    
    def set_steeringAndVelocity(self, steering: float, velocity: float, reverse_steering=False, reverse_velocity=False):
        """
        Steuerbefehl zum gleichzeigen setzen von Geschwindigkeit und Lenkung

        @param reverse_steering Default False, wder steering-Wert wird mit *-1 multipliziert um die Drehrichtung zu ändern
        @param reverse_velocity Default False, der vecolicty wert wird mit *-1 multipliziert um die Drehrichtung zu ändern
        """
        if steering < -1.0:
            steering = -1.0
        if steering > 1.0:
            steering = 1.0
        if velocity < -1.0:
            velocity = -1.0
        if velocity > 1.0:
            velocity = 1.0

        if reverse_steering:
            steering *= -1
        if reverse_velocity:
            velocity *= -1

        cmd = CommandID.SERVO_WRITE
        scmd = SubCommandID.SCMD_SERVO_SPEED_POSITION
        data = [velocity, steering, 0,0,0]
        self.logger.info(self.__msg(f"set_steeringAndVelocity(CMD:{cmd.value}, SCMD:{scmd.value}, {data}) "))
        return self.servo_driver.write(cmd=cmd, scmd=scmd,servo_data=data)
 
    def __msg(self, txt, prefix='[RoverDriver]', suffix='', line_break=True):
        """
        erzeugt eine log textzeile und gibt diese zurück
        """
        if line_break:
            line_break = '\n'
        else:
            line_break = ''
            txt = f"{line_break}{prefix} {txt} {suffix}"
        return txt