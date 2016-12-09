#!/usr/bin/env python
import rospy
from dji_ronin.msg import gimbalangle
import serial
from dji_ronin.srv import controlgimbal_srv
import time

__author__ = 'Itamar Eliakim'

class DJIRoninControl():
    def __init__(self):
        rospy.init_node("DJI_Ronin_Control")
        self.port = rospy.get_param("~gimbal_COM","/dev/serial/by-id/usb-Arduino_Srl_Arduino_Uno_85439303033351E051D0-if00")
        self.ser = serial.Serial(self.port, 9600)
        rospy.sleep(2)
        self.time = 0
        self.pan = 0
        self.tilt = 0
        self.roll = 0
        self.tolerance = 5
        rospy.Subscriber('/GimbalAngle', gimbalangle, self.Handle_Gimbal)
        self.s = rospy.Service('/DJI_RotateTo', controlgimbal_srv, self.Rotate_To)
        rospy.loginfo("Starting DJI Ronin Control Server")
        self.Boot(20)
        rospy.spin()

    def Handle_Gimbal(self,data):
        self.time = data.header.stamp
        self.pan = data.pan
        self.tilt = data.tilt
        self.roll = data.roll

    def Rotate_To(self,msg):
        ## Such a slow system, PID Controller won't give us better results.
        if msg.dir==1:
            toAngle = (-1)*msg.angle
        if msg.dir==2:
            toAngle = (1)*msg.angle
        for j in range(0,10):
            while abs((toAngle) - (self.pan)) > self.tolerance:          ##Error Calc or timeout
                #print abs(toAngle - self.pan)
                if toAngle>self.pan:
                    speed = -7                                           ##Values are set by the calibration process
                else:
                    speed = 15                                           ##Values are set by the calibration process

                #print speed
                self.ser.write(str(speed) +"\n\r")
            rospy.sleep(0.5)
        rospy.loginfo("Done Rotating")
        return True

    def Boot(self,speed):
        for i in range(0,8):
            self.ser.write(str(speed) + "\n\r")
            time.sleep(0.5)

if __name__ == "__main__":
    DJIRoninControl()