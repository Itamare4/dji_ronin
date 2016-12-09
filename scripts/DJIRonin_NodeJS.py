#!/usr/bin/env python
import rospy
from Naked.toolshed.shell import execute_js
import subprocess, signal
import os

__author__ = 'Itamar Eliakim'

class DJIRonin_NodeJS():
    def __init__(self):
        rospy.init_node("DJI_Ronin_NodeJS")
        rospy.loginfo("Starting DJI Ronin NodeJS Server")
        #self.startApp = subprocess.Popen("adb shell am start -n com.dji.gimbal/com.dji.gimbal.ui.HomeActivity", shell=True, stdout=subprocess.PIPE).stdout.read()
        self.getROSpath = subprocess.Popen("echo $ROS_PACKAGE_PATH", shell=True, stdout=subprocess.PIPE).stdout.read().split(':')[0]
        self.success = execute_js(self.getROSpath + '/dji_ronin/scripts/sji-android-screen-capture/bin/asc.js')
        if self.success==False:
            self.killserver()
        else:
            rospy.loginfo("ROS DJI Ronin NodeJS Server is ON")
        ### If doesnt work - ps aux | grep node -> kill -9 proccessid
        rospy.spin()


    def killserver(self):
        rospy.loginfo("Kill old server!")
        p = subprocess.Popen(['ps', '-A'], stdout=subprocess.PIPE)
        out, err = p.communicate()

        for line in out.splitlines():
            if 'node' in line:
                pid = int(line.split(None, 1)[0])
                os.kill(pid, signal.SIGKILL)
        self.success = execute_js(os.getcwd() + '/sji-android-screen-capture/bin/asc.js')
        rospy.loginfo("ROS DJI Ronin NodeJS Server is ON")
if __name__ == "__main__":
    DJIRonin_NodeJS()

