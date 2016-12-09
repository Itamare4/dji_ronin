#!/usr/bin/env python
from PIL import Image, ImageFont, ImageDraw, ImageOps
import cv2
import urllib
import numpy as np
import rospy
from sensor_msgs.msg import Image as ROSIMAGE
import os
from matplotlib import pyplot as plt
from dji_ronin.msg import gimbalangle
import tf
from cv_bridge import CvBridge
import subprocess

__author__ = 'Itamar Eliakim'

class DJIRoninClassifier():
    def __init__(self):
        rospy.init_node("DJI_Ronin_App")
        self.pub = rospy.Publisher('/GimbalAngle', gimbalangle, queue_size=10)
        self.app_pub = rospy.Publisher("/DJI_Ronin_App", ROSIMAGE, queue_size=10)
        self.digit_pub = rospy.Publisher("/DJI_Ronin_App_Digits", ROSIMAGE, queue_size=10)
        self.url = rospy.get_param("~Phone_URL","http://localhost:3000/capture?device=9885e6305357475141%40localhost%3A5037&accessKey=&size=320x640&fastCapture=false&fastResize=true&orientation=")
        self.digitheight = 14
        self.bool_publish_Digit = rospy.get_param("~Publish_Digits",0)
        self.bool_publish_App = rospy.get_param("~Publish_App", 0)
        self.getROSpath = subprocess.Popen("echo $ROS_PACKAGE_PATH", shell=True, stdout=subprocess.PIPE).stdout.read().split(':')[0]
        self.model = self.createDigitsModel(self.getROSpath + "/dji_ronin/scripts/open-sans.light.ttf", self.digitheight)
        rospy.loginfo("######################################")
        rospy.loginfo("##      DJI Ronin Server is ON      ##")
        rospy.loginfo("#Update URL in launch set to 360x640 #")
        rospy.loginfo("######################################")

        #rospy.loginfo("Starting DJI Ronin App")
        self.lastyaw = 0
        self.panangle = 0
        #self.updateangle()
        try:
            rospy.sleep(5)                                                              ##Let the server load
            self.runClassifier(self.url)
            rospy.spin()
        except:
            rospy.loginfo("[DJI Ronin] - Phone is not connected, Restart Server.")


    def updateangle(self):
        br = tf.TransformBroadcaster()
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            br.sendTransform((0, 0, 0), tf.transformations.quaternion_from_euler(0, 0, np.deg2rad(self.panangle)),rospy.Time.now(), "base_link_ronin", "base_link")
            rate.sleep()

    def createDigitsModel(self,fontfile, digitheight):
        ttfont = ImageFont.truetype(fontfile, digitheight)
        samples = np.empty((0, digitheight * (digitheight / 2)))
        responses = []
        for n in range(10):
            pil_im = Image.new("RGB", (digitheight, digitheight * 2))
            ImageDraw.Draw(pil_im).text((0, 0), str(n), font=ttfont)
            pil_im = pil_im.crop(pil_im.getbbox())
            pil_im = ImageOps.invert(pil_im)
            #pil_im.save(str(n) + ".png")

            # convert to cv image
            cv_image = cv2.cvtColor(np.array(pil_im), cv2.COLOR_RGBA2BGRA)
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            blur = cv2.GaussianBlur(gray, (5, 5), 0)
            thresh = cv2.adaptiveThreshold(blur, 255, 1, 1, 11, 2)

            roi = cv2.resize(thresh, (digitheight, digitheight / 2))
            responses.append(n)
            sample = roi.reshape((1, digitheight * (digitheight / 2)))
            samples = np.append(samples, sample, 0)


        samples = np.array(samples, np.float32)
        responses = np.array(responses, np.float32)

        model = cv2.KNearest()
        model.train(samples, responses)
        return model


    def findDigits(self,im, digitheight, lower, upper):
        im = im[450+lower:450+upper, 200:270]
        out = np.zeros(im.shape, np.uint8)
        gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
        thresh = cv2.adaptiveThreshold(gray, 255, 1, 1, 11, 15)
        contours, hierarchy = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        stringlist = []
        sign=1

        #contours1, hierarchy1 = cv2.findContours(thresh.copy(),cv2.cv.CV_RETR_LIST,cv2.cv.CV_CHAIN_APPROX_SIMPLE)

        centers = []
        radii = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 2:
                continue
            br = cv2.boundingRect(contour)
            radii.append(br[2])

            m = cv2.moments(contour)
            try:
                center = (int(m['m10'] / m['m00']), int(m['m01'] / m['m00']))
                centers.append(center)
            except:
                pass

        if len(centers)>1:
            center = max(centers)
        try:
            cv2.circle(out, center, 3, (255, 0, 0), -1)
        except:
            center = [50,1]

        for cnt in contours:
            x, y, w, h = cv2.boundingRect(cnt)
            if h > w and h > (digitheight * 4) / 5 and h < (digitheight * 6) / 5 and x<center[0]:
                cv2.rectangle(im, (x, y), (x + w, y + h), (0, 255, 0), 1)
                roi = thresh[y:y + h, x:x + w]  # crop
                roi = cv2.resize(roi, (digitheight, digitheight / 2))
                roi = roi.reshape((1, digitheight * (digitheight / 2)))
                roi = np.float32(roi)
                retval, results, neigh_resp, dists = self.model.find_nearest(roi, k=1)
                string = str(int((results[0][0])))
                #cv2.drawContours(out,[cnt],-1,(0,255,255),1)
                cv2.putText(out, string, (x-5, y + h), 0, 0.7, (0, 255, 0))
                stringlist.append(string)

            elif x < 40:
                approx = cv2.approxPolyDP(cnt, 0.01 * cv2.arcLength(cnt, True), True)
                if len(approx) <= 4:
                    cv2.drawContours(out, [cnt], 0, (0, 0, 255), -1)
                    sign = -1

        return stringlist, sign, im, thresh, out


    def runClassifier(self,url):
        stream=urllib.urlopen(url)
        anglist = ["Pan", "Tilt", "Roll"]
        bytes=''
        while True:
            bytes+=stream.read(1024)
            a = bytes.find('\xff\xd8')
            b = bytes.find('\xff\xd9')
            if a!=-1 and b!=-1:
                jpg = bytes[a:b+2]
                bytes= bytes[b+2:]
                i = cv2.imdecode(np.fromstring(jpg, dtype=np.uint8),cv2.CV_LOAD_IMAGE_COLOR)
                #cv2.imshow('DJI Ronin',i)

                angles = []
                implotlist = []
                for j in range(0, len(anglist)):
                                string, sign, im,thresh,out = self.findDigits(i, self.digitheight, 30*(j), 30*(j+1))
                                tempstring = ""
                                implotlist.extend([im,thresh,out])
                                for k in range(0, len(string)):
                                    tempstring += string[len(string) - 1 - k]
                                #print anglist[j] + " - " + tempstring
                                if abs(int(tempstring))>180:
                                    angles.append(sign * int(tempstring[0:2]))
                                else:
                                    angles.append(sign * int(tempstring))

                if self.bool_publish_App:
                                bridge = CvBridge()
                                msg = bridge.cv2_to_imgmsg(i,encoding="bgr8")
                                self.app_pub.publish(msg)

                if self.bool_publish_Digit:
                                name = os.getcwd() + "/DigitsRecognition.jpg"
                                plt.figure(1)
                                plt.imshow(i)
                                plt.figure(2)
                                for j in range(0,9):
                                    plt.subplot(3, 3, j+1);
                                    plt.imshow(implotlist[j], 'gray')
                                plt.savefig(name)
                                plt.show()

                                bridge = CvBridge()
                                pic = cv2.imread(name)
                                msg = bridge.cv2_to_imgmsg(pic, encoding="bgr8")
                                self.digit_pub.publish(msg)


                if abs(angles[0]-self.lastyaw)<50:                                              ##Remove False Readings..
                            msg = gimbalangle()
                            msg.header.stamp = rospy.get_rostime()    
                            msg.pan = self.panangle = angles[0]
                            msg.tilt = angles[1]
                            msg.roll = angles[2]

                            self.pub.publish(msg)

                self.lastyaw = self.panangle


if __name__ == "__main__":
    DJIRoninClassifier()
