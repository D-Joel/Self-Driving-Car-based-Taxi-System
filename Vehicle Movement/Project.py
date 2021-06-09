import RPi.GPIO as GPIO
import time
import wiringpi
import sys
import time
import sys, tty, termios, time
import pyrebase
from __main__ import lg
import math
from navigation import * 
import gpsdData as GPS	
import GPS_log
 
#GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
 
#set GPIO Pins
GPIO_TRIGGER = 2
GPIO_ECHO = 3
 
#set GPIO direction (IN / OUT)
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)

wiringpi.wiringPiSetup()

wiringpi.pinMode(23, 1)
wiringpi.pinMode(22, 1) 
wiringpi.pinMode(24, 1)
wiringpi.pinMode(25, 1)

def forward():
    
    wiringpi.digitalWrite(22, 1)
    wiringpi.digitalWrite(23, 0)
    wiringpi.digitalWrite(24, 0)
    wiringpi.digitalWrite(25, 0)
    print("going forward")
    
def stop():
    
    wiringpi.digitalWrite(23, 0)
    wiringpi.digitalWrite(22, 1)
    wiringpi.digitalWrite(24, 1)
    wiringpi.digitalWrite(25, 0)
    print("going stop")
    
def right():
    wiringpi.digitalWrite(23, 1)
    wiringpi.digitalWrite(22, 0)
    wiringpi.digitalWrite(24, 0)
    wiringpi.digitalWrite(25, 0)
    print("going right")

def left():
    wiringpi.digitalWrite(23, 1)
    wiringpi.digitalWrite(22, 1)
    wiringpi.digitalWrite(24, 0)
    wiringpi.digitalWrite(25, 0)
    print("going left")

config = {
  "apiKey": "AIzaSyC8TJsU32BKBGNsSTWzoiywP0KRby73DWw",
  "authDomain": "smart-car-system-595b6-default-rtdb.firebaseapp.com",
  "databaseURL": "https://smart-car-system-595b6-default-rtdb.firebaseio.com/",
  "storageBucket": "smart-car-system-595b6-default-rtdb.appspot.com"
}

firebase = pyrebase.initialize_app(config)
db = firebase.database()

def comp_heading(GPS_from, GPS_goingto):
    "calculate heading, based on compass"
    
    # latitude = y-coord. longitude = x-coord.
    # coord = [x-coord, y-coord] in the reference frame of the car -> current_position: origin [0, 0] 
    coord = [0., 0.]
    coord[0] = GPS_goingto[1] - GPS_from[1]
    coord[1] = GPS_goingto[0] - GPS_from[0]

    phi = math.atan2(coord[1], coord[0])

    return phi*180/math.pi  


def auth():
	otp_in = input('Enter OTP')
	if otp_in == str(db.child('otp').get().val()) :
		print('Started')
		forward()
	else:
		print('wrong otp')
		auth()

auth()
	
import cv2 


stop_cascade = cv2.CascadeClassifier(r'/home/pi/Desktop/stop_sign_pjy.xml')
traffic_cascade = cv2.CascadeClassifier(r'/home/pi/Desktop/TrafficLight_HAAR_16Stages.xml')

cap = cv2.VideoCapture(0)

def stop_sign():
    while 1:
        ret, img = cap.read()
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        stopsigns = stop_cascade.detectMultiScale(gray, 1.3, 5)

        for (x,y,w,h) in stopsigns:
            cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
            roi_gray = gray[y:y+h, x:x+w]
            roi_color = img[y:y+h, x:x+w]
            print('found stop sign')
            return 1
        traffics = traffic_cascade.detectMultiScale(gray, 1.3, 5)
        for (ex,ey,ew,eh) in traffics:
            cv2.rectangle(img,(ex,ey),(ex+ew,ey+eh),(255,0,0),2)
            roi_gray = gray[ey:ey+eh, ex:ex+ew]
            roi_color = img[ey:ey+eh, ex:ex+ew]
            print('found traffic light')
            return 1
        

        cv2.imshow('img',img)
        k = cv2.waitKey(30) & 0xff
        if k == 27:
            break
        break
    #cap.release()
    #cv2.destroyAllWindows()
    return 0


def distance():
    # set Trigger to HIGH
    GPIO.output(GPIO_TRIGGER, True)
 
    # set Trigger after 0.01ms to LOW
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)
 
    StartTime = time.time()
    StopTime = time.time()
 
    # save StartTime
    while GPIO.input(GPIO_ECHO) == 0:
        StartTime = time.time()
 
    # save time of arrival
    while GPIO.input(GPIO_ECHO) == 1:
        StopTime = time.time()
 
    # time difference between start and arrival
    TimeElapsed = StopTime - StartTime
    # multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    distance = (TimeElapsed * 34300) / 2
 
    return distance

def command():
    dist = distance()
    #time.sleep(1)
    if (dist <=50):
        print("obstacle detected")
        
        stop()
        time.sleep(3)
        
        right()
        time.sleep(0.5)
        
        forward()
        time.sleep(0.6)
        
        left()
        time.sleep(0.42)
        
        stop()
        time.sleep(0.8)
        
        forward()
        time.sleep(1.3)
        
        left()
        time.sleep(0.4)
        
        forward()
        time.sleep(0.5)
        
        right()
        time.sleep(0.44)
        
        forward()

    else:
        forward()


if __name__ == '__main__':
        stop()
        while 1:
            if stop_sign() == 1:
                stop()
            
            else:
                command()
            time.sleep(0.5)
    

