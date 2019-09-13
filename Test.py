import serial
import time
import RPi.GPIO as GPIO
import picamera
from picamera import PiCamera
from picamera.array import PiRGBArray
import numpy
import cv2
value=0

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
 
Motor1A = 33
Motor1B = 35
Motor1E = 37
Motor2A = 36
Motor2B = 38
Motor2E = 40

GPIO.setup(Motor1A,GPIO.OUT)
GPIO.setup(Motor1B,GPIO.OUT)
GPIO.setup(Motor1E,GPIO.OUT)
GPIO.setup(Motor2A,GPIO.OUT)
GPIO.setup(Motor2B,GPIO.OUT)
GPIO.setup(Motor2E,GPIO.OUT)
left = GPIO.PWM(Motor1E, 1000)
right = GPIO.PWM(Motor2E, 1000)

left.start(25)
right.start(25)
 

s = serial.Serial("/dev/ttyUSB0",9600)
#s.close()
#s.open()
cap=PiCamera()
rc=PiRGBArray(cap,(320,240))
cap.framerate=8
while True:
    t = s.read(1)
    val = str(t).strip("b'").strip("\\n").strip("\\r")
    #print(t)
    print(val)
    if (val=='S'):

        print("Turning motor on")
        left.start(25)
        right.start(25)
        GPIO.output(Motor1A,GPIO.LOW)
        GPIO.output(Motor1B,GPIO.HIGH)
        left.ChangeDutyCycle(100)
        GPIO.output(Motor2A,GPIO.LOW)
        GPIO.output(Motor2B,GPIO.HIGH)
        right.ChangeDutyCycle(100)

        #time.sleep(1)

        

       
        
        for image in cap.capture_continuous(rc, format="bgr", use_video_port=True, resize=(320,240)):
            frame = image.array
            cv2.imshow('frame',frame)
            cv2.waitKey(10)
            rc.truncate(0)
            value+=1
            if (value>20):
                val="A"
                left.stop()
                right.stop()
                print("Stopping motor")
                GPIO.output(Motor1E,GPIO.LOW)
                GPIO.output(Motor2E,GPIO.LOW)
                #GPIO.cleanup()
                break
            
        
    
    #s.close()
#GPIO.cleanup()