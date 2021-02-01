import RPi.GPIO as GPIO
import sys
from pad4pi import rpi_gpio
import time
from picamera import PiCamera
from enum import Enum
import os
import numpy

class State(Enum):
    LOCKED = 1
    UNLOCKED = 2
    VERIFYING = 3


def duty(angle):
    return angle /(10.0)+2.5

camera=PiCamera()
GPIO.setmode(GPIO.BCM)
GPIO.setup(25,GPIO.OUT)
GPIO.setup(22,GPIO.OUT)

GPIO.output(25, GPIO.HIGH)
import face_recognition

pwm = GPIO.PWM(22,100)
pwm.start(5)

#inital state
global state
state=State.LOCKED
pwm.ChangeDutyCycle(duty(100))

# Setup Keypad
KEYPAD = [
        ["1","2","3","A"],
        ["4","5","6","B"],
        ["7","8","9","C"],
        ["*","0","#","D"]
]

# same as calling: factory.create_4_by_4_keypad, still we put here fyi:
ROW_PINS = [6, 13, 19, 26] # BCM numbering
COL_PINS = [12, 16, 20, 21] # BCM numbering

factory = rpi_gpio.KeypadFactory()

# Try factory.create_4_by_3_keypad
# and factory.create_4_by_4_keypad for reasonable defaults
keypad = factory.create_keypad(keypad=KEYPAD, row_pins=ROW_PINS, col_pins=COL_PINS)

known_people = []
for filename in os.listdir("/home/pi/Documents/face unlock box/known_people/"):
    known_people.append(face_recognition.face_encodings(
        face_recognition.load_image_file("/home/pi/Documents/face unlock box/known_people/"+filename))[0])

def compareFaces():
    unknown_picture = face_recognition.load_image_file("unknown.jpg")
    face = face_recognition.face_encodings(unknown_picture)
    if len(face) != 0:
        unknown_face_encoding = face[0]
        for person in known_people:
            results = face_recognition.compare_faces([person], unknown_face_encoding)
            if results[0]:
                return True
    
    return False

GPIO.output(25, GPIO.LOW)
def printKey(key):
    global state
    print(key)
    if key == "C":
        if state == State.LOCKED:
            
            state = State.VERIFYING
            
            #turn on LED
            GPIO.output(25, GPIO.HIGH)
            
            #takes photo
            camera.capture('unknown.jpg')
            
            #compare image to correct faces
            approvedFace = compareFaces()
            
            GPIO.output(25, GPIO.LOW)
            if approvedFace:
                #turn off LED
                print("right")
                pwm.ChangeDutyCycle(duty(0))
                state = State.UNLOCKED
            else:
                print("wrong")
                state = State.LOCKED
                for i in range(5):
                    GPIO.output(25, GPIO.HIGH)
                    time.sleep(0.25)
                    GPIO.output(25, GPIO.LOW)
                    time.sleep(0.25)
    
    elif key == "*":
        if state == State.UNLOCKED:
            pwm.ChangeDutyCycle(duty(100))
            state = State.LOCKED
            
    elif key == "A":
        if state == State.UNLOCKED:
            new = len(known_people)
            camera.capture('/home/pi/Documents/face unlock box/known_people/face%d.jpg'%new)
            GPIO.output(25, GPIO.HIGH)
            face = face_recognition.face_encodings(
                face_recognition.load_image_file("/home/pi/Documents/face unlock box/known_people/face%d.jpg"%new))
            GPIO.output(25, GPIO.LOW)
            if len(face) == 0:
                os.remove("/home/pi/Documents/face unlock box/known_people/face%d.jpg"%new)
                for i in range(5):
                    GPIO.output(25, GPIO.HIGH)
                    time.sleep(0.25)
                    GPIO.output(25, GPIO.LOW)
                    time.sleep(0.25)
            else:
                known_people.append(face[0])
            
            


# printKey will be called each time a keypad button is pressed
keypad.registerKeyPressHandler(printKey)

try:
  while(True):
    time.sleep(0.2)
except:
 keypad.cleanup()