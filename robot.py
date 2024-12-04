#Aqui poneis el Docstring que querais

#import Wire
#import Adafruit_PWMServoDriver
import board 
import busio 
import Jetson.GPIO as GPIO
import adafruit_pca9685 
import time 
i2c = busio.I2C(board.SCL, board.SDA)
from adafruit_servokit import ServoKit


#Declaro variables globales
MIN_PULSE_WIDTH = 650
MAX_PULSE_WIDTH = 2350
FREQUENCY = 50

#Instancio el Driver del controlador de servos
#Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver()  esta queda asi selecionada y la linea de abajo es su traduccion a pyton
pwm = adafruit_pca9685.PCA9685(i2c)
kit = ServoKit(channels=16)

#Configuro el SetUp
time.sleep(5)     #So i have time to get controller to starter position
pwm.frequency = FREQUENCY
GPIO.setmode(GPIO.BOARD)
hand = pmw.channels[0]
hand = adafruit_motor.servo.Servo(0)  #cualquiera de las dos, mejor la de abajo
wrist = adafruit_motor.servo.Servo(1)
elbow = adafruit_motor.servo.Servo(2)
shoulder = adafruit_motor.servo.Servo(3) 
base = adafruit_motor.servo.Servo(4)
potWrist = adafruit_motor.servo.Servo(5)
potElbow = adafruit_motor.servo.Servo(6)
potShoulder = adafruit_motor.servo.Servo(7)
potBase = adafruit_motor.servo.Servo(8)

pwm.setPWMFreq(FREQUENCY)
pwm.setPWM(32, 0, 90)             #Set Gripper to 90 degrees (Close Gripper)
pwm.begin()
GPIO.setup(7, GPIO.IN)     # Channel tiene que ser un pin valido para en Jetson

def MoveMotor(controlIn, motorOut):
    pulse_wide, pulse_width, potVal = -7

    #potVal = analogRead(controlIn);  #lectura en C
    potVal = GPIO.input(controlIn)
    pulse_wide = map(potVal, 800, 240, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH)
    pulse_width = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096)
    #pwm.setPWM(motorOut, 0, pulse_width);     #lectura en C
    pwm = GPIO.PWM(motorOut, 0, pulse_width)

While(True):
    moveMotor(potWrist, wrist)
    moveMotor(potElbow, elbow)
    moveMotor(potShoulder, shoulder)
    moveMotor(potBase, base)
    pushButton = GPIO.input(7)
    if(pushButton == GPIO.LOW):
        pwm.setPWM(hand, 0, 180)               
        print("Grab")
    else:
        pwm.setPWM(hand, 0, 90)
        print("Release")

GPIO.cleanup()