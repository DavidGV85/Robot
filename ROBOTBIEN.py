#Aqui poneis el Docstring que querais
def MoveMotor(argument1):
    """
    
     Descripcion de  los movimientos del motor:
    
    Args:
      controlIn (int):Cualquier PIN GPIO de entrada activa el movimiento del motor.
      motorOut (int):Cualquier pin de salida del motor se utiliza para enviar la señal PWM que va a controlar el motor.
    
     
    Returns:
      Esta funcion solo sirve para devolver el robot a su posicion inicial """
#import Wire
#import Adafruit_PWMServoDriver
import board
import busio
import Jetson.GPIO as GPIO
import adafruit_pca9685
import time
i2c = busio.I2C(board.SCL,board.SDA)
from adafruit_servokit import Servokit

#Declaro variables globales
MIN_PULSE_WIDTH  =  650
MAX_PULSE_WIDTH  =  2350
FREQUENCY        =  50


#Instancio el Driver del controlador de servos
#Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
pwm = adafruit_pca9685.PCA9685(i2c)
kit = ServoKit(channels=16)
potWrist = GPIO.input(13)
potElbow = GPIO.input (15)                  #//Assign Potentiometers to pins on Arduino Uno
potShoulder = GPIO.input(19)
potBase = GPIO.input(21)

#Configuro el SetUP
time.sleep(5)                         #<---  So I have time to get controller to starting position
pwm.frequency = FREQUENCY

GPIO.setmode(GPIO.BOARD)
hand = adafruit_motor.servo.Servo(0)
wrist = adafruit_motor.servo.Servo(1)
elbow = adafruit_moptor.servo.Servo(2)
shoulder = adafruit_motor.servo.Servo(3)
base = adafruit_motor.servo.Serv0(4)
potWrist = adafruit_motor.servo.Servo(5)
potElbow = adafruit_motor.servo.Servo(6)
potShoulder = adafruit_motor.servo.Servo(7)
potBase = adafruit_moto.servo.Servo(8)


pwm.setPWMFreq(FREQUENCY)
pwm.setPWM(X, 0, 90)                  #Set Gripper to 90 degrees (Close Gripper) X en Jetson
pwm.begin()
GPIO.setup(11, GPIO.IN)    # channel tiene que ser un pin válido en jetson


def moveMotor(controlIn, motorOut):
  pulse_wide, pulse_width, potVal = -7
  
#  potVal = analogRead(controlIn);                                                   #//Read value of Potentiometer
  potVal = GPIO.input(controlIN)
  pulse_wide = map(potVal, 800, 240, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH)
  pulse_width = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096);                #//Map Potentiometer position to Motor
  
#  pwm.setPWM(motorOut, 0, pulse_width);
  pwm = GPIO.PWM(motorOut, 0, pulse_width)
 
 
while (True):  
  moveMotor(potWrist, wrist)
  moveMotor(potElbow, elbow)                              # //Assign Motors to corresponding Potentiometers
  moveMotor(potShoulder, shoulder)
  moveMotor(potBase, base)
  pushButton = GPIO.input(11)      
  if(pushButton == GPIO.LOW):

    pwm.setPWM(hand, 0, 180);                             #//Keep Gripper closed when button is not pressed
    print("Grab")
  
  else:
  
    pwm.setPWM(hand, 0, 90);                              #//Open Gripper when button is pressed
    print("Release")
    
GPIO.cleanup()
setup()

pinMode(13,INPUT_PULLUP)

potWrist = A3
potElbow = A2                        #//Assign Potentiometers to pins on Arduino Uno
potShoulder = A1
potBase = A0

hand = 11
wrist = 12
elbow = 13                         #//Assign Motors to pins on Servo Driver Board
shoulder = 14
base = 15