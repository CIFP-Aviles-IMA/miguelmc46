#Aqui poneis el Docstring que querais
"""
Este código está sacado de https://www.printables.com/model/818975-compact-robot-arm-arduino-3d-printed/files.

Este script está diseñado para controlar un brazo robótico utilizando servomotores y un controlador PWM PCA9685 
conectado a una placa Jetson. El brazo robótico consta de varios servomotores para mover las articulaciones del 
brazo (hombro, codo, muñeca, base y garra) y sensores de posición (potenciómetros) que permiten el control de 
estos servos en tiempo real. Además, se incluye un botón que, cuando se presiona, controla la apertura y cierre 
de la garra del brazo robótico.

Requisitos:
- Jetson.GPIO: Para el control de los pines GPIO en la placa Jetson.
- adafruit_pca9685: Para la comunicación con el controlador PWM PCA9685.
- adafruit_servokit: Para la gestión de servomotores a través de la librería Adafruit.
- time: Para introducir retrasos entre las acciones y configurar el sistema.

Funcionamiento:
- El script configura los servos mediante el controlador PWM PCA9685, que gestiona las señales PWM necesarias 
  para controlar la posición de cada motor.
- Los potenciómetros se leen a través de los pines GPIO y se mapean a un rango de valores que se usan para ajustar 
  el ángulo de los servos de las articulaciones (hombro, codo, muñeca, base).
- El valor del potenciometro se convierte en un ancho de pulso PWM para controlar el movimiento de los servos.
- Un botón conectado al pin GPIO 15 permite controlar la garra del brazo. Cuando el botón no está presionado, 
  la garra se cierra, y cuando está presionado, la garra se abre.
  
Funciones principales:
- `moveMotor(controlIn, motorOut)`: Lee el valor de un potenciometro conectado a un pin GPIO y ajusta el 
  movimiento del motor (servo) correspondiente según la posición del potenciometro.
- El script entra en un bucle infinito donde constantemente se ajustan las posiciones de los servos de acuerdo 
  a los valores de los potenciómetros y se controla la garra según el estado del botón.

Parámetros de configuración:
- `MIN_PULSE_WIDTH`: Ancho de pulso mínimo para el movimiento de los servos (650 microsegundos).
- `MAX_PULSE_WIDTH`: Ancho de pulso máximo para el movimiento de los servos (2350 microsegundos).
- `FREQUENCY`: Frecuencia de actualización de la señal PWM (50 Hz).
"""
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
pwm = adafruit_pca9685.PCA9685("i2C")
kit = ServoKit(channels=16)


#Configuro el SetUP
time.sleep(5)                         #<---  So I have time to get controller to starting position
pca.frequency = FREQUENCY
GPIO.setmode(GPIO.BOARD)
hand = adafruit_motor.servo.Servo(0)  #cualquiera de las dos, mejor la de abajo
wrist = adafruit_motor.servo.Servo(1)
elbow = adafruit_motor.servo.Servo(2)
shoulder = adafruit_motor.servo.Servo(3) 
base = adafruit_motor.servo.Servo(4)
potWrist = adafruit_motor.servo.Servo(5)
potElbow = adafruit_motor.servo.Servo(6)                      #//Assign Potentiometers to pins on Arduino Uno
potShoulder = adafruit_motor.servo.Servo(7)
potBase = adafruit_motor.servo.Servo(8)

pwm.setPWMFreq(FREQUENCY)
pwm.setPWM(32, 0, 90)                  #Set Gripper to 90 degrees (Close Gripper) X en Jetson
pwm.begin()
GPIO.setup(15, GPIO.IN)    # channel tiene que ser un pin válido en jetson


def moveMotor(controlIn, motorOut):
  """
    Descripción de las funciones relacionadas con la función def MoveMotor(controlIN, motorOUT):
     
    
    Args:
      controlIn (int):El pin GPIO donde se lee el valor del potenciometro el cual se obtiene mediante el pin seleccionado.
      motorOut (int):El pin de salida del motor.Este pin se utiliza para enviar la señal PWM al motor que se va a controlar.
    
     
    Returns:
      Esta función no devuelve ningún valor 
  """
  pulse_wide, pulse_width, potVal = -3
  
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
  pushButton = GPIO.input(15)      
  if(pushButton == GPIO.LOW):

    pwm.setPWM(hand, 0, 180);                             #//Keep Gripper closed when button is not pressed
    print("Grab")
  
  else:
  
    pwm.setPWM(hand, 0, 90);                              #//Open Gripper when button is pressed
    print("Release")

GPIO.cleanup()
