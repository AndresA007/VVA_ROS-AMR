# Script para probar la lectura de los ticks desde el encoder de hall effect de los motores
# CQrobot.

from gpiozero import DigitalInputDevice
from gpiozero import Motor
from time import sleep

ticks_goal = 2000
my_speed = 0.1

ticks_A = 0
ticks_B = 0
parar = False

def activado_pin_A():
  #print('tick en A')
  global parar
  global ticks_A
  ticks_A += 1
  if ticks_A >= ticks_goal:
      parar = True

def activado_pin_B():
  #print('tick en B')
  global ticks_B
  ticks_B += 1

encoder_pin_A = DigitalInputDevice(11, pull_up=False)
encoder_pin_B = DigitalInputDevice(26, pull_up=False)

encoder_pin_A.when_activated = activado_pin_A
encoder_pin_B.when_activated = activado_pin_B

right_front_motor = Motor(forward=21, backward=20)
right_back_motor = Motor(forward=16, backward=12)
left_back_motor = Motor(forward=5, backward=6)
left_front_motor = Motor(forward=13, backward=19)

left_back_motor.forward(my_speed)

detenido = False
while not detenido:
    if parar:
      left_back_motor.stop()
      detenido = True
      
    # Se coloca para que el ciclo no consuma todos los recursos de CPU
    # No colocar valores menores a 0.1 o la RbPi sufre retardos a la hora de detener las ruedas
    sleep(0.2)

print("\nTicks pin A: %d" % (ticks_A))
print("\nTicks pin B: %d" % (ticks_B))

# Esperar que la rueda deje de girar por la inercia
sleep(2)

print("\nTicks incluyendo los de inercia, pin A: %d" % (ticks_A))
print("\nTicks incluyendo los de inercia, pin B: %d" % (ticks_B))


