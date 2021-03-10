# Script para medir la cantidad de ticks reportados por el encoder y la cantidad de revoluciones
# (medidas manualmente) dadas por la rueda, para luego hacer la equivalencia de ticks y RPM o
# Radianes.

from gpiozero import DigitalInputDevice
from gpiozero import Motor
from time import sleep

ticks_A = 0
ticks_B = 0

def activado_pin_A():
  global ticks_A
  ticks_A += 1

def activado_pin_B()
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


# Set the motor to move, the speed and time
motor = left_back_motor
my_speed = 0.3
my_time = 10



motor.forward(my_speed)
sleep(my_time)
motor.stop()

# Esperar que la rueda deje de girar por la inercia
sleep(2)

print("\nTicks_A: %d, Ticks_B: %d" % (ticks_A, ticks_B))

