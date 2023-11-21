## Laser control class: Initializes and controls laser function (GPIO).
##
## Francesca Bonetta-Misteli, fbonetta-misteli@wustl.edu
## August 14, 2023

import RPi.GPIO as GPIO

class laser_ctrl:
    def __init__(self):
        self.laser_pin = 25
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.laser_pin, GPIO.OUT)
        
    def laser_on(self):
        GPIO.output(self.laser_pin, GPIO.HIGH)
        
    def laser_off(self):
        GPIO.output(self.laser_pin, GPIO.LOW)