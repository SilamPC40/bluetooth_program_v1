## IR sensor class: Initializes and has functions for reading IR contact sensor status (GPIO).
##
## Francesca Bonetta-Misteli, fbonetta-misteli@wustl.edu
## August 14, 2023

import RPi.GPIO as GPIO

class ir_sensor:
    def __init__(self):
        self.IR_pin = 26
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.IR_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        
    def read(self):
        return GPIO.input(self.IR_pin)