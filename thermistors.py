## Temperature sensor class: Initializes and has functions for reading and saving dual thermistor data
## from an ADC chip (SPI).
##
## Francesca Bonetta-Misteli, fbonetta-misteli@wustl.edu
## August 14, 2023

import spidev
from math import log

class thermistors:
    def __init__(self):
        self.spi = spidev.SpiDev()
        self.spi.open(0, 0)   
        self.spi.mode = 0
        self.spi.threewire = False
        self.spi.loop = False
        self.spi.max_speed_hz = 100000
        self.A = .001
        self.B = 2.39* 10**-4
        self.C = 1.56 * 10**-7
        
    def read(self):
        r = self.spi.xfer2([0b10010110, 0, 0])
        data = (r[1] << 8) + r[2]
        data = data/32767
        try:
            res = ((1-data)/data)*7400
            amb_temp = (1/(self.A + self.B*log(res) + self.C*log(res)**3))-273.15-16
        except:
            amb_temp = 0
        
        r = self.spi.xfer2([0b11010110, 0, 0])
        data = (r[1] << 8) + r[2]
        data = data/32767
        try:
            res = ((1-data)/data)*7400
            body_temp = (1/(self.A + self.B*log(res) + self.C*log(res)**3))-273.15-16
        except:
            body_temp = 0
        
        return (amb_temp, body_temp)