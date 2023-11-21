## Accelerometer class: Defines accelerometer object and has associated functions for recording and
## saving data (I2C).
## 
## Francesca Bonetta-Misteli, fbonetta-misteli@wustl.edu
## August 14, 2023

import smbus

class accelerometer:
    def __init__(self):
        self.bus = smbus.SMBus(1)
        try:
            self.bus.write_byte_data(0x19, 0x20, 0x27)
            self.bus.write_byte_data(0x19, 0x23, 0x00)
        except:
            print('There is a accelerometer error.')
        
    def read(self):
        data0 = self.bus.read_byte_data(0x19, 0x28)
        data1 = self.bus.read_byte_data(0x19, 0x29)
        x_accl = data1*256 + data0
        if x_accl > 32767:
            x_accl -= 65536
                
        data0 = self.bus.read_byte_data(0x19, 0x2A)
        data1 = self.bus.read_byte_data(0x19, 0x2B)
        y_accl = data1*265 + data0
        if y_accl > 32767:
            y_accl -= 65536
                
        data0 = self.bus.read_byte_data(0x19, 0x2C)
        data1 = self.bus.read_byte_data(0x19, 0x2D)
        z_accl = data1*265 + data0
        if z_accl > 32767:
            z_accl -= 65536
            
        return (x_accl, y_accl, z_accl)