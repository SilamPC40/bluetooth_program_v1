## Francesca Bonetta-Misteli, fbonetta-misteli@wustl.edu
## Script to run connect to PC device via bluetooth connection.
## This file is for official trials. This is the most up-to-date version.
## Last edited: August 30, 2023

from time import sleep, time
from speckle_sensor_bt import speckle_sensor_bt

ss = speckle_sensor_bt()
ss.start_server() # this is the pairing of the android to the pi
ss.devicePrep() # commented out becuase this is functionality that runs in response to a command from the android, which is all within the start listening code
#ss.calibrate_system()
ss.start_listening() 