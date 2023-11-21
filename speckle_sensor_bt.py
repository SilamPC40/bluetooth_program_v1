## Francesca Bonetta-Misteli, fbonetta-misteli@wustl.edu
## Script to run connect to PC device via Bluetooth for healthy human, pregnant patient, and phlebotomy trials
## This file is for the official trials

# Define component classes
# from thermistors import thermistors as therm
from accelerometer import accelerometer as acc
from ir_sensor import ir_sensor as ir
from laser_ctrl import laser_ctrl as laser
from picamera import PiCamera, PiCameraError
import helper_funcs as hf
from bluetooth import * 
from time import sleep, time
import numpy as np
import math
from datetime import datetime
from subprocess import call
import subprocess as sp
import cv2
import RPi.GPIO as GPIO
import gain_setter as gain_set
import os
from threading import Thread
import logging
# to do - implement logging with the logger library in


## A class that does everything for the bluetooth program
class speckle_sensor_bt:
    
    ## Initialize class with camera parameters to stay consistent through recording
    def __init__(self):
        ## Does a command line call to set up the Pi for bluetooth
        cmd1 = "sudo hciconfig hci0 piscan"
        cmd2 = "sudo chmod o+rw /var/run/sdp"
        call([cmd1], shell=True)
        call([cmd2], shell=True)
        self.uuid = "94f39d29-7d6d-437d-973b-fba39e49d4ee" #only needed if tablet not hardcoded to specific pi
        self.noSetParams = True
        self.activeMode = True
        self._running = True
        

    ## Connect with client device and initialize camera
    def start_server(self):
        ## Initialize Bluetooth connection 
        try:
            self.server_sock = BluetoothSocket(RFCOMM)
            self.server_sock.bind(("", PORT_ANY))
            self.server_sock.listen(1)
            port = self.server_sock.getsockname()[1]
            advertise_service(self.server_sock, "BTServer",
                              service_id=self.uuid,
                              service_classes=[self.uuid, SERIAL_PORT_CLASS],
                              profiles=[SERIAL_PORT_PROFILE],
                              protocols=[OBEX_UUID])

            print("Waiting for connection on RFCOMM channel %d" % port)
            self.client_sock, self.client_info = self.server_sock.accept() ## socket error notice happens right here
            print("Accepted connection from ", self.client_info)
            errors = 0b00
            try:
                self.acc = acc()
            except:
                errors = errors + 1
#             self.therm = therm()
            self.ir = ir()
            self.laser = laser()
            try:
                self.camera = PiCamera()
            except:
                errors = errors + 2
            self.client_sock.sendall(str(errors))
            
        ## If the connection fails the program will run with no feedback
        except:
            #noBT.noBTatStartRecording()\
            print('error connecting')

    def devicePrep(self):
        # run this once device is on the wrist
        # first data is in response to user input on the client site "What do you want to name with recording session"
        data = self.client_sock.recv(1024)
        recording_session_name = data.decode("utf-8").strip().replace(' ','')
        self.masterDir = '/home/pi/Documents/capturedData/' + recording_session_name
        hf.make_folder(self.masterDir)
        
        # first data is client timestamp
        data = self.client_sock.recv(1024)
        client_timestamp = data.decode("utf-8").strip() 
        rpi_timestamp = datetime.now().strftime("%Y_%m_%d-%I_%M_%S_%f_%p")

        # add a folder for metadata within the recording session
        self.metadataDir = self.masterDir + '/metadata'
        log_filename = self.metadataDir+'log.log'
        hf.make_folder(self.metadataDir)
        # save initial client timestamp
        synch_timestamp_name = self.metadataDir + '/synch_timestamp.txt'
        synch_timestamp_text =  'rpi_ts='+f"{rpi_timestamp}," + 'client_ts='+f"{client_timestamp}"
        hf.write_to_file(synch_timestamp_text, synch_timestamp_name)
        
        # initialize data log into the metadata folder
        logging.basicConfig(filename=log_filename, level=logging.DEBUG, format='[%(asctime)s] %(levelname)s [%(name)s:%(lineno)s] %(message)s')
        self.log = logging.getLogger('root')
        self.log.debug('test log')
        self.log.debug('synchronization timestamp step:' +synch_timestamp_text)
        
#         while self.noSetParams:
#             print('in setup')
#           
#             
#             # next data is resolution mode
#             data = self.client_sock.recv(1024)
#             resModeData = data.decode("utf-8").strip()
#             self.resMode = int(resModeData)
#             
#             # next data is exposure
#             data = self.client_sock.recv(1024)
#             shutterSpeedData = data.decode("utf-8").strip()
#             self.expMode = int(shutterSpeedData)
        self.expMode = 1000
#             # convert input settings to actual camera settings for 
#             if self.resMode == 1:
#                 self.camera.resolution = (320,240)
#                 self.camera.framerate = 100
#             elif self.resMode == 2:
#                 self.camera.resolution = (640,480)
#                 self.camera.framerate = 80
#             else:
        self.camera.resolution = (1920,1080)
        self.camera.framerate = 30
                
        self.camera.shutter_speed = self.expMode # this is the input exposure mode
        sleep(2)
        self.camera.exposure_mode = 'off'
        self.camera.awb_mode = 'off'
        self.camera.awb_gains = (1)
        gain_set.set_analog_gain(self.camera, 1)
        gain_set.set_digital_gain(self.camera, 1)
            
        paramsStr = 'set_%dres_%dfps_%dexp' % (self.camera.resolution[0], self.camera.framerate, self.expMode)
        self.log.debug('Parameters for initial camera capture:'+paramsStr)
            
        self.paramsDir = self.masterDir + '/' + paramsStr
        hf.make_folder(self.paramsDir)
        self.testDir = self.paramsDir + '/tests'
        hf.make_folder(self.testDir)
        self.capDir = self.paramsDir + '/captures'
        hf.make_folder(self.capDir)
         
    def autoExposureAdjustment(self):
        image_output_per_shutter_speed = [] # store the values at each shutter speed in thi structure
        # step through  self.camera.shutter_speed values
        for i in range(1000, 5000, 1000): # ranging from 1000 to 5000 microseconds
            self.camera.shutter_speed = i
            sleep(1)
            self.laser.laser_on()
            vid_path = self.speckleProcessing_capture(self.testDir, 'preview', 0.5)
            self.laser.laser_off()
            cap = cv2.VideoCapture(vid_path)
            imgMean, imgStdev, imgMax = self.checkFramesSaturated(cap)
            
        # we have to decide what to do with this information-> one option is to store a data structure consisting of shutter speed followed by mean/std/max
        # then have a separate function that analyzes that data and output the 'desired' shutter speed.
        self.determine_optimal_shutterspeed(image_output_per_shutter_speed)
        self.client_sock.sendall('done')
        
    def checkFramesSaturated(self, cap):
        # check video frames for evidence of being saturated
        frame_means = []
        frame_maxes= []
        frame_stDevs = []
        my_str = '' 
        try:
            while True:
                ret, frame = cap.read();
                Iraw = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                meanVal, stdVal= cv2.meanStdDev(Iraw) # 4-tuple is standard output for meanStdDev even though it's a grayscale image, only first value of the 4-tuple is relevant, the rest are zero
                maxVal = Iraw.max()              
                frame_means.append(meanVal[0][0])
                frame_stDevs.append(stdVal[0][0])
                frame_maxes.append(maxVal)
        except:
            frames_meanVal = np.mean(frame_means)
            frames_stDevsVal = np.mean(frame_stDevs)
            frames_maxVal = np.mean(frame_maxes)
            self.log.debug('For shutter speed'+f"{self.camera.shutter_speed}" +', mean, stdev, and max values:'+(f"{frames_meanVal}" + ','+f"{frames_stDevsVal}"+','+f"{frames_maxVal}"))
            return frames_meanVal, frames_stDevsVal, frames_maxVal
        
    def processVideo(self, cap):
        res = self.camera.resolution
        h = res[1]
        w = res[0]
        my_str = ''
        hheight = int(h/4)
        hwidth = int(w/4)
        NxN = 7
        frame_means = []
        try:
            while True:
                ret, frame = cap.read()
                frame = frame[3*int(h/8):5*int(h/8), 3*int(w/8):5*int(w/8)]
                Iraw = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY).astype('f')/256
                Irawsq = Iraw**2
                Idiag = Iraw[7:hheight,7:hwidth]+Iraw[6:hheight-1,6:hwidth-1]+Iraw[5:hheight-2,5:hwidth-2]+Iraw[4:hheight-3,4:hwidth-3]+Iraw[3:hheight-4,3:hwidth-4]+Iraw[2:hheight-5,2:hwidth-5]+Iraw[1:hheight-6,1:hwidth-6]
                Idiagsq = Irawsq[7:hheight,7:hwidth]+Irawsq[6:hheight-1,6:hwidth-1]+Irawsq[5:hheight-2,5:hwidth-2]+Irawsq[4:hheight-3,4:hwidth-3]+Irawsq[3:hheight-4,3:hwidth-4]+Irawsq[2:hheight-5,2:hwidth-5]+Irawsq[1:hheight-6,1:hwidth-6]
                sc = np.mean(np.sqrt((NxN*Idiagsq-Idiag**2)/(NxN*(NxN-1)))/(Idiag/NxN))
                frame_means.append(sc)
        
        except:
            for j in frame_means:
                if not math.isnan(j):
                    val = int(round(j*65535))
                    my_str = my_str + "%04x" % val
            self.client_sock.sendall(my_str)


    def start_processing(self, cap):
        res = self.camera.resolution
        h = res[1]
        w = res[0]
        my_str = ''
        hheight = int(h/4)
        hwidth = int(w/4)
        NxN = 7
        frame_means = []
        print('here')
        
        try:
            while self._running:
                ret, frame = cap.read()
                frame = frame[3*int(h/8):5*int(h/8), 3*int(w/8):5*int(w/8)]
                Iraw = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY).astype('f')/256
                Irawsq = Iraw**2
                Idiag = Iraw[7:hheight,7:hwidth]+Iraw[6:hheight-1,6:hwidth-1]+Iraw[5:hheight-2,5:hwidth-2]+Iraw[4:hheight-3,4:hwidth-3]+Iraw[3:hheight-4,3:hwidth-4]+Iraw[2:hheight-5,2:hwidth-5]+Iraw[1:hheight-6,1:hwidth-6]
                Idiagsq = Irawsq[7:hheight,7:hwidth]+Irawsq[6:hheight-1,6:hwidth-1]+Irawsq[5:hheight-2,5:hwidth-2]+Irawsq[4:hheight-3,4:hwidth-3]+Irawsq[3:hheight-4,3:hwidth-4]+Irawsq[2:hheight-5,2:hwidth-5]+Irawsq[1:hheight-6,1:hwidth-6]
                sc = np.mean(np.sqrt((NxN*Idiagsq-Idiag**2)/(NxN*(NxN-1)))/(Idiag/NxN))
                frame_means.append(sc)
                if not math.isnan(sc):
                    val = int(round(sc*65535))
                    my_str = my_str + "%04x" % val
            # send a string of values to the client consisting of speckle contrast means, each value is encoded in hexadecimal by adding the %04x before it
            self.client_sock.sendall(my_str)
        except:            
            for j in frame_means:
                if not math.isnan(j):
                    val = int(round(j*65535))
                    my_str = my_str + "%04x" % val
            self.client_sock.sendall(my_str) 
        
    ## Records and processes capture speckle video
    def speckleProcessing_capture(self, path, name, length):
        timestamp = datetime.now().strftime("%Y_%m_%d-%I_%M_%S_%p")
        name_str = f"{name}_{timestamp}" + '.h264'
        self.camera.start_recording(path + "/"  + name_str) 
        sleep(length)
        self.camera.stop_recording()
        return path + "/"  + name_str
#**************************** update with code determining which shutter speed to choose            
    def determine_optimal_shutterspeed(self, image_output_per_shutter_speed):
        # in this function we look at the image_output_per_shutter_speed values at each shutter speed and decide on the optimum value
        
        self.camera.shutter_speed = 2000 #placeholder for default setting
        self.log.debug('Optimal shutter speed is determined to be' + f"{self.camera.shutter_speed}")
#*************************************************************    
    ##
    def system_baseline(self):
        # autoexpsure
        self.autoExposureAdjustment()#runs through exposure times 1000-5000 us in increments of 1000 us and determines the best exposure time for that criteria     
        # optimal exposure settings are logged in determine_optimal_shutterspeed which is called in autoExposureAdjustment 
        # measure 1 min of video w/laser on followed by 5 seconds of dark frames
        self.laser.laser_on()
        self.timed_recording(12)#setting 12 cycles of the 2 seconds every 10 seconds for a total of 60 seconds of data capture
                self.laser.laser_off()
        self.speckleProcessing_capture(self.testDir, 'dark', 5) #capturing 5 seconds of dark frames
        self.client_sock.sendall('ping')
                
#     def calibrate_system(self):
#         print('in noise')
#         self.camera.shutter_speed = 200
#         sleep(1)
#         data = self.client_sock.recv(1024)
#         data = data.decode("utf-8").strip()
#         if data == 'y':
#             self.speckleProcessing_capture(self.testDir, 'noise', 5)
#             self.client_sock.sendall('ping')
#             self.camera.shutter_speed = self.expMode
#             
#         print('in paper')
#         data = self.client_sock.recv(1024)
#         data = data.decode("utf-8").strip()
#         if data == 'y':
#             self.laser.laser_on()
#             self.speckleProcessing_capture(self.testDir, 'paper', 5)
#             self.laser.laser_off()
#             self.client_sock.sendall('ping')
#             
            
    def record(self):
        acc_tmp = []
#         rec_dur = self.client_sock.recv(1024)
#         rec_dur = int(rec_dur)
        rec_dur = 20
        rec_cyc = 2
#         rec_cyc = self.client_sock.recv(1024)
#         rec_cyc = int(rec_cyc)
        self.client_sock.sendall('recording')
        timestamp = datetime.now().strftime("%Y_%m_%d-%I_%M_%S_%p")
        name_str = f"data_{timestamp}" + '.h264'
        full_name = self.capDir + "/"  + name_str
        starttime = time()
        print('line 314')
        self.camera.start_recording(full_name) 
        while (time() < starttime + rec_dur):
            acc_tmp.append(self.acc.read())
#             tmp_vals = self.therm.read()
#             acc_tmp.append(tmp_vals)
            acc_tmp.append(datetime.now().strftime("%I_%M_%S_%f_%p"))
            if  self.ir.read():
                self.laser.laser_off()
                print('Device has lost contact and the laser is now off.')
#             if tmp_vals[1] > 40:
#                 print('Device is hot. Turn off.')
        self.camera.stop_recording()
        cap = cv2.VideoCapture(full_name)
        at_name = self.capDir + "/"  + f"accthm_{timestamp}" + '.txt'
        hf.write_to_file(str(acc_tmp), at_name)
        
        while True:
            print('New loop')
            acc_tmp = []
            print(time())
            data = self.client_sock.recv(1024)
            if 's' in str(data):
                print(data)
                self.laser.laser_off()
                self.client_sock.sendall('ended')
                break
            print(time())
            timestamp = datetime.now().strftime("%Y_%m_%d-%I_%M_%S_%p")
            name_str = f"data_{timestamp}" + '.h264'
            full_name = self.capDir + "/"  + name_str
            starttime = time()
            self._running = True
            self.camera.start_recording(full_name)
            t = Thread(target = self.start_processing, args = (cap,))
            t.start()
            while (time() < starttime + rec_dur):
                acc_tmp.append(self.acc.read())
#                 tmp_vals = self.therm.read()
#                 acc_tmp.append(tmp_vals)
                acc_tmp.append(datetime.now().strftime("%I_%M_%S_%f_%p"))
                if self.ir.read():
                    self.laser.laser_off()
                    print('Device has lost contact and the laser is now off.')
#                 if tmp_vals[1] > 40:
#                     print('Device is hot. Turn off.')
            self._running = False
#             t.join()python3 main_bt.py
            self.camera.stop_recording()
            print(time())
            self.client_sock.sendall('done')
            cap = cv2.VideoCapture(full_name)
            while (time() < starttime + rec_cyc):
                print('In second loop')
                acc_tmp.append(self.acc.read())
#                 tmp_vals = self.therm.read()
#                 acc_tmp.append(tmp_vals)
                acc_tmp.append(datetime.now().strftime("%I_%M_%S_%f_%p"))
                if self.ir.read():
                    self.laser.laser_off()
                    print('Device has lost contact and the laser is now off.')
#                 if tmp_vals[1] > 40:
#                     print('Device is hot. Turn off.')
            at_name = self.capDir + "/"  + f"accthm_{timestamp}" + '.txt'
            hf.write_to_file(str(acc_tmp), at_name)
            print(time())

    def timed_recording(self,cycle_number):
        print(cycle_number)
        acc_tmp = []
#         rec_dur = self.client_sock.recv(1024)
#         rec_dur = int(rec_dur)
        rec_dur = 5
        rec_cyc = 2
#         rec_cyc = self.client_sock.recv(1024)
#         rec_cyc = int(rec_cyc)
        self.client_sock.sendall('recording')
        timestamp = datetime.now().strftime("%Y_%m_%d-%I_%M_%S_%p")
        name_str = f"data_{timestamp}" + '.h264'
        full_name = self.capDir + "/"  + name_str
        starttime = time()
        self.camera.start_recording(full_name)
        while (time() < starttime + rec_dur):
            acc_tmp.append(self.acc.read())
#             tmp_vals = self.therm.read()
#             acc_tmp.append(tmp_vals)
            acc_tmp.append(datetime.now().strftime("%I_%M_%S_%f_%p"))
            if self.ir.read():
                self.laser.laser_off()
                print('Device has lost contact and the laser is now off.')
#             if tmp_vals[1] > 40:
#                 print('Device is hot. Turn off.')
        self.camera.stop_recording()
        cap = cv2.VideoCapture(full_name)
        at_name = self.capDir + "/"  + f"accthm_{timestamp}" + '.txt'
        hf.write_to_file(str(acc_tmp), at_name)
        a = 1
        while (a < cycle_number):
            print('New loop')
            acc_tmp = []
            print(time())
            data = self.client_sock.recv(1024)
            if 's' in str(data):
                print(data)
                self.laser.laser_off()
                self.client_sock.sendall('ended')
                break
            print(time())
            timestamp = datetime.now().strftime("%Y_%m_%d-%I_%M_%S_%p")
            name_str = f"data_{timestamp}" + '.h264'
            full_name = self.capDir + "/"  + name_str
            starttime = time()
            self._running = True
            self.camera.start_recording(full_name)
            t = Thread(target = self.start_processing, args = (cap,))
            t.start()
            while (time() < starttime + rec_dur):
                acc_tmp.append(self.acc.read())
#                 tmp_vals = self.therm.read()
#                 acc_tmp.append(tmp_vals)
                acc_tmp.append(datetime.now().strftime("%I_%M_%S_%f_%p"))
                if self.ir.read():
                    self.laser.laser_off()
                    print('Device has lost contact and the laser is now off.')
#                 if tmp_vals[1] > 40:
#                     print('Device is hot. Turn off.')
            self._running = False
#             t.join()python3 main_bt.py
            self.camera.stop_recording()
            print(time())
            self.client_sock.sendall('done')
            cap = cv2.VideoCapture(full_name)
            while (time() < starttime + rec_cyc):
                print('In second loop')
                acc_tmp.append(self.acc.read())
#                 tmp_vals = self.therm.read()
#                 acc_tmp.append(tmp_vals)
                acc_tmp.append(datetime.now().strftime("%I_%M_%S_%f_%p"))
                if self.ir.read():
                    self.laser.laser_off()
                    print('Device has lost contact and the laser is now off.')
#                 if tmp_vals[1] > 40:
#                     print('Device is hot. Turn off.')
            at_name = self.capDir + "/"  + f"accthm_{timestamp}" + '.txt'
            hf.write_to_file(str(acc_tmp), at_name)
            print(time())
            a=a+1
    # Listens for a command to setup device including: 'LASER ON','LASER OFF', 'BASELINE','OCCLUSION','FULL TRIAL','USB BACKUP', or 'END'.
    def start_listening(self):
        try:
            while True:
                ## Reads the input to the PC 
                data = self.client_sock.recv(1024)
                if len(data) == 0:
                    break
                data = data.decode("utf-8").strip()
                print('Received input: ' + data)
                
                ## Laser control inputs
                if data == 'LASER ON':
                    self.laser.laser_on()
                    
                if data == 'LASER OFF':
                    self.laser.laser_off()
                
                #data capture inputs
                if data == 'BASELINE':## Captures data during baseline
                    self.system_baseline()
                                        
                if data == 'OCCLUSION':## Captures data during cuff occlusion
                    self.laser.laser_on()
                    self.timed_recording(8)
                    self.laser.laser_off()
                    
#               if data == 'CAL'
#                     self.calibrate_system(self)
                                          
                if data == 'FULL TRIAL':## Captures data during labor/cesarean delivery/postpartum recovery
                    self.laser.laser_on()
                    self.record()
                    self.laser.laser_off()
                
                #data transfer to usb drive after study is over
                if data == 'USB BACKUP':
                    # run system script to transfer data from A to B
                    # send message to client that transfer is done
                    cmd1 = "sudo python3 something something"
                    call([cmd1], shell=True)
                
                ## Ends the program
                if data == 'END':
                    self.activeMode = False
                    self.laser.laser_off()
                    self.close()
                    break
                    
#                 ## Previewing data functions
#                 if data == 'PREVIEW':
#                     self.laser.laser_on()
#                     vid_path = self.speckleProcessing_capture(self.testDir, 'preview', 10)
#                     self.laser.laser_off()
#                     cap = cv2.VideoCapture(vid_path)
#                     self.processVideo(cap)
#                     self.client_sock.sendall('done')
                                    
        except IOError:
            self.close()
            
    # Terminates connection + passes to a different file if Bluetooth is lost
    def close(self):
        print("disconnected")
        self.client_sock.close()
        self.server_sock.close()
#         if self.activeMode == True:
#             noBT.noBTCont(self.masterDir, self.camera)
        GPIO.cleanup()
        print("all done")
        # todo -> launch the main script again, something like below but need to double check
        cmd1 = "python3 /home/pi/Documents/device_v3/bluetooth_program_v1/main_bt.py"
        call([cmd1], shell=True)
        