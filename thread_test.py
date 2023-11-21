from thermistors import thermistors as therm
from accelerometer import accelerometer as acc
from ir_sensor import ir_sensor as ir
from laser_ctrl import laser_ctrl as laser
from picamera import PiCamera, PiCameraError
from threading import Thread
from time import sleep, time
import cv2
import numpy as np
import math

class video_process:
    def __init__(self, video_path, res):
        self._running = True
        self.path = video_path
        self.res = res
        
    def terminate(self):
        self._running = False
        
    def run(self):
        cap = cv2.VideoCapture(self.path)
        res = self.res
        h = res[1]
        w = res[0]
        my_str = 'a'
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
        except:
            print(len(frame_means))
            
            for j in frame_means:
                if not math.isnan(j):
                    val = int(round(j*65535))
                    my_str = my_str + "%04x" % val
            print(my_str)

starttime = time()
v = video_process('/home/pi/Documents/PPH_studies/test01.h264', (640,480))
t = Thread(target = v.run, args = ())
t.start()
sleep(9)
v.terminate()
#t.join()