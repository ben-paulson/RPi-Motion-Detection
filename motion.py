from imutils.video import VideoStream
from imutils import face_utils
import imutils
import cv2
import time
import RPi.GPIO as GPIO
import math
from threading import Thread
import spidev

class MotionDetector():

    def __init__(self):
        print ("Initializing...")
        self.pxCheck = 20 # larger number = less pixels to check each frame
        self.vs = VideoStream(usePiCamera=True).start() # using webcam for testing, will change to RPi
        time.sleep(2.0)
        self.height, self.width = self.vs.read().shape[:2] # won't be correct if frame is resized later
        self.numPixels = (self.width * self.height) / self.pxCheck # number of pixels to check
        self.prevColorTotals = [] # BGR
        self.prevColor = []
        self.pixDiff = [0, 0, 0] # BGR
        self.colorMax = self.numPixels * 255.0
        self.outputPin = 12
        self.outputPin2 = 32
        self.outFreq = 1000
        self.duty = 100
        self.motionMax = 0.1#0.05
        self.running = True
        self.sweeping = False
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.outputPin, GPIO.OUT)
        GPIO.setup(self.outputPin2, GPIO.OUT)
        self.p = GPIO.PWM(self.outputPin, self.outFreq)
        self.p.start(self.duty)
        self.p2 = GPIO.PWM(self.outputPin2, self.outFreq)
        self.p2.start(self.duty)

        self.just_swept = False
        self.spi = spidev.SpiDev()
        self.spi2 = spidev.SpiDev()
        self.spi.open(0, 0)
        self.spi2.open(0, 1)
        self.spi.max_speed_hz = 976000
        self.spi2.max_speed_hz = 976000
        self.sweep_speed = 0.005

    def write_pot(self, input, pot):
        msb = input >> 8
        lsb = input & 0xFF
        #print(lsb)
        if pot == 1:
                self.spi.xfer([msb, lsb])
        else:
                self.spi2.xfer([msb, lsb])

    def sweep(self):
        self.sweeping = True
        for i in range(0x00, 0x80, 1):
                self.write_pot(i, 1)
                self.write_pot(i, 2)
                time.sleep(self.sweep_speed)
        for i in range(0x80, 0x00, -1):
                self.write_pot(i, 1)
                self.write_pot(i, 2)
                time.sleep(self.sweep_speed)
        self.sweeping = False

    def run(self):
        if self.running:
            frame = self.vs.read()
        #frame = imutils.resize(frame, width=450)
        #frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        #print frame[0][0]
        #print checkGrayscale(frame)
        if not self.sweeping:
            self.write_pot(0x80, 1)
            self.write_pot(0x80, 2)
            newDuty = self.motionToPWM(self.checkGrayscale(frame))
            if self.just_swept:
                self.duty = newDuty
            #print (abs(newDuty - self.duty))
            self.just_swept = False
            if abs(newDuty - self.duty) > 15:
                #self.sweeping = True
                #print ("yes")
                #self.sweep()
                #Sweeper(self.spi, self.spi2, self.sweep_speed, self, self.sweeping).start()
                #self.sweep()
                #Thread(target=self.sweep, args=()).start()
                self.p.ChangeDutyCycle(100 - newDuty)  #self.updateDuty(newDuty, self.duty, 10, 0.01)
                self.p2.ChangeDutyCycle(100 - newDuty)
                self.duty = newDuty
                #self.sweeping = False
                #self.just_swept = True
            #print (self.duty)
        else:
             newDuty = self.motionToPWM(self.checkGrayScale(frame))
             self.duty = newDuty

    def updateDuty(self, old, new, counts, delay):
        while abs(old - new) > 1:
            
            if old < new:
                old += 1
            else:
                old -= 1
            self.p.ChangeDutyCycle(old)
            self.p2.ChangeDutyCycle(old)
            time.sleep(delay)

    def stop(self):
        self.running = False

    def checkGrayscale(self, f):
        f = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)
        colorTotal = 0
        movement = 0

        for x in range(self.width):
            for y in range(self.height):
                if x % self.pxCheck == y % self.pxCheck:
                    colorTotal += f[y, x]
                    #print f[y, x]

        if len(self.prevColor) == 0:
            self.prevColor.append(colorTotal)
        else:
            movement = abs(self.prevColor[0] - colorTotal) / self.colorMax
            del self.prevColor[:]
            self.prevColor.append(colorTotal)
        cv2.imshow("Frame", f) 
        return movement

        #print colorTotal
        cv2.imshow("Frame", f)

    def checkBGR(self, f):
        currColorTotals = [0, 0, 0] # BGR
    

        movement = 0
                
        for x in range(self.width):
            for y in range(self.height):
                if x % self.pxCheck == y % self.pxCheck:
                    # Store color values of some pixels
                    currColorTotals[0] += f[y, x][0]
                    currColorTotals[1] += f[y, x][1]
                    currColorTotals[2] += f[y, x][2]
                    #colorTotal += f[y, x]
                    #frame[y, x] = (255, 255, 0)

        if len(self.prevColorTotals) == 0: # load initial values in first frame
            self.prevColorTotals.extend(currColorTotals)
        else:
            # Compare current frame to previous frame, average all 3 channels
            for i in range(3):
                self.pixDiff[i] = abs(self.prevColorTotals[i] - currColorTotals[i]) / self.colorMax
            for colorChannel in self.pixDiff:
                movement += colorChannel

            movement /= 3.0
            del self.prevColorTotals[:]
            self.prevColorTotals.extend(currColorTotals)

        cv2.imshow("Frame", f)
                
        return movement

    def motionToPWM(self, motion):
        if motion > self.motionMax:
            motion = self.motionMax

        # PWM duty cycle
        dty = (motion / self.motionMax) * 100.0
        return dty

    def cleanup(self):
        cv2.destroyAllWindows()
        self.vs.stop()
        self.p.stop()
        self.p2.stop()
        GPIO.cleanup()

class Sweeper(Thread):
    def __init__(self, spi, spi2, speed, det, sweeping):
        Thread.__init__(self)
        self.spi = spi
        self.spi2 = spi2
        self.sweep_speed = speed
        self.det = det
        self.sweeping = sweeping

    def run(self):
        #self.p.ChangeDutyCycle(1)
        #self.p2.ChangeDutyCycle(1)
        self.sweep()
        #self.p.ChangeDutyCycle(100)
        #self.p2.ChangeDutyCycle(100)
    def sweep(self):
        self.sweeping = True
        for i in range(0x00, 0x80, 1):
                self.write_pot(i, 1)
                self.write_pot(i, 2)
                time.sleep(self.sweep_speed)
        for i in range(0x80, 0x00, -1):
                self.write_pot(i, 1)
                self.write_pot(i, 2)
                time.sleep(self.sweep_speed)
        self.sweeping = False

    def write_pot(self, input, pot):
        msb = input >> 8
        lsb = input & 0xFF
        #print(lsb)
        if pot == 1:
                self.spi.xfer([msb, lsb])
        else:
                self.spi2.xfer([msb, lsb])


detector = MotionDetector()

while True:
    detector.run()
    # if the `q` key was pressed, break from the loop
    if cv2.waitKey(1) & 0xFF == ord("q"):
        detector.stop()
        break

detector.cleanup()
