from imutils.video import VideoStream
from imutils import face_utils
import imutils
import cv2
import time

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
        self.outFreq = 100
        self.duty = 50
        self.motionMax = 0.05
        self.running = True

    def run(self):
        if self.running:
            frame = self.vs.read()
        #frame = imutils.resize(frame, width=450)
        #frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        #print frame[0][0]
    
        #print checkGrayscale(frame)
            print (self.motionToPWM(self.checkGrayscale(frame)))

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
            
        return movement

        #print colorTotal
        #cv2.imshow("Frame", f)

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

detector = MotionDetector()

while True:
    detector.run()
    # if the `q` key was pressed, break from the loop
    if cv2.waitKey(1) & 0xFF == ord("q"):
        detector.stop()
        break

detector.cleanup()
