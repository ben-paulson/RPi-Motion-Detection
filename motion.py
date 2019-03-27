from imutils.video import VideoStream
from imutils import face_utils
import imutils
import cv2

pxCheck = 100000000 # larger number = less pixels to check each frame
vs = VideoStream(src=0).start() # using webcam for testing, will change to RPi
height, width = vs.read().shape[:2] # won't be correct if frame is resized later
numPixels = (height * width) / pxCheck # number of pixels to check
prevColorTotals = [] # BGR
prevColor = []
pixDiff = [0, 0, 0] # BGR
colorMax = numPixels * 255.0

previous = None

def checkGrayscale(f):
    f = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)
    colorTotal = 0
    movement = 0

    for x in range(width):
        for y in range(height):
            if x % pxCheck == y % pxCheck:
                colorTotal += f[y, x]
                #print f[y, x]

    if len(prevColor) == 0:
        prevColor.append(colorTotal)
    else:
        movement = abs(prevColor[0] - colorTotal) / colorMax
        del prevColor[:]
        prevColor.append(colorTotal)

    #print colorTotal
    cv2.imshow("Frame", f)

    return movement

def checkBGR(f):
    currColorTotals = [0, 0, 0] # BGR
    

    movement = 0
    
    for x in range(width):
        for y in range(height):
            if x % pxCheck == y % pxCheck:
                # Store color values of some pixels
                currColorTotals[0] += f[y, x][0]
                currColorTotals[1] += f[y, x][1]
                currColorTotals[2] += f[y, x][2]
                #colorTotal += f[y, x]
                #frame[y, x] = (255, 255, 0)

    if len(prevColorTotals) == 0: # load initial values in first frame
        prevColorTotals.extend(currColorTotals)
    else:
        # Compare current frame to previous frame, average all 3 channels
        for i in range(3):
            pixDiff[i] = abs(prevColorTotals[i] - currColorTotals[i]) / colorMax
        for colorChannel in pixDiff:
            movement += colorChannel

        movement /= 3.0
        del prevColorTotals[:]
        prevColorTotals.extend(currColorTotals)

    cv2.imshow("Frame", f)
    
    return movement

while True:
 
    frame = vs.read()
    #frame = imutils.resize(frame, width=450)
    #frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    #print frame[0][0]
    
    print checkGrayscale(frame)
    
    
    
    key = cv2.waitKey(1) & 0xFF
 
    # if the `q` key was pressed, break from the loop
    if key == ord("q"):
        break


 
# cleanup
cv2.destroyAllWindows()
vs.stop()
