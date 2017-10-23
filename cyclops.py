#!/usr/bin/python

# This is a PARED-DOWN version of eyes.py designed for the Gakken
# WorldEye display.  It renders a single eye (centered on screen) and
# does NOT require the OLED or TFT displays...doesn't even require the
# Snake Eyes Bonnet if you just have it running in autonomous mode.
# Code is just as in-progress as eyes.py and could use some work.

import Adafruit_ADS1x15
import math
import pi3d
import random
import thread
import time
import RPi.GPIO as GPIO
from svg.path import Path, parse_path
from xml.dom.minidom import parse
from gfxutil import *

# INPUT CONFIG for eye motion ----------------------------------------------
# ANALOG INPUTS REQUIRE SNAKE EYES BONNET

JOYSTICK_X_IN   = -1    # Analog input for eye horiz pos (-1 = auto)
JOYSTICK_Y_IN   = -1    # Analog input for eye vert position (")
PUPIL_IN        = -1    # Analog input for pupil control (-1 = auto)
JOYSTICK_X_FLIP = False # If True, reverse stick X axis
JOYSTICK_Y_FLIP = False # If True, reverse stick Y axis
PUPIL_IN_FLIP   = False # If True, reverse reading from PUPIL_IN
TRACKING        = True  # If True, eyelid tracks pupil
PUPIL_SMOOTH    = 16    # If > 0, filter input from PUPIL_IN
PUPIL_MIN       = 0.0   # Lower analog range from PUPIL_IN
PUPIL_MAX       = 1.0   # Upper "
BLINK_PIN       = 23    # GPIO pin for blink button
AUTOBLINK       = True  # If True, eye blinks autonomously


#-----------------------------------------------------------------------------------------------


# Code originally from https://github.com/pageauc/motion-track/blob/master/motion-track.py

print("Loading Please Wait ....")

import os
mypath=os.path.abspath(__file__)       # Find the full path of this python script
baseDir=mypath[0:mypath.rfind("/")+1]  # get the path location only (excluding script name)
baseFileName=mypath[mypath.rfind("/")+1:mypath.rfind(".")]
progName = os.path.basename(__file__)

# Read Configuration variables from config.py file
from config import *

# import the necessary packages
import logging
import time
import cv2
from threading import Thread

from picamera.array import PiRGBArray
from picamera import PiCamera

if debug:
    logging.basicConfig(level=logging.DEBUG,
                    format='%(asctime)s %(levelname)-8s %(funcName)-10s %(message)s\r',
                    datefmt='%Y-%m-%d %H:%M:%S')

# Color data for OpenCV lines and text
cvWhite = (255,255,255)
cvBlack = (0,0,0)
cvBlue = (255,0,0)
cvGreen = (0,255,0)
cvRed = (0,0,255)

mo_color = cvRed  # color of motion circle or rectangle

class PiVideoStream:
    def __init__(self, resolution=(CAMERA_WIDTH, CAMERA_HEIGHT), framerate=CAMERA_FRAMERATE, rotation=0, hflip=False, vflip=False):
        # initialize the camera and stream
        self.camera = PiCamera()
        self.camera.resolution = resolution
        self.camera.rotation = rotation
        self.camera.framerate = framerate
        self.camera.hflip = hflip
        self.camera.vflip = vflip
        self.rawCapture = PiRGBArray(self.camera, size=resolution)
        self.stream = self.camera.capture_continuous(self.rawCapture,
            format="bgr", use_video_port=True)

        # initialize the frame and the variable used to indicate
        # if the thread should be stopped
        self.frame = None
        self.stopped = False

    def start(self):
        # start the thread to read frames from the video stream
        t = Thread(target=self.update, args=())
        t.daemon = True
        t.start()
        return self

    def update(self):
        # keep looping infinitely until the thread is stopped
        for f in self.stream:
            # grab the frame from the stream and clear the stream in
            # preparation for the next frame
            self.frame = f.array
            self.rawCapture.truncate(0)

            # if the thread indicator variable is set, stop the thread
            # and resource camera resources
            if self.stopped:
                self.stream.close()
                self.rawCapture.close()
                self.camera.close()
                return

    def read(self):
        # return the frame most recently read
        return self.frame

    def stop(self):
        # indicate that the thread should be stopped
        self.stopped = True


# GPIO initialization ------------------------------------------------------

GPIO.setmode(GPIO.BCM)
if BLINK_PIN >= 0: GPIO.setup(BLINK_PIN , GPIO.IN, pull_up_down=GPIO.PUD_UP)


# Load SVG file, extract paths & convert to point lists --------------------

# Thanks Glen Akins for the symmetrical-lidded cyclops eye SVG!
# Iris & pupil have been scaled down slightly in this version to compensate
# for how the WorldEye distorts things...looks OK on WorldEye now but might
# seem small and silly if used with the regular OLED/TFT code.
dom               = parse("graphics/cyclops-eye.svg")
vb                = getViewBox(dom)
pupilMinPts       = getPoints(dom, "pupilMin"      , 32, True , True )
pupilMaxPts       = getPoints(dom, "pupilMax"      , 32, True , True )
irisPts           = getPoints(dom, "iris"          , 32, True , True )
scleraFrontPts    = getPoints(dom, "scleraFront"   ,  0, False, False)
scleraBackPts     = getPoints(dom, "scleraBack"    ,  0, False, False)
upperLidClosedPts = getPoints(dom, "upperLidClosed", 33, False, True )
upperLidOpenPts   = getPoints(dom, "upperLidOpen"  , 33, False, True )
upperLidEdgePts   = getPoints(dom, "upperLidEdge"  , 33, False, False)
lowerLidClosedPts = getPoints(dom, "lowerLidClosed", 33, False, False)
lowerLidOpenPts   = getPoints(dom, "lowerLidOpen"  , 33, False, False)
lowerLidEdgePts   = getPoints(dom, "lowerLidEdge"  , 33, False, False)


# Set up display and initialize pi3d ---------------------------------------

DISPLAY = pi3d.Display.create(samples=4)
DISPLAY.set_background(0, 0, 0, 1) # r,g,b,alpha

# eyeRadius is the size, in pixels, at which the whole eye will be rendered.
if DISPLAY.width <= (DISPLAY.height * 2):
    # For WorldEye, eye size is -almost- full screen height
    eyeRadius   = DISPLAY.height / 2.1
else:
    eyeRadius   = DISPLAY.height * 2 / 5

# A 2D camera is used, mostly to allow for pixel-accurate eye placement,
# but also because perspective isn't really helpful or needed here, and
# also this allows eyelids to be handled somewhat easily as 2D planes.
# Line of sight is down Z axis, allowing conventional X/Y cartesion
# coords for 2D positions.
cam    = pi3d.Camera(is_3d=False, at=(0,0,0), eye=(0,0,-1000))
shader = pi3d.Shader("uv_light")
light  = pi3d.Light(lightpos=(0, -500, -500), lightamb=(0.2, 0.2, 0.2))


# Load texture maps --------------------------------------------------------

irisMap   = pi3d.Texture("graphics/iris.jpg"  , mipmap=False,
              filter=pi3d.GL_LINEAR)
scleraMap = pi3d.Texture("graphics/sclera.png", mipmap=False,
              filter=pi3d.GL_LINEAR, blend=True)
lidMap    = pi3d.Texture("graphics/lid.png"   , mipmap=False,
              filter=pi3d.GL_LINEAR, blend=True)
# U/V map may be useful for debugging texture placement; not normally used
#uvMap     = pi3d.Texture("graphics/uv.png"    , mipmap=False,
#              filter=pi3d.GL_LINEAR, blend=False, m_repeat=True)


# Initialize static geometry -----------------------------------------------

# Transform point lists to eye dimensions
scalePoints(pupilMinPts      , vb, eyeRadius)
scalePoints(pupilMaxPts      , vb, eyeRadius)
scalePoints(irisPts          , vb, eyeRadius)
scalePoints(scleraFrontPts   , vb, eyeRadius)
scalePoints(scleraBackPts    , vb, eyeRadius)
scalePoints(upperLidClosedPts, vb, eyeRadius)
scalePoints(upperLidOpenPts  , vb, eyeRadius)
scalePoints(upperLidEdgePts  , vb, eyeRadius)
scalePoints(lowerLidClosedPts, vb, eyeRadius)
scalePoints(lowerLidOpenPts  , vb, eyeRadius)
scalePoints(lowerLidEdgePts  , vb, eyeRadius)

# Regenerating flexible object geometry (such as eyelids during blinks, or
# iris during pupil dilation) is CPU intensive, can noticably slow things
# down, especially on single-core boards.  To reduce this load somewhat,
# determine a size change threshold below which regeneration will not occur;
# roughly equal to 1/2 pixel, since 2x2 area sampling is used.

# Determine change in pupil size to trigger iris geometry regen
irisRegenThreshold = 0.0
a = pointsBounds(pupilMinPts) # Bounds of pupil at min size (in pixels)
b = pointsBounds(pupilMaxPts) # " at max size
maxDist = max(abs(a[0] - b[0]), abs(a[1] - b[1]), # Determine distance of max
              abs(a[2] - b[2]), abs(a[3] - b[3])) # variance around each edge
# maxDist is motion range in pixels as pupil scales between 0.0 and 1.0.
# 1.0 / maxDist is one pixel's worth of scale range.  Need 1/2 that...
if maxDist > 0: irisRegenThreshold = 0.5 / maxDist

# Determine change in eyelid values needed to trigger geometry regen.
# This is done a little differently than the pupils...instead of bounds,
# the distance between the middle points of the open and closed eyelid
# paths is evaluated, then similar 1/2 pixel threshold is determined.
upperLidRegenThreshold = 0.0
lowerLidRegenThreshold = 0.0
p1 = upperLidOpenPts[len(upperLidOpenPts) / 2]
p2 = upperLidClosedPts[len(upperLidClosedPts) / 2]
dx = p2[0] - p1[0]
dy = p2[1] - p1[1]
d  = dx * dx + dy * dy
if d > 0: upperLidRegenThreshold = 0.5 / math.sqrt(d)
p1 = lowerLidOpenPts[len(lowerLidOpenPts) / 2]
p2 = lowerLidClosedPts[len(lowerLidClosedPts) / 2]
dx = p2[0] - p1[0]
dy = p2[1] - p1[1]
d  = dx * dx + dy * dy
if d > 0: lowerLidRegenThreshold = 0.5 / math.sqrt(d)

# Generate initial iris mesh; vertex elements will get replaced on
# a per-frame basis in the main loop, this just sets up textures, etc.
iris = meshInit(32, 4, True, 0, 0.5/irisMap.iy, False)
iris.set_textures([irisMap])
iris.set_shader(shader)
irisZ = zangle(irisPts, eyeRadius)[0] * 0.99 # Get iris Z depth, for later

# Eyelid meshes are likewise temporary; texture coordinates are
# assigned here but geometry is dynamically regenerated in main loop.
upperEyelid = meshInit(33, 5, False, 0, 0.5/lidMap.iy, True)
upperEyelid.set_textures([lidMap])
upperEyelid.set_shader(shader)
lowerEyelid = meshInit(33, 5, False, 0, 0.5/lidMap.iy, True)
lowerEyelid.set_textures([lidMap])
lowerEyelid.set_shader(shader)

# Generate sclera for eye...start with a 2D shape for lathing...
angle1 = zangle(scleraFrontPts, eyeRadius)[1] # Sclera front angle
angle2 = zangle(scleraBackPts , eyeRadius)[1] # " back angle
aRange = 180 - angle1 - angle2
pts    = []
for i in range(24):
    ca, sa = pi3d.Utility.from_polar((90 - angle1) - aRange * i / 23)
    pts.append((ca * eyeRadius, sa * eyeRadius))

eye = pi3d.Lathe(path=pts, sides=64)
eye.set_textures([scleraMap])
eye.set_shader(shader)
reAxis(eye, 0.0)


# Init global stuff --------------------------------------------------------

mykeys = pi3d.Keyboard() # For capturing key presses

startX       = random.uniform(-30.0, 30.0)
n            = math.sqrt(900.0 - startX * startX)
startY       = random.uniform(-n, n)
destX        = startX
destY        = startY
curX         = startX
curY         = startY
moveDuration = random.uniform(0.075, 0.175)
holdDuration = random.uniform(0.1, 1.1)
startTime    = 0.0
isMoving     = False

frames        = 0
beginningTime = time.time()

eye.positionX(0.0)
iris.positionX(0.0)
upperEyelid.positionX(0.0)
upperEyelid.positionZ(-eyeRadius - 42)
lowerEyelid.positionX(0.0)
lowerEyelid.positionZ(-eyeRadius - 42)

currentPupilScale  =  0.5
prevPupilScale     = -1.0 # Force regen on first frame
prevUpperLidWeight = 0.5
prevLowerLidWeight = 0.5
prevUpperLidPts    = pointsInterp(upperLidOpenPts, upperLidClosedPts, 0.5)
prevLowerLidPts    = pointsInterp(lowerLidOpenPts, lowerLidClosedPts, 0.5)

ruRegen = True
rlRegen = True

timeOfLastBlink = 0.0
timeToNextBlink = 1.0
blinkState      = 0
blinkDuration   = 0.1
blinkStartTime  = 0

trackingPos = 0.3

# Generate one frame of imagery
def frame(p):

    global startX, startY, destX, destY, curX, curY
    global moveDuration, holdDuration, startTime, isMoving
    global frames
    global iris
    global pupilMinPts, pupilMaxPts, irisPts, irisZ
    global eye
    global upperEyelid, lowerEyelid
    global upperLidOpenPts, upperLidClosedPts, lowerLidOpenPts, lowerLidClosedPts
    global upperLidEdgePts, lowerLidEdgePts
    global prevUpperLidPts, prevLowerLidPts
    global prevUpperLidWeight, prevLowerLidWeight
    global prevPupilScale
    global irisRegenThreshold, upperLidRegenThreshold, lowerLidRegenThreshold
    global luRegen, llRegen, ruRegen, rlRegen
    global timeOfLastBlink, timeToNextBlink
    global blinkState
    global blinkDuration
    global blinkStartTime
    global trackingPos

    DISPLAY.loop_running()

    now = time.time()
    dt  = now - startTime

    frames += 1
#	if(now > beginningTime):
#		print(frames/(now-beginningTime))


    # initialize variables
    motion_found = False
    biggest_area = MIN_AREA
    image2 = vs.read()  # initialize image2
    global grayimage1

    grayimage2 = cv2.cvtColor(image2, cv2.COLOR_BGR2GRAY)
    # Get differences between the two greyed images
    differenceimage = cv2.absdiff(grayimage1, grayimage2)
    grayimage1 = grayimage2  # save grayimage2 to grayimage1 ready for next image2
    differenceimage = cv2.blur(differenceimage,(BLUR_SIZE,BLUR_SIZE))
    # Get threshold of difference image based on THRESHOLD_SENSITIVITY variable
    retval, thresholdimage = cv2.threshold( differenceimage, THRESHOLD_SENSITIVITY, 255, cv2.THRESH_BINARY )
    try:
        thresholdimage, contours, hierarchy = cv2.findContours( thresholdimage, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE )
    except Exception,e:
        print(str(e))
        contours, hierarchy = cv2.findContours( thresholdimage, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE )

    if contours:
        total_contours = len(contours)  # Get total number of contours
        cx, cy, cw, ch = 0, 0, 0, 0
        for c in contours:              # find contour with biggest area
            found_area = cv2.contourArea(c)  # get area of next contour
            # find the middle of largest bounding rectangle
            if found_area > biggest_area:
                motion_found = True
                biggest_area = found_area
                (x, y, w, h) = cv2.boundingRect(c)
                cx = int(x + w/2)   # put circle in middle of width
                cy = int(y + h/2)   # put circle closer to top
                cw, ch = w, h

    if motion_found:
        # Do Something here with motion datas
        if debug:
            logging.info("cx,cy(%3i,%3i) C:%2i  LxW:%ix%i=%i SqPx" %
                            (cx ,cy, total_contours, cw, ch, biggest_area))

    # Autonomous eye position
    if isMoving == True:
        if dt <= moveDuration:
            scale        = (now - startTime) / moveDuration
            # Ease in/out curve: 3*t^2-2*t^3
            scale = 3.0 * scale * scale - 2.0 * scale * scale * scale
            curX         = startX + (destX - startX) * scale
            curY         = startY + (destY - startY) * scale
        else:
            startX       = destX
            startY       = destY
            curX         = destX
            curY         = destY
            startTime    = now
            isMoving     = False
            if debug:
                logging.info('end motion destX,destY(%3i,%3i)' % (destX, destY))
    else:
        if motion_found:
            destX = cx/float(300)
            destY = cy/float(300)
            destX = -30.0 + destX * 60.0
            destY = -30.0 + destY * 60.0
            moveDuration = 0.2
            holdDuration = 5
            startTime    = now
            isMoving     = True
            if debug:
                logging.info('motion found destX,destY(%3i,%3i)' % (destX, destY))
        elif dt >= holdDuration:
            destX        = random.uniform(-30.0, 30.0)
            n            = math.sqrt(900.0 - destX * destX)
            destY        = random.uniform(-n, n)
            # Movement is slower in this version because
            # the WorldEye display is big and the eye
            # should have some 'mass' to it.
            moveDuration = random.uniform(0.12, 0.35)
            holdDuration = random.uniform(0.15, 1.7)            
            startTime    = now
            isMoving     = True
            if debug:
                logging.info('new random motion destX,destY(%3i,%3i)\r\n' % (destX, destY))


    # Regenerate iris geometry only if size changed by >= 1/2 pixel
    if abs(p - prevPupilScale) >= irisRegenThreshold:
        # Interpolate points between min and max pupil sizes
        interPupil = pointsInterp(pupilMinPts, pupilMaxPts, p)
        # Generate mesh between interpolated pupil and iris bounds
        mesh = pointsMesh(None, interPupil, irisPts, 4, -irisZ, True)
        iris.re_init(pts=mesh)
        prevPupilScale = p

    # Eyelid WIP

    if AUTOBLINK and (now - timeOfLastBlink) >= timeToNextBlink:
        # Similar to movement, eye blinks are slower in this version
        timeOfLastBlink = now
        duration        = random.uniform(0.06, 0.12)
        if blinkState != 1:
            blinkState     = 1 # ENBLINK
            blinkStartTime = now
            blinkDuration  = duration
        timeToNextBlink = duration * 3 + random.uniform(0.0, 4.0)

    if blinkState: # Eye currently winking/blinking?
        # Check if blink time has elapsed...
        if (now - blinkStartTime) >= blinkDuration:
            # Yes...increment blink state, unless...
            if (blinkState == 1 and # Enblinking and...
                (BLINK_PIN >= 0 and    # blink pin held
                 GPIO.input(BLINK_PIN) == GPIO.LOW)):
                # Don't advance yet; eye is held closed
                pass
            else:
                blinkState += 1
                if blinkState > 2:
                    blinkState = 0 # NOBLINK
                else:
                    blinkDuration *= 2.0
                    blinkStartTime = now
    else:
        if BLINK_PIN >= 0 and GPIO.input(BLINK_PIN) == GPIO.LOW:
            blinkState     = 1 # ENBLINK
            blinkStartTime = now
            blinkDuration  = random.uniform(0.035, 0.06)

    if TRACKING:
        # 0 = fully up, 1 = fully down
        n = 0.5 - curY / 70.0
        if   n < 0.0: n = 0.0
        elif n > 1.0: n = 1.0
        trackingPos = (trackingPos * 3.0 + n) * 0.25

    if blinkState:
        n = (now - blinkStartTime) / blinkDuration
        if n > 1.0: n = 1.0
        if blinkState == 2: n = 1.0 - n
    else:
        n = 0.0
    newUpperLidWeight = trackingPos + (n * (1.0 - trackingPos))
    newLowerLidWeight = (1.0 - trackingPos) + (n * trackingPos)

    if (ruRegen or (abs(newUpperLidWeight - prevUpperLidWeight) >=
      upperLidRegenThreshold)):
        newUpperLidPts = pointsInterp(upperLidOpenPts,
          upperLidClosedPts, newUpperLidWeight)
        if newUpperLidWeight > prevUpperLidWeight:
            upperEyelid.re_init(pts=pointsMesh(
              upperLidEdgePts, prevUpperLidPts,
              newUpperLidPts, 5, 0, False, True))
        else:
            upperEyelid.re_init(pts=pointsMesh(
              upperLidEdgePts, newUpperLidPts,
              prevUpperLidPts, 5, 0, False, True))
        prevUpperLidWeight = newUpperLidWeight
        prevUpperLidPts    = newUpperLidPts
        ruRegen = True
    else:
        ruRegen = False

    if (rlRegen or (abs(newLowerLidWeight - prevLowerLidWeight) >=
      lowerLidRegenThreshold)):
        newLowerLidPts = pointsInterp(lowerLidOpenPts,
          lowerLidClosedPts, newLowerLidWeight)
        if newLowerLidWeight > prevLowerLidWeight:
            lowerEyelid.re_init(pts=pointsMesh(
              lowerLidEdgePts, prevLowerLidPts,
              newLowerLidPts, 5, 0, False, True))
        else:
            lowerEyelid.re_init(pts=pointsMesh(
              lowerLidEdgePts, newLowerLidPts,
              prevLowerLidPts, 5, 0, False, True))
        prevLowerLidWeight = newLowerLidWeight
        prevLowerLidPts    = newLowerLidPts
        rlRegen = True
    else:
        rlRegen = False

    # Draw eye

    iris.rotateToX(curY)
    iris.rotateToY(curX)
    iris.draw()
    eye.rotateToX(curY)
    eye.rotateToY(curX)
    eye.draw()
    upperEyelid.draw()
    lowerEyelid.draw()

    k = mykeys.read()
    if k==27:
        mykeys.close()
        DISPLAY.stop()
        exit(0)


def split( # Recursive simulated pupil response when no analog sensor
  startValue, # Pupil scale starting value (0.0 to 1.0)
  endValue,   # Pupil scale ending value (")
  duration,   # Start-to-end time, floating-point seconds
  range):     # +/- random pupil scale at midpoint
    startTime = time.time()
    if range >= 0.125: # Limit subdvision count, because recursion
        duration *= 0.5 # Split time & range in half for subdivision,
        range    *= 0.5 # then pick random center point within range:
        midValue  = ((startValue + endValue - range) * 0.5 +
                     random.uniform(0.0, range))
        split(startValue, midValue, duration, range)
        split(midValue  , endValue, duration, range)
    else: # No more subdivisons, do iris motion...
        dv = endValue - startValue
        while True:
            dt = time.time() - startTime
            if dt >= duration: break
            v = startValue + dv * dt / duration
            if   v < PUPIL_MIN: v = PUPIL_MIN
            elif v > PUPIL_MAX: v = PUPIL_MAX
            frame(v) # Draw frame w/interim pupil scale value


def init_camera():
    global vs

    while True:
        try:
            # Save images to an in-program stream
            # Setup video stream on a processor Thread for faster speed
            print("Initializing Pi Camera ....")
            vs = PiVideoStream().start()
            vs.camera.rotation = CAMERA_ROTATION
            vs.camera.hflip = CAMERA_HFLIP
            vs.camera.vflip = CAMERA_VFLIP
            time.sleep(2.0)  # Allow PiCamera to initialize

            return			
        except KeyboardInterrupt:
            vs.stop()
            print("")
            print("+++++++++++++++++++++++++++++++++++")
            print("User Pressed Keyboard ctrl-c")
            print("+++++++++++++++++++++++++++++++++++")
            print("")
            quit(0)


init_camera()

image1 = vs.read()   # initialize image1 (done once)

try:
    grayimage1 = cv2.cvtColor(image1, cv2.COLOR_BGR2GRAY)
except:

    vs.stop()
    print("Problem Connecting To Camera Stream.")
    print("Restarting Camera.  One Moment Please .....")
    time.sleep(4)

print("Press ctrl-c to Quit")
print("Start Motion Tracking ....")

if not debug:
    print("Note: Console Messages Suppressed per debug=%s" % debug)

cx, cy, cw, ch = 0, 0, 0, 0   # initialize contour center variables

# MAIN LOOP -- runs continuously -------------------------------------------

while True:

    # Fractal auto pupil scale
    v = random.random()
    split(currentPupilScale, v, 4.0, 1.0)
    currentPupilScale = v
