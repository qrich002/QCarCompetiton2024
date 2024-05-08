# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
#***NEED SETUP COMPETITION IN SAME DIRECTORY/FOLDER
#region : File Description and Imports

"""
vehicle_control.py

Skills acivity code for vehicle control lab guide.
Students will implement a vehicle speed and steering controller.
Please review Lab Guide - vehicle control PDF
"""
import os
import signal
import numpy as np
from threading import Thread
import time
import cv2
import pyqtgraph as pg

from pal.products.qcar import QCar, QCarGPS, QCarCameras, QCarRealSense, IS_PHYSICAL_QCAR
from pal.utilities.scope import MultiScope
from pal.utilities.math import wrap_to_pi
from hal.products.qcar import QCarEKF
from hal.products.mats import SDCSRoadMap
import pal.resources.images as images
from hal.utilities.image_processing import ImageProcessing
from pal.utilities.vision import Camera3D


# -------------------------------- Time Parameters -----------------------------------------
# - tf: experiment duration in seconds.
# - startDelay: delay to give filters time to settle in seconds.
# - controllerUpdateRate: control update rate in Hz. Shouldn't exceed 500
tf = 120
startDelay = 0
controllerUpdateRate = 150

# -------------------------- Speed Control Parameters --------------------------------------
# - v_ref: desired velocity in m/s
# - K_p: proportional gain for speed controller
# - K_i: integral gain for speed controller
v_ref = .75
K_p = 1
K_i = 0

# -------------------------- Steering Control Parameters ----------------------------------
# - enableSteeringControl: whether or not to enable steering control
# - K_stanley: K gain for stanley controller
# - nodeSequence: list of nodes from roadmap. Used for trajectory generation.
enableSteeringControl = True
K_stanley = .5
nodeSequence = [10, 2, 4, 14, 20, 22, 10]


#endregion
# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --

#region : Initial setup
if enableSteeringControl:
    roadmap = SDCSRoadMap(leftHandTraffic=False)
    waypointSequence = roadmap.generate_path(nodeSequence)
    initialPose = roadmap.get_node_pose(nodeSequence[0]).squeeze()
else:
    initialPose = [0, 0, 0]

#if not IS_PHYSICAL_QCAR:
#import Setup_Competition
import Setup_Competition
Setup_Competition.terminate()
Setup_Competition.setup(initialPosition=[-1.205,-0.83,0.005], initialOrientation=[0,0,-44.7])

# Used to enable safe keyboard triggered shutdown
global KILL_THREAD
KILL_THREAD = False
def sig_handler(*args):
    global KILL_THREAD
    KILL_THREAD = True
signal.signal(signal.SIGINT, sig_handler)
#endregion

#----------------------Speed Controller-----------------------
class SpeedController:

    def __init__(self, kp=0, ki=0):
        self.maxThrottle = 0.3

        self.kp = kp
        self.ki = ki

        self.ei = 0

    def update(self, v, v_ref, dt):
        
        e = v_ref - v
        self.ei += dt*e

        return np.clip(
            self.kp*e + self.ki*self.ei,
            -self.maxThrottle,
            self.maxThrottle
        )
        
        return 0
#----------------------Steering Controller-----------------------
class SteeringController:

    def __init__(self, waypoints, k=1, cyclic=True):
        self.maxSteeringAngle = np.pi/6

        self.wp = waypoints
        self.N = len(waypoints[0, :])
        self.wpi = 0

        self.k = k
        self.cyclic = cyclic

        self.p_ref = (0, 0)
        self.th_ref = 0
 
    def update(self, p, th, speed):
        wp_1 = self.wp[:, np.mod(self.wpi, self.N-1)]
        wp_2 = self.wp[:, np.mod(self.wpi+1, self.N-1)]
        
        v = wp_2 - wp_1
        v_mag = np.linalg.norm(v)
        try:
            v_uv = v / v_mag
        except ZeroDivisionError:
            return 0

        tangent = np.arctan2(v_uv[1], v_uv[0])

        s = np.dot(p-wp_1, v_uv)

        if s >= v_mag:
            if  self.cyclic or self.wpi < self.N-2:
                self.wpi += 1

        ep = wp_1 + v_uv*s
        ct = ep - p
        dir = wrap_to_pi(np.arctan2(ct[1], ct[0]) - tangent)

        ect = np.linalg.norm(ct) * np.sign(dir)
        psi = wrap_to_pi(tangent-th)

        self.p_ref = ep
        self.th_ref = tangent

        return np.clip(
            wrap_to_pi(psi + np.arctan2(self.k*ect, speed + 0.001)),
            -self.maxSteeringAngle,
            self.maxSteeringAngle)
        
        return 0
#------------------------Main Control Loop Function---------------------------------
def controlLoop():
    
    #region controlLoop setup
    global KILL_THREAD
    u = 0
    delta = 0
    # used to limit data sampling to 10hz
    countMax = controllerUpdateRate / 10
    count = 0
    #-------------------------Camera Initialization--------------------------
    cameraInterfacingLab = ImageInterpretation(
        imageSize=[[820,410], [64,48]],
        frameRate=np.array([30, 30]),
        streamInfo=[3, "RGB"],
        chessDims=6,
        boxSize=1
    )
 
    cameraMatrix  = np.array([
        [495.84,   0.00, 408.03],
        [0.00, 454.60, 181.21],
        [0.00,   0.00,   1.00]
    ])
    
    distortionCoefficients = np.array([
        -0.57513,
        0.37175,
        -0.00489,
        -0.00277,
        -0.11136
    ])
    cameraInterfacingLab.d435CamIntrinsics = cameraMatrix
    cameraInterfacingLab.d435DistParam = distortionCoefficients
    #----------------------"Global" Variables-----------------------
    timerStart = 0
    carStopped = False
    carStopped2 = False
    carStopped3 = False
    carStopped4 = False
    StopLight = False
    StopLight2 = False
    imageWidth = 1280
    imageHeight = 720
    stopLight = True
    counter2 = 0
    counter3 = 0
    #myCam1  = Camera3D(mode='RGB&DEPTH', frameWidthRGB=imageWidth, frameHeightRGB=imageHeight)
    #------------------------------------------------------------------------------
    #endregion

    #region Controller initialization
    speedController = SpeedController(
        kp=K_p,
        ki=K_i
    )
    if enableSteeringControl:
        steeringController = SteeringController(
            waypoints=waypointSequence,
            k=K_stanley
        )
    #endregion

    #region QCar interface setup
    qcar = QCar(readMode=1, frequency=controllerUpdateRate)
    if enableSteeringControl:
        ekf = QCarEKF(x_0=initialPose)
        gps = QCarGPS(initialPose=initialPose)
    else:
        gps = memoryview(b'')
    #endregion

    with qcar, gps:
        t0 = time.time()
        t=0
        counter = 0
        while (t < tf+startDelay) and (not KILL_THREAD):
            #region : Loop timing update
            tp = t
            t = time.time() - t0
            dt = t-tp
            #endregion

            #region : Read from sensors and update state estimates
            qcar.read()
            counter = counter + 1
            if (counter % 60 == 0):
                cameraInterfacingLab.d435Color.read_RGB()
            if enableSteeringControl:
                if gps.readGPS():
                    y_gps = np.array([
                        gps.position[0],
                        gps.position[1],
                        gps.orientation[2]
                    ])
                    ekf.update(
                        [qcar.motorTach, delta],
                        dt,
                        y_gps,
                        qcar.gyroscope[2],
                    )
                else:
                    ekf.update(
                        [qcar.motorTach, delta],
                        dt,
                        None,
                        qcar.gyroscope[2],
                    )

                x = ekf.x_hat[0,0]
                y = ekf.x_hat[1,0]
                th = ekf.x_hat[2,0]
                p = ( np.array([x, y])
                    + np.array([np.cos(th), np.sin(th)]) * 0.2)
            v = qcar.motorTach
            #endregion

            #region : Update controllers and write to car
            if t < startDelay:
                u = 0
                delta = 0
            else:
                #region : Speed controller update
                #Stop Sign 1----------------------------------------------------------
                if p[0] < 2.75 and p[0] > 1.25 and p[1] < 3.5 and p[1] > 2.9:
                    #When car first enters box, stop car and start timer
                    if carStopped == False:
                        u = 0
                        timerStart = time.time()
                        carStopped = True
                    #When timer reaches 3 seconds, continue
                    else:
                        timerEnd = time.time()
                        if (timerEnd - timerStart) >= 3:
                            u = speedController.update(v, v_ref, dt)
                #Stop Sign 2-----------------------------------------------------------
                elif p[0] < -.8 and p[0] > -1.25 and p[1] < 4.5 and p[1] > 4.25:
                    #When car first enters box, stop car and start timer
                    if carStopped2 == False:
                        u = 0
                        timerStart = time.time()
                        carStopped2 = True
                    #When timer reaches 3 seconds, continue
                    else:
                        timerEnd = time.time()
                        if (timerEnd - timerStart) >= 3:
                            u = speedController.update(v, v_ref, dt)
                #Stop Light 1---------------------------------------------------------
                elif p[0] < 2.5 and p[0] > 1.75 and p[1] < 0.15 and p[1] > -.2:
                    #False as long as car has first entered box and light is assumed to be red                    
                    if (StopLight == False):
                        #count every second (counter2 starts from 0)
                        second = counter2 % 60
                        #if it has been one second, isSecond is true
                        if second == 0:
                            isSecond = True
                        else:
                            isSecond = False
                        #increment the counter every loop
                        counter2 = counter2 + 1
                        #if it has been one second, take a picture
                        if isSecond == True:
                            image = cameraInterfacingLab.d435Color.imageBufferRGB
                            #--------------------Image Processing------------------------
                            hsv_img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
                            carStopped3 = True
                            # lower range of green stoplight color in HSV
                            hsv_green_lower = (50, 160, 200)
                            # upper range of green stoplight color in HSV
                            hsv_green_upper = (70, 255, 255)
                            #mask with the range of greens for stoplight color
                            mask = cv2.inRange(hsv_img, hsv_green_lower, hsv_green_upper)
                            #only show the pixels that contain the stoplight color
                            color_image = cv2.bitwise_and(image, image, mask=mask)

                            #print(np.average(color_image))
                            
                            #If the average pixel value is above 0.0015,
                            #stoplight must be green
                            if np.average(color_image) >= 0.0015:
                                carStopped3 = False
                                print("Light is green go!")
                                StopLight = True
                                u = speedController.update(v, v_ref, dt)
                            #otherwise, stop at red light
                            else:
                                print("Light is red stop!")
                                u = 0
                                delta = 0
                #Stop Light 2--------------------------------------------------
                elif p[0] < -1.75 and p[0] > -2.15 and p[1] < 2.1 and p[1] > 1.7:
                    #False as long as car has first entered box and light is assumed to be red  
                    if (StopLight2 == False):
                        #count every second (counter2 starts from 0)
                        second = counter3 % 60
                        #if it has been one second, isSecond is true
                        if second == 0:
                            isSecond = True
                        else:
                            isSecond = False
                        #increment the counter every loop
                        counter3 = counter3 + 1
                        #if it has been one second, take a picture
                        if isSecond == True:
                            image = cameraInterfacingLab.d435Color.imageBufferRGB
                            #--------------------Image Processing------------------------
                            hsv_img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
                            carStopped4 = True
                            # lower range of green stoplight color in HSV
                            hsv_green_lower = (50, 160, 200)
                            # upper range of green stoplight color in HSV
                            hsv_green_upper = (70, 255, 255)
                            # mask with the range of greens for stoplight color
                            mask = cv2.inRange(hsv_img, hsv_green_lower, hsv_green_upper)
                            # only show the pixels that contain the stoplight color
                            color_image = cv2.bitwise_and(image, image, mask=mask)

                            # print(np.average(color_image))
                            
                            #If the average pixel value is above 0.001,
                            #stoplight must be green
                            if np.average(color_image) > 0.001:
                                carStopped4 = False
                                print("Light is green go!")
                                StopLight2 = True
                                u = speedController.update(v, v_ref, dt)
                            #otherwise, stop at red light
                            else:
                                print("Light is red stop!")
                                u = 0
                                delta = 0
                #if car is not in region for any stoplight or stopsign
                #run default controller and reset parameters
                else:
                    u = speedController.update(v, v_ref, dt)
                    StopLight = False
                    StopLight2 = False
                    carStopped2 =False
                    carStopped = False
                    counter2 = 0
                    counter3 = 0

                #endregion

                #region : Steering controller update
                if enableSteeringControl:
                    delta = steeringController.update(p, th, v)
                else:
                    delta = 0
                #endregion
            qcar.write(u, delta)
            #endregion

            #region : Update Scopes
            count += 1
            if count >= countMax and t > startDelay:
                t_plot = t - startDelay

                # Speed control scope
                speedScope.axes[0].sample(t_plot, [v, v_ref])
                speedScope.axes[1].sample(t_plot, [v_ref-v])
                speedScope.axes[2].sample(t_plot, [u])

                # Steering control scope
                if enableSteeringControl:
                    steeringScope.axes[4].sample(t_plot, [[p[0],p[1]]])

                    p[0] = ekf.x_hat[0,0]
                    p[1] = ekf.x_hat[1,0]

                    x_ref = steeringController.p_ref[0]
                    y_ref = steeringController.p_ref[1]
                    th_ref = steeringController.th_ref

                    x_ref = gps.position[0]
                    y_ref = gps.position[1]
                    th_ref = gps.orientation[2]

                    steeringScope.axes[0].sample(t_plot, [p[0], x_ref])
                    steeringScope.axes[1].sample(t_plot, [p[1], y_ref])
                    steeringScope.axes[2].sample(t_plot, [th, th_ref])
                    steeringScope.axes[3].sample(t_plot, [delta])


                    arrow.setPos(p[0], p[1])
                    arrow.setStyle(angle=180-th*180/np.pi)

                count = 0
            #endregion
            continue
    cameraInterfacingLab.stop_cameras()    

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --

class ImageInterpretation():

    def __init__(self,
            imageSize,
            frameRate,
            streamInfo,
            chessDims,
            boxSize):

        # Camera calibration constants:
        self.NUMBER_IMAGES = 15

        # List of variables given by students
        self.imageSize      = imageSize
        self.chessboardDim  = [chessDims,chessDims]
        self.frameRate      = frameRate
        self.boxSize        = boxSize
        self.sampleRate     = 1/self.frameRate
        self.calibFinished  = False

        self.d435CamIntrinsics = np.eye(3,3,dtype= np.float32)

        self.d435DistParam = np.ones((1,5), dtype= np.float32)

        self.streamD435 = np.zeros((self.imageSize[1][0],self.imageSize[1][1]))

        # Information for interfacing with front CSI camera
        enableCameras = [False, False, False, False]
        enableCameras[streamInfo[0]] = True

        # Information for interfacing with Realsense camera
        self.d435Color = QCarRealSense(
            mode=streamInfo[1],
            frameWidthRGB  = self.imageSize[1][0],
            frameHeightRGB = self.imageSize[1][1],
            frameRateRGB   = self.frameRate[1]
        )

        # Initialize calibration tool:
        self.camCalibTool = ImageProcessing()

        self.SimulationTime = 15
 
    def stop_cameras(self):
        # Stopping the image feed for both cameras
        self.d435Color.terminate()

#endregion

#region : Setup and run experiment
if __name__ == '__main__':

    #region : Setup scopes
    if IS_PHYSICAL_QCAR:
        fps = 10
    else:
        fps = 30
    
    # Scope for monitoring speed controller
    speedScope = MultiScope(
        rows=3,
        cols=1,
        title='Vehicle Speed Control',
        fps=fps
    )
    speedScope.addAxis(
        row=0,
        col=0,
        timeWindow=tf,
        yLabel='Vehicle Speed [m/s]',
        yLim=(0, 1)
    )
    speedScope.axes[0].attachSignal(name='v_meas', width=2)
    speedScope.axes[0].attachSignal(name='v_ref')

    speedScope.addAxis(
        row=1,
        col=0,
        timeWindow=tf,
        yLabel='Speed Error [m/s]',
        yLim=(-0.5, 0.5)
    )
    speedScope.axes[1].attachSignal()

    speedScope.addAxis(
        row=2,
        col=0,
        timeWindow=tf,
        xLabel='Time [s]',
        yLabel='Throttle Command [%]',
        yLim=(-0.3, 0.3)
    )
    speedScope.axes[2].attachSignal()

    # Scope for monitoring steering controller
    if enableSteeringControl:
        steeringScope = MultiScope(
            rows=4,
            cols=2,
            title='Vehicle Steering Control',
            fps=fps
        )

        steeringScope.addAxis(
            row=0,
            col=0,
            timeWindow=tf,
            yLabel='x Position [m]',
            yLim=(-2.5, 2.5)
        )
        steeringScope.axes[0].attachSignal(name='x_meas')
        steeringScope.axes[0].attachSignal(name='x_ref')

        steeringScope.addAxis(
            row=1,
            col=0,
            timeWindow=tf,
            yLabel='y Position [m]',
            yLim=(-1, 5)
        )
        steeringScope.axes[1].attachSignal(name='y_meas')
        steeringScope.axes[1].attachSignal(name='y_ref')

        steeringScope.addAxis(
            row=2,
            col=0,
            timeWindow=tf,
            yLabel='Heading Angle [rad]',
            yLim=(-3.5, 3.5)
        )
        steeringScope.axes[2].attachSignal(name='th_meas')
        steeringScope.axes[2].attachSignal(name='th_ref')

        steeringScope.addAxis(
            row=3,
            col=0,
            timeWindow=tf,
            yLabel='Steering Angle [rad]',
            yLim=(-0.6, 0.6)
        )
        steeringScope.axes[3].attachSignal()
        steeringScope.axes[3].xLabel = 'Time [s]'

        steeringScope.addXYAxis(
            row=0,
            col=1,
            rowSpan=4,
            xLabel='x Position [m]',
            yLabel='y Position [m]',
            xLim=(-2.5, 2.5),
            yLim=(-1, 5)
        )

        im = cv2.imread(
            images.SDCS_CITYSCAPE,
            cv2.IMREAD_GRAYSCALE
        )

        steeringScope.axes[4].attachImage(
            scale=(-0.002035, 0.002035),
            offset=(1125,2365),
            rotation=180,
            levels=(0, 255)
        )
        steeringScope.axes[4].images[0].setImage(image=im)

        referencePath = pg.PlotDataItem(
            pen={'color': (85,168,104), 'width': 2},
            name='Reference'
        )
        steeringScope.axes[4].plot.addItem(referencePath)
        referencePath.setData(waypointSequence[0, :],waypointSequence[1, :])

        steeringScope.axes[4].attachSignal(name='Estimated', width=2)

        arrow = pg.ArrowItem(
            angle=180,
            tipAngle=60,
            headLen=10,
            tailLen=10,
            tailWidth=5,
            pen={'color': 'w', 'fillColor': [196,78,82], 'width': 1},
            brush=[196,78,82]
        )
        arrow.setPos(initialPose[0], initialPose[1])
        steeringScope.axes[4].plot.addItem(arrow)
    #endregion

    #region : Setup control thread, then run experiment
    controlThread = Thread(target=controlLoop)
    controlThread.start()

    try:
        while controlThread.is_alive() and (not KILL_THREAD):
            MultiScope.refreshAll()
            time.sleep(0.01)
    finally:
        KILL_THREAD = True
    #endregion
    if not IS_PHYSICAL_QCAR:
        #qlabs_setup.terminate()
        #Setup_Competition.terminate()

        input('Experiment complete. Press any key to exit...')
#endregion
