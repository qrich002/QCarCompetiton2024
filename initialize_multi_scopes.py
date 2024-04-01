# initialize_multi_scopes

import cv2
import pyqtgraph as pg
from parameters_config import Config, GeneralConfig
import pal.resources.images as images
from pal.utilities.scope import MultiScope


def initialize_multi_scopes(IS_PHYSICAL_CAR, enableSteeringControl, waypointSequence, initialPoseition, ControlLoop, config: GeneralConfig):
    
    #Returns:
    # - A tuple containing the speed scope, steering scope (or None if not enabled),
    #   the control thread, and the KILL_THREAD flag.

    KILL_THREAD = False 

    # Setup the speed control monitoring scope with 3 rows for different measurements:
    # vehicle speed, speed error, and throttle command.
    speedScope = MultiScope(
        rows=3,
        cols=1,
        title='Vehicle Speed Control',
        fps= config.fps
    )

    # First row for monitoring vehicle speed
    speedScope.addAxis(
        row=0,
        col=0,
        timeWindow= config.tf,  # Display data for the duration of the experiment
        yLabel='Vehicle Speed [m/s]',
        yLim=(0, 1)  # Expected speed range
    )
    speedScope.axes[0].attachSignal(name='v_meas', width=2)  # Measured speed
    speedScope.axes[0].attachSignal(name='v_ref')  # Reference speed

    # Second row for monitoring the speed error (difference between reference and measured speed)
    speedScope.addAxis(
        row=1,
        col=0,
        timeWindow= config.tf,
        yLabel='Speed Error [m/s]',
        yLim=(-0.5, 0.5)  # Expected error range
    )
    speedScope.axes[1].attachSignal()

    # Third row for monitoring the throttle command sent to the vehicle
    speedScope.addAxis(
        row=2,
        col=0,
        timeWindow=config.tf,
        xLabel='Time [s]',
        yLabel='Throttle Command [%]',
        yLim=(-0.3, 0.3)  # Expected throttle range
    )
    speedScope.axes[2].attachSignal()

     # Scope for monitoring steering controller
    if enableSteeringControl:
        steeringScope = MultiScope(
            rows=4,
            cols=2,
            title='Vehicle Steering Control',
            fps=config.fps
        )

        steeringScope.addAxis(
            row=0,
            col=0,
            timeWindow=config.tf,
            yLabel='x Position [m]',
            yLim=(-2.5, 2.5)
        )
        steeringScope.axes[0].attachSignal(name='x_meas')
        steeringScope.axes[0].attachSignal(name='x_ref')

        steeringScope.addAxis(
            row=1,
            col=0,
            timeWindow=config.tf,
            yLabel='y Position [m]',
            yLim=(-1, 5)
        )
        steeringScope.axes[1].attachSignal(name='y_meas')
        steeringScope.axes[1].attachSignal(name='y_ref')

        steeringScope.addAxis(
            row=2,
            col=0,
            timeWindow=config.tf,
            yLabel='Heading Angle [rad]',
            yLim=(-3.5, 3.5)
        )
        steeringScope.axes[2].attachSignal(name='th_meas')
        steeringScope.axes[2].attachSignal(name='th_ref')

        steeringScope.addAxis(
            row=3,
            col=0,
            timeWindow=config.tf,
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

        # Load an image of the cityscape to visually represent the vehicle's environment.
        im = cv2.imread(images.SDCS_CITYSCAPE, cv2.IMREAD_GRAYSCALE)
        steeringScope.axes[4].attachImage(
            scale=(-0.002035, 0.002035),
            offset=(1125, 2365),
            rotation=180,
            levels=(0, 255)
        )
        steeringScope.axes[4].images[0].setImage(image=im)

        # Add a reference path to the steering scope to visualize the intended path.
        referencePath = pg.PlotDataItem(
            pen={'color': (85, 168, 104), 'width': 2},
            name='Reference'
        )
        steeringScope.axes[4].plot.addItem(referencePath)
        referencePath.setData(waypointSequence[0, :], waypointSequence[1, :])

        # Visual marker for the vehicle's current position and orientation.
        arrow = pg.ArrowItem(
            angle=180,
            tipAngle=60,
            headLen=10,
            tailLen=10,
            tailWidth=5,
            pen={'color': 'w', 'width': 1},
            brush=[196, 78, 82]
        )
        arrow.setPos(initialPoseition[0], initialPoseition[1])
        steeringScope.axes[4].plot.addItem(arrow)
    

