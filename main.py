# see changelog and README for explanation

simulation = False  # True

bufferLandmarks = True

makeConesOnlyFirstLap = True
## TODO?: makeConesAfterMapload = False #whether to completely rely on the loaded map, or to allow for new cone detection
delaySLAMuntillDriving = True  # NOTE: only affects SLAM's pos updates, not it's cone placement (SLAM needs to place
# new cones in order to start driving, it's a fun paradox)

autoTrackDiscovery = False

useDrawer = False  # whether to draw stuff on the screen (pretty useful for debugging and user-interfacing, but it does consume a bunch of processing power)
printConnectionDebug = True  # whether to explicitely print out the details of the kartMCU and LiDAR connections (serial port stuff)
saveMapOnClose = False  # please enable when doing any sort of relevant testing (but having this set to False will prevent your folders overflowing with map files)

importCmdlineMapsAsBoth = True  # if enabled, maps imported from the cmdline will be imported as both an undiscovered map AND the discovered cones. This is mostly for demo or test purposes

from Map import Map

import coneConnecting as CC
import pathFinding as PF
import pathPlanningTemp as PP  # autodriving and splines

from log.mainLog import mapLoggerClass  # some basic logging for the masterMap

import realCar as RC
from HWserialConn import shuffleSerials

import time  # used for time.sleep()

########### ROS ###########
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32


class MinimalSubscriber(Node):

    def __init__(self, main_func):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Float32,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.main_func = main_func

    def listener_callback(self, msg):
        self.main_func(msg.data)
        self.get_logger().info('I heard: "%s"' % msg.data)


########### ROS ###########


class masterMapClass(Map):
    """a wrapper for the Map class which adds/initializes things
        the Map class is the same for both simulations and IRL runs,
         this class changes (a little) based on the situation"""

    def __init__(self):
        Map.__init__(self)  # init map class
        self.car = RC.realCar(self.clock)
        self.car.pathFolData = PP.pathPlannerCarData()
        self.pathFolData = PP.pathPlannerMapData()


def main_function(theta):
    loopStart = masterMap.clock()
    loopSpeedTimers = [('start', time.time())]

    doNothing = 0
    mapLogger.logMap(masterMap)
    loopSpeedTimers.append(('mapLogger', time.time()))

    # print("speed times:", [(loopSpeedTimers[i][0], round((loopSpeedTimers[i][1]-loopSpeedTimers[i-1][1])*1000, 1)) for i in range(1,len(loopSpeedTimers))])

    ## Setting steering
    masterMap.car.desired_steering = theta
    # print("desired steering:", masterMap.car.desired_steering)

    ## Actual steering
    print("steering:", masterMap.car.steering)

    ## Setting velocity
    masterMap.car.desired_velocity = 1
    # print("desired velocity:", masterMap.car.desired_velocity)

    ## Actual velocity
    print("velocity:", masterMap.car.velocity)

    masterMap.car.update()  # retrieve data from the kartMCU and transmit the desired speed/steering
    loopSpeedTimers.append(('car.update', time.time()))

    loopEnd = masterMap.clock()  # this is only for the 'framerate' limiter (time.sleep() doesn't accept negative numbers, this solves that)
    if (loopEnd - loopStart) < 0.015:  # 60FPS limiter (optional)
        time.sleep(0.0155 - (loopEnd - loopStart))
    elif (loopEnd - loopStart) > (1 / 5):
        print("main process running slow", 1 / (loopEnd - loopStart))


try:
    masterMap = masterMapClass()

    #### connect the kartMCU and lidar(s):
    defaultExclusionList = [
        "COM256", ]  # a quick fix for the bad MB drivers (which make an un-blockable COM port for no reason)
    ## first the kartMCU:
    while (not masterMap.car.connect(comPort=None, autoFind=True, tryAny=True, exclusionList=defaultExclusionList,
                                     printDebug=printConnectionDebug)):
        time.sleep(0.5)  # a nice long delay, to avoid spamming the terminal output
    masterMap.car.doHandshakeIndef(resetESP=True, printDebug=True)
    if not masterMap.car.is_ready:
        print("realCar handshake failed or something, i don't feel like dealing with this")
        raise (Exception("nah, bro"))
    ## initialize the lidar(s)
    ## now make sure the serial ports are actually connected to the correct objects:
    shuffleSerials(
        masterMap.car)  # , *lidars) # pass all things with a handshakeSerial, so they can be shuffled untill correct

    ## now that all the connections are established, let's start initializing some stuff:
    masterMap.car.setSteeringEnable(True)  # enable/disable the steering motor (so a human can drive the kart)
    masterMap.car.setPedalPassthroughEnable(
        True)  # enable/disable the steering motor (so a human can drive the kart)

    if bufferLandmarks:
        landmarkBufferLengthThreshold = 5  # if the number of landmarks in the buffer >= this threshold, then process them immediately (dont wait for the timer)
        landmarkBufferTimer = masterMap.clock()
        landmarkBufferTimeInterval = 0.2

    masterMap.car.slamData = masterMap.clock()  # store time since last SLAM update in car.slamData
    # SlamUpdateInterval = landmarkBufferTimeInterval

    simDriftVelocityError = simDriftVelocityError = 0.0  # init var
    mapLogger = mapLoggerClass()

    lastCones = {False: None, True: None}  # the cones at the end of each boundry. Attempt to connect these
    autoPathFindTimer = masterMap.clock()
    autoPathFindInterval = CC.coneConnecter.findFirstConesDelay  # initialized to how long the car should wait before building attempting the first connections

    ######################################################################################## main loop ###############################################################################
    rclpy.init()

    minimal_subscriber = MinimalSubscriber(main_function)

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()
finally:
    print("main ending")
    if not simulation:
        try:
            masterMap.car.desired_velocity = 0.0;
            masterMap.car.desired_steering = 0.0
            masterMap.car.setSteeringEnable(
                False)  # this has the useful side-effect of sending the desired velocity and steering to the kartMCU (hopefully stopping it)
            masterMap.car.disconnect()
            print("closing car comm success")
        except:
            print("couldn't stop car communication")
    print("main ended")
