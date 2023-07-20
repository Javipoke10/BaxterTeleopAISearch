import numpy as np

import message_filters

from simple_node import Node

from std_msgs.msg import(
    Float64MultiArray,
    Int32MultiArray
) 

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import(
    PointCloud2,
    JointState
) 

from PyQt5 import QtGui

from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError

from vision_msgs.msg import Detection2DArray

from tf_baxter_realsense.srv import CalculatePosition

import cv2

import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'

import tensorflow as tf

connected = True

toZero = False

search = False

side = 'Left'

class MinimalSubscriber(Node):

    def __init__(self, updateRightGripLabel, updateLeftGripLabel, updateRightCameraLabel, updateLeftCameraLabel, updateHeadCameraLabel):
        super().__init__('minimal_subscriber')

        self.angles = [0.0 for i in range(7)]

        self.red = tf.keras.models.load_model('./src/baxter_controller/red/pesosBaxter.h5', compile = False)
        self.red.compile()

        #Funciones por constructor
        self.updateRightGripLabel = updateRightGripLabel

        self.updateLeftGripLabel = updateLeftGripLabel

        self.updateRightCameraLabel = updateRightCameraLabel

        self.updateLeftCameraLabel = updateLeftCameraLabel

        self.updateHeadCameraLabel = updateHeadCameraLabel

        self.bridge = CvBridge()

        self.info = [0.0 for i in range(9)]

        self.vision = [0.0 for i in range(4)]

        #Sub boton
        self.buttonSubscription = self.create_subscription(
            Int32MultiArray,
            'Button_info',
            self.button_callback,
            1)
        self.buttonSubscription

        #Sub IA vision
        self.cameraSubscription = message_filters.Subscriber(
            self,
            Detection2DArray,
            '/cv_basics/detections')
        self.cameraSubscription

        #Sub nube de puntos
        self.depthPointSubscription = message_filters.Subscriber(
            self,
            PointCloud2,
            '/camera/depth/color/points')
        self.depthPointSubscription

        #Sincronizacion de mensajes(IA y nube de puntos)
        topicSync = message_filters.ApproximateTimeSynchronizer([self.cameraSubscription, self.depthPointSubscription], queue_size=10, slop=0.5)
        topicSync.registerCallback(self.cameraDetectionsCallback)
        
        self.cvCameraSubscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.headCameraCallback,
            1)
        self.cvCameraSubscription

        #sub posicion haptico
        self.subscription = self.create_subscription(
            PoseStamped,
            '/omniEthernet/pose',
            self.listener_callback,
            1)
        self.subscription  # prevent unused variable warning

        #Sub de la posicion de la rotacion del stylus del haptico
        self.twistSubscription = self.create_subscription(
            JointState,
            '/omniEthernet/joint_states',
            self.twitsCallback,
            1)
        self.twistSubscription  # prevent unused variable warning

        #Sub camara mano derecha
        self.rightCameraSub = self.create_subscription(
            Image,
            '/cameras/right_hand_camera/image',
            self.rightCameraCallback,
            1)
        self.rightCameraSub  # prevent unused variable warning

        #Sub camara mano izquierda
        self.leftCameraSub = self.create_subscription(
            Image,
            '/cameras/left_hand_camera/image',
            self.leftCameraCallback,
            1)
        self.leftCameraSub  # prevent unused variable warning

        #Pub de la info del haptico
        self.publisher_ = self.create_publisher(Float64MultiArray, 'haptic_info', 1)

        #Pub deteciones de la IA
        self.visionPublisher_ = self.create_publisher(Float64MultiArray, 'vision/detections', 1)

        timer_period = 0.1  # seconds

        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.timer2 = self.create_timer(timer_period, self.vision_callback)
        
    #Haptic button subscriber method
    def button_callback(self, msg):
        global connected

        if(connected):

            greyButton = "ON" if msg.data[0] else "OFF"

            whiteButton = "ON" if msg.data[1] else "OFF"

            rightGripper = "OPEN" if msg.data[4] else "CLOSE"

            leftGripper = "OPEN" if msg.data[5] else "CLOSE"

            self.updateRightGripLabel("Right gripper: " + rightGripper)

            self.updateLeftGripLabel("Left griper: " + leftGripper)

    #Funcion que recibe las detecciones y los puntos de la camra y devuelve la posicion en angulos del brazo del baxter del objeto
    def cameraDetectionsCallback(self, msgDetect, msgDepth):

        self.angles[0] = 0.0

        if not msgDetect.detections:
            #print(msgDetect.detections)
            return

        if  msgDetect.detections[0].results[0].score < 0.90 or msgDetect.detections[0].results[0].id != 'glass':
            #print(msgDetect.detections)
            return

        vision =  [0.0 for i in range(4)]

        vision[0] = msgDetect.detections[0].bbox.center.x
        vision[1] = msgDetect.detections[0].bbox.center.y
        vision[2] = 0.0 if msgDetect.detections[0].results[0].id == 'glass' else 1.0
        vision[3] = msgDetect.detections[0].results[0].score

        points = np.frombuffer(msgDepth.data, dtype=np.float32).reshape( msgDepth.height, msgDepth.width, -1)

        center_point = points[int(vision[1])][int(vision[0])]

        poseCamera = PoseStamped()

        poseCamera.header = msgDepth.header

        poseCamera.pose.position.x = float(center_point[0])
        poseCamera.pose.position.y = float(center_point[1])
        poseCamera.pose.position.z = float(center_point[2])

        poseCamera.pose.orientation.x = 0.0
        poseCamera.pose.orientation.y = 0.0
        poseCamera.pose.orientation.z = 0.0
        poseCamera.pose.orientation.w = 1.0

        pointBase = self.clientTfCalculator(poseCamera)
        
        self.angles = [pointBase.pose.pose.position.x, pointBase.pose.pose.position.y, pointBase.pose.pose.position.z, 0.0, 1.0, 0.0, 0.0]
        #añadir 3,5 cm a la profundidad y 2,1cm a la altura
        '''datoRed = [pointBase.pose.pose.position.x , pointBase.pose.pose.position.y, pointBase.pose.pose.position.z - 0.15, 0.0, 1.0, 0.0, 0.0]

        self.angles = list(self.red.predict([datoRed])[0]) 

        print(self.angles)
        
        for i in range(len(self.angles)):
            self.angles[i] = self.angles[i].item()'''


    def clientTfCalculator(self, point):

        self.cli = self.create_client(CalculatePosition, 'calculate_position')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.req = CalculatePosition.Request()

        self.req.pose = point
        self.req.frame_id = 'base'        

        response = self.cli.call(self.req)

        return response


    #Funcion que aniade la info del giro del haptico
    def twitsCallback(self, msg):

        #print(msg.position)
        self.info[4] = msg.position[3]
        self.info[5] = msg.position[4]
        self.info[7] = msg.position[5]

    #Haptic position subscriber method
    def listener_callback(self, msg):

        global connected

        if connected:

            self.info[0] = msg.pose.position.x
            self.info[1] = msg.pose.position.y
            self.info[2] = msg.pose.position.z

            self.info[3] = msg.pose.orientation.x
            self.info[4] = msg.pose.orientation.y

    #Haptic publisher method
    def timer_callback(self):

        global toZero
        global search
        
        msgSend = Float64MultiArray()
    
        self.info[6] = 1.0 if toZero else 0.0
        self.info[7] = 1.0 if side == 'Left' else 0.0
        self.info[8] = 1.0 if search else 0.0

        msgSend.data = self.info
        
        self.publisher_.publish(msgSend)

    #Funcion de publicacion de las detecciones
    def vision_callback(self):
        
        #if self.angles[0] == 0: return

        visionDetections = Float64MultiArray()

        visionDetections.data = self.angles

        self.visionPublisher_.publish(visionDetections)

    #Funcion de la camara del brazo derecho
    def rightCameraCallback(self, msg):

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        else:
            src = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            h, w, ch = src.shape
            bytesPerLine = ch * w
            qImg = QtGui.QImage(
            src.data, w, h, bytesPerLine, QtGui.QImage.Format_RGB888
            )
    
            self.updateRightCameraLabel(qImg)

    #Funcion de la camara del brazo izquierdo
    def leftCameraCallback(self, msg):

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        else:
            src = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            h, w, ch = src.shape
            bytesPerLine = ch * w
            qImg = QtGui.QImage(
            src.data, w, h, bytesPerLine, QtGui.QImage.Format_RGB888
            )
    
            self.updateLeftCameraLabel(qImg)

    def headCameraCallback(self, msg):

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        else:
            src = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            h, w, ch = src.shape
            bytesPerLine = ch * w
            qImg = QtGui.QImage(
            src.data, w, h, bytesPerLine, QtGui.QImage.Format_RGB888
            )
    
            self.updateHeadCameraLabel(qImg)


    #Method to map the max and min of haptic device
    def maxYMin(self, msg):

        self.comprobarPos(0, msg.pose.position.x)
        self.comprobarPos(1, msg.pose.position.y)
        self.comprobarPos(2, msg.pose.position.z)

        self.comprobarPos(3, msg.pose.orientation.x)
        self.comprobarPos(4, msg.pose.orientation.y)
        self.comprobarPos(5, msg.pose.orientation.z)

    #Auxiliar method to compare the max and min
    def comprobarPos(self, pos, valor):
 
        if min[pos] > valor:
            min[pos] = valor

        if max[pos] < valor:
            max[pos] = valor

    #Funcion de parada
    def stop():

        global connected

        global toZero

        global search
        
        connected = not connected

        toZero = False
        search = False

    #Funcion que devuelve al cero
    def toZero():

        global toZero

        global connected

        toZero = True

        connected = False

    #Funcion de busqueda
    def searchMode():

        global connected
        global search

        connected = False
        search = True

    def changeSide():

        global side

        side = 'Right' if side == 'Left' else 'Left' #Left or Right
