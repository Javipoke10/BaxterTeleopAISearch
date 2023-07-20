#!/usr/bin/env python

import numpy as np
import rospy
import baxter_interface
import struct
import time

import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'

import tensorflow as tf

from std_msgs.msg import (
    Float64MultiArray,
    Int32MultiArray,
    Header
)
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

from omni_driver.msg import OmniButtonEvent

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

lado = 'left'
red = tf.keras.models.load_model('./src/baxter_cont/red/pesosBaxter.h5', compile=False)

fichero = open('coordenadas.txt', 'a')
ficheroSalida = open('angulos.txt', 'a')

limbRBax = 0
limbLBax = 0

flag = False

gripRBax=0
gripLBax=0

buttons = [0 for i in range(7)]

buttons[4] = 1
buttons[5] = 1

searchDone = False

search = False

poses = 0

zeroAngles = {
    "right":
        {'right_e0': -0.01572330307582549, 'right_e1': 2.5375877183594455, 'right_s0': 0.4049709280017492, 'right_s1': -1.050393344504537, 'right_w0': 0.07593204900032798, 'right_w1': 0.10239321759135137, 'right_w2': 3.043801378361632},
    "left":
        {'left_e0': 0.026461168591023387, 'left_e1': 2.492718780313797, 'left_s0': -0.4, 'left_s1': -0.9495341077010712, 'left_w0': -0.23700003172829642, 'left_w1': -0.07593204900032798, 'left_w2': 0.2373835269252678}
}

pub = 0

def callback(data):
    move(data.data)
    

def buttonCallback(data):

    global pub

    global buttons
    global lado

    buttons[0] = 1 if data.grey_button else 0
    buttons[1] = 1 if data.white_button else 0
    buttons[2] = 1 if data.grey_button_clicked else 0
    buttons[3] = 1 if data.white_button_clicked else 0
    buttons[6] = 1 if lado == 'right' else 0

    publicar = Int32MultiArray()

    publicar.data = buttons

    pub.publish(publicar) 

    pinza(data)

def pinza(data):

    global lado
    global buttons

    if (data.grey_button):
        if (lado == 'right'):
            lado = 'left'
        else:
            lado = 'right'

    gripBax = gripRBax if lado == 'right' else gripLBax
    
    if(data.white_button):

        buttons[4 if lado == "right" else 5] = 0 
        gripBax.close()

    else:

        buttons[4 if lado == "right" else 5] = 1
        gripBax.open()
    

def move(moves):

    global lado
    global limbRBax
    global limbLBax
    global poses
    global search

    orientacionPosterior = 0

    if moves[6]:
        toZero()
        return

    if moves[8]:
        search = True
        return
    else:
        search = False

    lado = 'left' if moves[7] else 'right'
    limbBax = limbRBax if lado == 'right' else limbLBax

    ns = "ExternalTools/" + lado + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()

    hdr = Header(stamp=rospy.Time.now(), frame_id='base')

    poses[lado].header = hdr

    if lado == 'left':

        poses[lado].pose.position.x = 0.895-(rescale(moves[1], 0.025993, 0.285218,0.05, 0.895))+0.1
        poses[lado].pose.position.y= (rescale(moves[0], -0.219272, 0.222985, -1.06, 0.23))+1.0
        poses[lado].pose.position.z= (rescale(moves[2], -0.037833, 0.348031, -0.60, 0.69))

    else:

        poses[lado].pose.position.x= 0.895 - (rescale(moves[1], 0.025993, 0.285218,0.05, 0.895))+0.1
        poses[lado].pose.position.y= (rescale(moves[0], -0.219272, 0.222985, -1.06, 0.23))
        poses[lado].pose.position.z= (rescale(moves[2], -0.037833, 0.348031, -0.60, 0.69))


    ikreq.pose_stamp.append(poses[lado])

    '''coords = []

    coords.append(poses[lado].pose.position.x)
    coords.append(poses[lado].pose.position.y)
    coords.append(poses[lado].pose.position.z)
    coords.append(poses[lado].pose.orientation.x)
    coords.append(poses[lado].pose.orientation.y)
    coords.append(poses[lado].pose.orientation.z)
    coords.append(poses[lado].pose.orientation.w)

    ang = red.predict([coords])[0]

    searchCords = {
    "left":
        {'left_e0': ang[0], 
        'left_e1': ang[1], 
        'left_s0': ang[2], 
        'left_s1': ang[3], 
        'left_w0': ang[4], 
        'left_w1': ang[5], 
        'left_w2': ang[6]}
    }

    limbBax.set_joint_positions(searchCords['left'])'''

    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException) as e:
        rospy.logerr("Service call failed: %s" % (e,))
        return 1

    resp_seeds = struct.unpack('<%dB' % len(resp.result_type), resp.result_type)

    if (resp_seeds[0] != resp.RESULT_INVALID):
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        limbBax.set_joint_positions(limb_joints)
        


def rescale(var, minVar, maxVar, minRange, maxRange):

    res = var 
    acum = 0

    if(minVar < 0):
        res += -minVar
        maxVar += -minVar
        minVar = 0

    if(minRange < 0):
        maxRange += -minRange
        acum = -minRange
        minRange = 0

    return res*maxRange/maxVar-acum

def toZero():

    global zeroAngles

    global limbRBax
    global limbLBax

    limbLBax.move_to_joint_positions(zeroAngles['left'])
    limbRBax.move_to_joint_positions(zeroAngles['right'])

def searchCallback(data):

    global search

    if(search):
        searchMode(data.data)


def searchMode(coords):

    print("searching...")
    toZero()

    if coords[0] == 0:
        print("No detection")
        return

    #enfrentar(datos())

    poses = [0 for i in range(7)]

    poses[0] = coords[0]
    poses[1] = coords[1]
    poses[2] = coords[2]
    poses[3] = coords[3]
    poses[4] = coords[4]
    poses[5] = coords[5]
    poses[6] = coords[6]

    #dataRecord()
    global zeroAngles
    global gripLBax
    global red

    gripLBax.open()
    poses[2] = coords[2] + 0.08
    angulos = list(red.predict([poses])[0])

    searchCords = {
    "left":
        {'left_e0': angulos[0], 
        'left_e1': angulos[1], 
        'left_s0': angulos[2], 
        'left_s1': angulos[3], 
        'left_w0': angulos[4], 
        'left_w1': angulos[5], 
        'left_w2': angulos[6]}
    }

    limbLBax.move_to_joint_positions(searchCords['left'])

    poses[0] = coords[0] + 0.02
    poses[1] = coords[1] + 0.02
    poses[2] = coords[2] - 0.1

    angulos = list(red.predict([poses])[0])

    searchCords = {
    "left":
        {'left_e0': angulos[0], 
        'left_e1': angulos[1], 
        'left_s0': angulos[2], 
        'left_s1': angulos[3], 
        'left_w0': angulos[4], 
        'left_w1': angulos[5], 
        'left_w2': angulos[6]}
    }
    
    limbLBax.move_to_joint_positions(searchCords['left'])
    gripLBax.close()

    time.sleep(0.1)

    poses[2] = coords[2] + 0.1

    angulos = list(red.predict([poses])[0])

    searchCords = {
    "left":
        {'left_e0': angulos[0], 
        'left_e1': angulos[1], 
        'left_s0': angulos[2], 
        'left_s1': angulos[3], 
        'left_w0': angulos[4], 
        'left_w1': angulos[5], 
        'left_w2': angulos[6]}
    }

    limbLBax.move_to_joint_positions(searchCords['left'])

    searchCords = {
    "left":{'left_e0': 0.5399612373356656, 'left_e1': 2.0923497946757044, 'left_s0': -0.75, 'left_s1': -0.6937428113211783, 'left_w0': -1.2041749184900499, 'left_w1': 0.40305345201689247, 'left_w2': 0.8256651590793239}
    }

    limbLBax.move_to_joint_positions(searchCords['left'])

    gripLBax.open()

    time.sleep(1)
    
    searchCords = {
    "left":{'left_e0': 0.5399612373356656, 'left_e1': 2.0923497946757044, 'left_s0': -0.75, 'left_s1': -0.9495341077010712, 'left_w0': -1.2041749184900499, 'left_w1': 0.40305345201689247, 'left_w2': 0.8256651590793239}
    }

    limbLBax.move_to_joint_positions(searchCords['left'])
    
    time.sleep(1)

    toZero()

    time.sleep(5)

    '''
    gripLBax.open()


    ns = "ExternalTools/" + "left" + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()

    hdr = Header(stamp=rospy.Time.now(), frame_id='base')

    
    poses = {
        'left': PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x= angulos[0],
                    y= angulos[1],
                    z= angulos[2],
                ),
                orientation=Quaternion(
                    x=0,
                    y=1,
                    z=0,
                    w=0,
                        
                ),
            )
                    ),
    }

    poses[lado].header = hdr

    ikreq.pose_stamp.append(poses[lado])

    poses[lado].pose.position.x = angulos[0]
    poses[lado].pose.position.y= angulos[1]
    poses[lado].pose.position.z= angulos[2]

    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException) as e:
        rospy.logerr("Service call failed: %s" % (e,))
        return 1

    resp_seeds = struct.unpack('<%dB' % len(resp.result_type), resp.result_type)

    if (resp_seeds[0] != resp.RESULT_INVALID):
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        limbLBax.move_to_joint_positions(limb_joints)
        #gripLBax.close()
        time.sleep(0.1)
    else:
        print('Invalido')

    poses[lado].pose.position.x = angulos[0]+0.05
    poses[lado].pose.position.y= angulos[1]
    poses[lado].pose.position.z= angulos[2]-0.1


    ikreq.pose_stamp.append(poses[lado])

    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException) as e:
        rospy.logerr("Service call failed: %s" % (e,))
        return 1

    resp_seeds = struct.unpack('<%dB' % len(resp.result_type), resp.result_type)

    if (resp_seeds[0] != resp.RESULT_INVALID):
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        limbLBax.move_to_joint_positions(limb_joints)
        gripLBax.close()
        time.sleep(0.1)
    else:
        print('Invalido')

    final={
        'left':{'left_e0': 1.124024422323037, 
        'left_e1': 1.189218605808167, 
        'left_s0': -0.6481068828815874, 
        'left_s1': 1.0419564501711673, 
        'left_w0': 0.3474466484560462, 
        'left_w1': 0.5829126993964572, 
        'left_w2': -3.0583741958465436}
    }

    limbLBax.move_to_joint_positions(zeroAngles['left'])

    gripLBax.open()

    time.sleep(0.4) '''


def dataRecord():
               
    print(limbLBax.endpoint_pose())

    global fichero

    inputPose = limbLBax.endpoint_pose()

    outputAngles= limbLBax.joint_angles()

    input = '{};{};{};{};{};{};{}'.format(inputPose["position"].x, inputPose["position"].y, inputPose["position"].z, inputPose["orientation"].x, inputPose["orientation"].y, inputPose["orientation"].z, inputPose["orientation"].w)
    
    output='{};{};{};{};{};{};{}'.format(outputAngles['left_e0'], outputAngles['left_e1'], outputAngles['left_s0'], outputAngles['left_s1'], outputAngles['left_w0'], outputAngles['left_w1'], outputAngles['left_w2'])

    fichero.write(input)
    fichero.write('\n')

    ficheroSalida.write(output)
    ficheroSalida.write('\n')

    time.sleep(0.01)


def controller():

    rospy.init_node('listener', anonymous=True) 

    global limbRBax, limbLBax

    if limbRBax == 0:
        limbRBax = baxter_interface.Limb('right')

    if limbLBax == 0:
        limbLBax = baxter_interface.Limb('left')

    global poses

    if poses == 0:

        hdr = Header(stamp=rospy.Time.now(), frame_id='base')

        poses = {
            'left': PoseStamped(
                header=hdr,
                pose=Pose(
                    position=Point(
                        x= 0.995,
                        y= 1.0,
                        z= 0,
                    ),
                    orientation=Quaternion(
                        x=0,
                        y=1,
                        z=0,
                        w=0,
                        
                    ),
                )
                        ),
            'right': PoseStamped(
                header=hdr,
                pose=Pose(
                    position=Point(
                        x= 0.995,
                        y= 0,
                        z= 0,
                    ),
                    orientation=Quaternion(
                        x=0,
                        y=1,
                        z=0,
                        w=0,
                        
                    ),
                ),
            ),
        }

    global gripRBax
    global gripLBax
    global pub

    if gripRBax == 0: 
        gripRBax = baxter_interface.Gripper('right')

    if gripLBax == 0:
        gripLBax = baxter_interface.Gripper('left') 

    if(not gripRBax.calibrated()):
            gripRBax.calibrate()
            print("Calibrating right...")

    if(not gripLBax.calibrated()):
            gripLBax.calibrate()
            print("Calibrating left...")

    rospy.Subscriber("/haptic_info", Float64MultiArray, callback, queue_size=1)
    rospy.Subscriber("/omniEthernet/button_state", OmniButtonEvent, buttonCallback, queue_size=1)
    rospy.Subscriber("/vision/detections", Float64MultiArray, searchCallback, queue_size=1)

    if pub == 0:
        pub = rospy.Publisher("Button_info", Int32MultiArray,queue_size=1)

    rospy.spin()

#A partir de aqui es experimento
def datos():

    input = open('coordenadas.txt' ,'r')

    readed = input.readline().replace('\n', '').split(';')

    entreno = []

    while readed != ['']:

        readed = [float(i) for i in readed]

        entreno.append(readed)

        readed = input.readline().replace('\n', '').split(';')

    return entreno

def inverseCinematic(punto):

    ns = "ExternalTools/" + "left" + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()

    hdr = Header(stamp=rospy.Time.now(), frame_id='base')

    
    poses = {
        'left': PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x= punto[0],
                    y= punto[1],
                    z= punto[2],
                ),
                orientation=Quaternion(
                    x=punto[3],
                    y=punto[4],
                    z=punto[5],
                    w=punto[6],
                        
                ),
            )
                    ),
    }

    poses[lado].header = hdr

    ikreq.pose_stamp.append(poses[lado])

    poses[lado].pose.position.x = punto[0]
    poses[lado].pose.position.y = punto[1]
    poses[lado].pose.position.z = punto[2]

    poses[lado].pose.orientation.x = punto[3]
    poses[lado].pose.orientation.y = punto[4]
    poses[lado].pose.orientation.z = punto[5]
    poses[lado].pose.orientation.w = punto[6]

    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException) as e:
        rospy.logerr("Service call failed: %s" % (e,))
        return 1

    resp_seeds = struct.unpack('<%dB' % len(resp.result_type), resp.result_type)

    if (resp_seeds[0] != resp.RESULT_INVALID):
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        
    else:
        print('Invalido')

    return limb_joints
    
def moveIA(punto):

    angulos = list(red.predict([punto], verbose=0, use_multiprocessing=True)[0])

    for i in range(len(angulos)):
        angulos[i] = angulos[i].item()

    searchCords = {
    "left":
        {'left_e0': angulos[0], 
        'left_e1': angulos[1], 
        'left_s0': angulos[2], 
        'left_s1': angulos[3], 
        'left_w0': angulos[4], 
        'left_w1': angulos[5], 
        'left_w2': angulos[6]}
    }

    return searchCords['left']



def enfrentar(listaOrigen):

    global flag

    if flag:
        return

    lista = []

    for i in range(0, 400, 4):
        lista.append(listaOrigen[i])

    print("Enfrentamiento")

    tiemposCine = open('tiemposCine.txt', 'a')
    tiemposIA = open('tiemposIA.txt', 'a')

    toZero()

    global red
    print("Cinematica")

    #cinematica
    '''for i in lista:
        print(lista.index(i))
        startTime = time.perf_counter_ns()

        #coord a angulos
        limb_joints = inverseCinematic(i)

        tiempoAngulos = time.perf_counter_ns() - startTime

        #ir al punto
        limbLBax.move_to_joint_positions(limb_joints)

        tiempoPunto = time.perf_counter_ns() - startTime

        linea = '{};{}'.format(tiempoAngulos, tiempoPunto)

        tiemposCine.write(linea)
        tiemposCine.write('\n')'''

    toZero()
    print("IA")
    #ia
    
    for i in lista:
        print(lista.index(i))
        startTime = time.perf_counter_ns()

        #coord a angulos

        angles = moveIA(i)

        tiempoAngulos = time.perf_counter_ns() - startTime

        #ir al punto
        limbLBax.move_to_joint_positions(angles)

        tiempoPunto = time.perf_counter_ns() - startTime

        linea = '{};{}'.format(tiempoAngulos, tiempoPunto)

        tiemposIA.write(linea)
        tiemposIA.write('\n')

    flag = True

    tiemposCine.close()
    tiemposIA.close()


if __name__ == '__main__':
    rospy.loginfo("Starting...")

    red.compile()

    controller()
