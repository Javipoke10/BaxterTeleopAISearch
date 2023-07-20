import rclpy
from rclpy.node import Node

import sys

import threading

from PyQt5 import uic, QtCore, QtGui, QtWidgets

from baxter_controller.hapticSub import MinimalSubscriber

from baxter_controller.robotInfo import GraphicInterface

app = QtWidgets.QApplication(sys.argv)

graph = GraphicInterface(app)

eventThread = threading.Event()

def main(args=None):

    print("Starting...")

    #Creating app ui
    #app = QtWidgets.QApplication(sys.argv)

    global app
    global graph
    
    graph.show()

    #Creating subscriber/publisher thread
    hiloSub = threading.Thread(target=startSub)

    hiloSub.start() 

    app.exec_()

    hiloSub.join()

    
    
def startSub():
    
    global graph

    global app

    rclpy.init()

    minimal_subscriber = MinimalSubscriber(graph.updateRightGripLabel, graph.updateLeftGripLabel, 
                                            graph.updateRightCameraLabel, graph.updateLeftCameraLabel,
                                            graph.updateHeadCameraLabel)

    minimal_subscriber.join_spin()
    rclpy.shutdown()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()

if __name__ == '__main__':
    main()