from turtle import right
import rclpy

from rclpy.node import Node

from PyQt5.QtCore import QSize, Qt
from PyQt5 import uic, QtCore, QtGui, QtWidgets

from PyQt5.QtWidgets import QMainWindow, QPushButton, QGridLayout, QLabel, QVBoxLayout, QFrame

from PyQt5.QtGui import QColor, QFont

from PyQt5.QtMultimedia import QMediaContent, QMediaPlayer
from PyQt5.QtMultimediaWidgets import QVideoWidget

from baxter_controller.hapticSub import MinimalSubscriber

connected = True

side = "Left"

class GraphicInterface(QMainWindow):

    def __init__(self, app):

        #Inicio y ventana setteamos ventana inicial
        super().__init__()
        self.setWindowTitle("Baxter haptic teleop")

        #self.menuBar = self.menuBar()

        app.aboutToQuit.connect(self.toZero)
        wid = QtWidgets.QWidget()
        self.setCentralWidget(wid)

        self.setFixedSize(QSize(1800, 1000))

        mainFont = QFont()

        mainFont.setPointSize(16)

        mainFont.setBold(True)

        #Aniadimos labels de contenido y botones
        self.zeroButton = QPushButton("Move to zero")

        self.button = QPushButton("Disconnect haptic")

        self.searchButton = QPushButton("Search mode")

        self.changeArmButton = QPushButton("Change arm")

        self.linea = QLabel("STATUS: CONNECTED")

        self.linea.setFont(mainFont)

        self.linea.setStyleSheet('color : green;')

        self.controllingLabel = QLabel("Controlling: Left arm")

        self.controllingLabel.setFont(mainFont)

        #self.controllingLabel.setFont(QtGui.Qfont("arial", 22, QFont.bold))

        labelHaptic = QLabel("Haptic status")

        labelHaptic.setFont(mainFont)

        labelHaptic.setAutoFillBackground(True)

        #labelHaptic.setStyleSheet("background-color: blue;")

        labelGrip = QLabel("Grip status")

        labelGrip.setFont(mainFont)
        
        self.rightGripLabel = QLabel("Right grip: OPEN")

        self.rightGripLabel.setStyleSheet('color : green;')

        self.leftGripLabel = QLabel("Left grip: OPEN")

        self.leftGripLabel.setStyleSheet('color : green;')

        self.headCam = QLabel("No camera")

        self.armCam = QLabel("No camera")

        self.armCam.setStyleSheet("border: 1px green;")

        self.headCam.setStyleSheet("border: 1px red;")

        #Aniadimos layouts
        self.rightLay = QVBoxLayout()

        self.gridLay = QGridLayout()

        leftTopLay = QVBoxLayout()

        leftBotLay = QVBoxLayout()

        #Interaccion de los botones
        self.button.clicked.connect(self.botones)

        self.zeroButton.clicked.connect(self.toZero)

        self.searchButton.clicked.connect(self.search)

        self.changeArmButton.clicked.connect(self.changeSide)

        #Aniadimos los widget
        self.rightLay.addWidget(self.headCam, alignment=Qt.AlignCenter)

        self.rightLay.addWidget(self.controllingLabel, alignment=Qt.AlignCenter)

        self.rightLay.addWidget(self.changeArmButton, alignment=Qt.AlignCenter)        

        self.rightLay.addWidget(self.armCam, alignment=Qt.AlignCenter)

        #Aniadimos contenido al layout central
        leftTopLay.addWidget(self.zeroButton, alignment=Qt.AlignCenter)

        leftTopLay.addWidget(self.button, alignment=Qt.AlignCenter)

        leftTopLay.addWidget(self.searchButton, alignment=Qt.AlignCenter)

        #Aniadimos contenido al inferior izquierdo

        leftBotLay.addWidget(labelHaptic, alignment=Qt.AlignCenter)   

        leftBotLay.addWidget(self.linea, alignment=Qt.AlignCenter)           

        leftBotLay.addWidget(labelGrip, alignment=Qt.AlignCenter)

        leftBotLay.addWidget(self.rightGripLabel, alignment=Qt.AlignCenter)

        leftBotLay.addWidget(self.leftGripLabel, alignment=Qt.AlignCenter)

        #setted layout y aniadidos layouts auxiliares
        wid.setLayout(self.gridLay)  

        self.gridLay.addLayout(leftTopLay, 0, 0, 1, 1) 

        self.gridLay.addLayout(leftBotLay, 1, 0, 1, 1)

        self.gridLay.addLayout(self.rightLay, 0, 1, 2, 1, Qt.AlignCenter)   


    def botones(self):
        
        global connected

        connected = not connected

        if(connected):
            self.linea.setText("STATUS: CONNECTED")
            self.linea.setStyleSheet('color : green;')
            self.button.setText("Disconnect Haptic")
            
        else:
            self.button.setText("Connect Haptic")
            self.linea.setStyleSheet('color : red;')
            self.linea.setText("STATUS: DISCONNECTED")

        self.zeroButton.setEnabled(True)
        self.searchButton.setEnabled(True)
        #self.repaint()
        MinimalSubscriber.stop()

    def toZero(self):

        global connected 
        
        connected = False
        
        self.button.setText("Connect Haptic")
        self.zeroButton.setEnabled(False)
        self.linea.setText("STATUS: DISCONNECTED")
        self.linea.setStyleSheet('color : red;')

        #self.repaint()
        MinimalSubscriber.toZero()
    
    def search(self):

        global connected 
        global side

        side = 'Left'
        
        connected = False
        
        self.button.setText("Connect Haptic")
        self.linea.setText("STATUS: SEARCHING...")
        self.linea.setStyleSheet('color : blue;')
        self.searchButton.setEnabled(False)

        #self.repaint()
        MinimalSubscriber.searchMode()

    def updateRightGripLabel(self, text):
        self.rightGripLabel.setText(text)

        if 'OPEN' in text:
            self.rightGripLabel.setStyleSheet('color : green;')

        else:
            self.rightGripLabel.setStyleSheet('color : red;')

    def updateLeftGripLabel(self, text):
        self.leftGripLabel.setText(text)

        if 'OPEN' in text:
            self.leftGripLabel.setStyleSheet('color : green;')

        else:
            self.leftGripLabel.setStyleSheet('color : red;')

    def updateRightCameraLabel(self, imagen):
        global side
        
        if side == 'Right':

            '''self.gridLay.addWidget(self.rightCam, 1, 1, 1, 2, alignment=Qt.AlignCenter)
            self.rightCam.setPixmap(QtGui.QPixmap.fromImage(imagen).scaledToWidth(1000))
            return'''
            #self.leftCam.setStyleSheet("border: 1px red;")
            #self.rightCam.setStyleSheet("border: 1px green;")
        
        #self.gridLay.addWidget(self.rightCam, 1, 2, 1, 1, alignment=Qt.AlignCenter)
            self.armCam.setPixmap(QtGui.QPixmap.fromImage(imagen))

        

    def updateLeftCameraLabel(self, imagen):
        global side

        if side == 'Left':
            '''self.gridLay.addWidget(self.leftCam, 1, 0, 1, 2, alignment=Qt.AlignCenter)
            self.leftCam.setPixmap(QtGui.QPixmap.fromImage(imagen).scaledToWidth(1000))
            return'''
            #self.leftCam.setStyleSheet("border: 1px green;")
            #self.rightCam.setStyleSheet("border: 1px red;")
            
        
        #self.gridLay.addWidget(self.leftCam, 1, 0, 1, 1, alignment=Qt.AlignCenter)
            self.armCam.setPixmap(QtGui.QPixmap.fromImage(imagen))

    def updateHeadCameraLabel(self, imagen):
        self.headCam.setPixmap(QtGui.QPixmap.fromImage(imagen))

    def changeSide(self, newSide):
        global side

        side = 'Right' if side == 'Left' else 'Left' #Left or Right

        self.controllingLabel.setText("Controlling: "+side+" arm")

        MinimalSubscriber.changeSide()