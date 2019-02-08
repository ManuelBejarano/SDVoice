#!/usr/bin/python
# -*- coding: utf-8 -*-
import sys
import time
import rospy
import socket
import roslaunch
import subprocess
from paramiko import client
from firebase import firebase
from PyQt5 import uic, QtGui, QtCore
from geometry_msgs.msg import PoseStamped
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QLineEdit, QGridLayout, QTextEdit, QPushButton, QMessageBox

# Custom Libraries
import includes as inc
from ros_utils import *

#load ui file
RELATIVE_DIR = './'
GUI_DIR = RELATIVE_DIR + './'
GUI_TITLE = 'Comandos SDV'
MAIN_WINDOW = GUI_DIR + 'home/sdvun/catkin_ws/src/SDVoice/resource/mainwindow.ui' #'mainwindow.ui'
UI_MAIN_WINDOW, Q_MAIN_WINDOW = uic.loadUiType(MAIN_WINDOW)

# Sockets connection
SERVER_PORT = 11311
SERVER_IP = ['192.168.1.11', '192.168.1.12', '192.168.1.13']
SERVER_USERNAME = ['sdvun1', 'sdvun2', 'sdvun3']
SERVER_PASSWORD = ['sdvun1', 'sdvun2', 'sdvun3']

#----------------------------------------------------
# Name: ssh
# Type: Class
# input args:
# output args: client for server connections
#----------------------------------------------------
class ssh():
    client = None

    def __init__(self, address, username, password):
        # Create a new SSH client
        self.client = client.SSHClient()
        # The following line is required if you want the script to be able to access a server that's not yet in the known_hosts file
        self.client.set_missing_host_key_policy(client.AutoAddPolicy())
        # Make the connection
        try:
            self.client.connect(address, username=username, password=password, look_for_keys=False)
            print self.client
        except:
            self.client = None

    def sendCommand(self, command):
        # Check if connection is made previously
        if(self.client):
            stdin, stdout, stderr = self.client.exec_command(command)
            while not stdout.channel.exit_status_ready():
                # Print stdout data when available
                if stdout.channel.recv_ready():
                    # Retrieve the first 1024 bytes
                    alldata = stdout.channel.recv(1024)
                    while stdout.channel.recv_ready():
                        # Retrieve the next 1024 bytes
                        alldata += stdout.channel.recv(1024)
                    # Print as string with utf8 encoding
                    print(str(alldata, "utf8"))
        else:
            #self.statusBar.showMessage("Connection not opened.")
            print("Connection not opened.")

#----------------------------------------------------
# Name: Sender
# Type: Class
# input args:
# output args: Rospy message publisher for robots
#----------------------------------------------------
class Sender():
    def __init__(self):
        # Crear mensaje PoseStamped
        self.pose = PoseStamped()
        self.pose.header.stamp = rospy.Time.now()
        self.pose.header.frame_id = "map"
        self.pose.pose.position.x = 0
        self.pose.pose.position.y = 0
        self.pose.pose.position.z = 0
        self.pose.pose.orientation.x = 0
        self.pose.pose.orientation.y = 0
        self.pose.pose.orientation.z = 0
        self.pose.pose.orientation.w = 1
        self.pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=100)

    def sendSDVtoCeldaExperimental(self):
        self.pose.header.stamp = rospy.Time().now()
        self.pose.pose.position.x = 0.8
        self.pose.pose.position.y = 5.8
        self.pose.pose.orientation.z = 0.0
        self.pose.pose.orientation.w = 0.1

    def sendSDVtoCeldaManufactura(self):
        self.pose.header.stamp = rospy.Time().now()
        self.pose.pose.position.x = 0.801138877869
        self.pose.pose.position.y = -4.6122341156
        self.pose.pose.orientation.z = 0.700009406104
        self.pose.pose.orientation.w = 0.714133622907

    def sendSDVtoCeldaIndustrial(self):
        self.pose.header.stamp = rospy.Time().now()
        self.pose.pose.position.x = 6.76584243774
        self.pose.pose.position.y = 3.13956570625
        self.pose.pose.orientation.z = 1.0
        self.pose.pose.orientation.w = 0.0

    def sendSDVtoDescargaMotoman(self):
        self.pose.header.stamp = rospy.Time().now()
        self.pose.pose.position.x = 6.76584243774
        self.pose.pose.position.y = 3.13956570625
        self.pose.pose.orientation.z = 1.0
        self.pose.pose.orientation.w = 0.0

    def publishPoseStamped(self):
        self.pub.publish(self.pose)

#----------------------------------------------------
# Name: SDVoice
# Type: Class
# input args: Q_MAIN_WINDOW and UI_MAIN_WINDOW
# output args: DOYViewer object
#----------------------------------------------------
class SDVoice(Q_MAIN_WINDOW, UI_MAIN_WINDOW):
    ''' Principal class in the way it contains the main GUI's objects '''
    roscore = Roscore()
    #----------------------------------------------------
    # Name: __init__
    # Type: Constructor
    # input args: None
    # output args: None
    #----------------------------------------------------
    def __init__(self):
        ''' Start window and declare initial values '''
        super(Q_MAIN_WINDOW,self).__init__()
        self.setupUi(self)
        self.comm = []
        self.EstadosDeConexion_model = QtGui.QStandardItemModel()
        self.EstadosDeConexion.setModel(self.EstadosDeConexion_model)
        self.TablaInstrucciones_model = QtGui.QStandardItemModel()
        self.TablaInstrucciones_model.setHorizontalHeaderLabels(['Robot', u'Acción', u'Máquina'])
        self.TablaInstrucciones.setModel(self.TablaInstrucciones_model)
        #print socket.gethostname()
        #IP1 = socket.gethostbyname(socket.gethostname()) # local IP adress of your computer
        #IP2 = socket.gethostbyname('name_of_your_computer') # IP adress of remote computer
        # Cargar complementos de la GUI
        self.ListaMaquinas.addItems(inc.PLACES) # Cargar lista de máquinas
        self.ListaInstrucciones.addItems(inc.COMMANDS) # Cargar lista de comandos
        # Definir callbacks de cada elemento de la gui
        self.Conectar.clicked.connect(lambda: self.connectSSH())
        self.Desconectar.clicked.connect(lambda: self.disconnectSSH())
        self.Anadir.clicked.connect(lambda: self.addManualCommand())
        self.Compilar.clicked.connect(lambda: self.sendCommand())
        #self.btn_sdv_experimental.clicked.connect(lambda: self.sdvGoToExperimental())
        #self.btn_sdv_manufactura.clicked.connect(lambda: self.sdvGoToManufactura())
        #self.btn_sdv_industrial.clicked.connect(lambda: self.sdvGoToIndustrial())
        
        # Lanzar ROS Core:
        self.roscore.run()
        
        # Inicializar nodo de envio de comandos
        rospy.init_node('sdv_commander', anonymous=True)
        
        # Crear Sender
        #self.sender = Sender()
        
        #rate = rospy.Rate(10) # 10hz
        #sender = Sender()
        #total_send = 1
        #n_send = 0
        #ready = False
        #while not rospy.is_shutdown():
    
        # Verificar lista de topicos
        #print rospy.client.get_published_topics()
        # Identificar los robots conectados
        
        # inicializar conexion
        self.initSocket()
        self.statusBar.showMessage("System Status | Ready. Welcome!")

    def initSocket(self):
        self.ssh_clients = [None for i in SERVER_USERNAME]
        # Agregar a ListaConexiones lista de hosts maquinas
        self.ListaConexiones.addItems(SERVER_USERNAME)

    def connectSSH(self):
        i = self.ListaConexiones.currentIndex()
        # Let the user know we're connecting to the server
        self.statusBar.showMessage("Connecting to server.")
        self.ssh_clients[i] = ssh(SERVER_IP[i], SERVER_USERNAME[i], SERVER_PASSWORD[i])
        # Agregar a EstadosDeConexion lista de hosts conectados
        if self.ssh_clients[i].client != None:
            # Lanzar el proyecto de sdvoice en el robot SDV
            #node = roslaunch.core.Node('sdvoice', 'agv_nav.launch')
            #launch = roslaunch.scriptapi.ROSLaunch()
            #launch.start()
            #process = launch.launch(node)
            #print process.is_alive()
            
            #self.ssh_clients[i].sendCommand("roslaunch sdvoice agv_nav.launch")
            #self.ssh_clients[i].sendCommand("ls")

            # Actualizar GUI
            item = QtGui.QStandardItem(self.ListaConexiones.currentText())
            self.EstadosDeConexion_model.appendRow(item)
            # NOTA: NO NECESARIO ---- > self.ListaMaquinas.addItem(self.ListaConexiones.currentText())
            # TODO: Agregar lista de comandos
        else:
            # QDialog de Warning por error de conexión
            # TODO: Cambiar tamaño de la ventana del mensaje
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Warning)
            msg.setText("Error de Conexión:")
            msg.setInformativeText("No se pudo conectar a %s!"%(self.ListaConexiones.currentText()))
            msg.setWindowTitle("Warning")
            ret = msg.exec_()
        self.statusBar.showMessage("")


    def disconnectSSH(self):
        for index in self.EstadosDeConexion.selectedIndexes():
            i = SERVER_USERNAME.index(index.data())
            self.ssh_clients[i].client.close()
            self.ssh_clients[i] = None
            self.EstadosDeConexion_model.removeRow(index.row())
            # TODO: Remover robot de la lista
            #self.ListaMaquinas.removeItem(self.ListaConexiones.currentText())
            
    def addManualCommand(self):
        indexes = self.EstadosDeConexion.selectedIndexes()
        # SI no se ha seleccionado ningun robot SDV arrojar error
        if not indexes:
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Warning)
            msg.setText(u"Error de Asignación de Comando:")
            msg.setInformativeText("Seleccione al menos un robot de la lista de conexiones")
            msg.setWindowTitle("Error")
            ret = msg.exec_()
        else:
            for index in indexes:
                sdv = index.data()
                self.comm = [QtGui.QStandardItem(str(sdv)), 
                        QtGui.QStandardItem(str(self.ListaInstrucciones.currentText())),
                        QtGui.QStandardItem(str(self.ListaMaquinas.currentText()))]
                self.TablaInstrucciones_model.appendRow(self.comm)
            
    def sendCommand(self):
        #if str(self.comm[0]) == 'sdvun2':
        print ':)'
        self.sender = Sender()
        self.sender.sendSDVtoCeldaExperimental()
    
    def closeEvent(self, event):
        # Cerrar todos los subprocesos abiertos
        self.roscore.terminate()

    
'''
        if self.ssh_clients[i].client != None:
            item = QtGui.QStandardItem(self.ListaConexiones.currentText())
            self.EstadosDeConexion_model.appendRow(item)
        else:
            # TODO: Agregar widget de error de conexion
            self.statusBar.showMessage("No se pudo conectar a %s!"%(self.ListaConexiones.currentText()))

    
    def sdvGoToExperimental(self):
        s = "rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped '\
header: \
    seq: 0\
    stamp: \
        secs: 0\
        nsecs: 0\
    frame_id \'map\'\
pose:\
    position:\
        x: 0.8\
        y: 5.8\
        z: 0\
    position:\
        x: 0\
        y: 0\
        z: 0\
        w: 1\
'\
"
        self.connection.sendCommand(s)
        print 'SDV ir a 1'

    def sdvGoToManufactura(self):
        self.connection.sendCommand('SDV ir a 2')
        print 'SDV ir a 2'

    def sdvGoToIndustrial(self):
        self.connection.sendCommand('SDV ir a 3')
        print 'SDV ir a 3'
'''


#----------------------------------------------------
# MAIN PROGRAM
#----------------------------------------------------
# Name: main
# Type: Main Function
# input args: None
# output args: None
#----------------------------------------------------
def main():
    '''
    Main function of execution for the application. 
    It generates the QT App and executes it 
    '''
    
    app = QApplication(sys.argv)
    app.setApplicationName(GUI_TITLE)
    gui = SDVoice()
    gui.show()
    sys.exit(app.exec_())
    
if __name__ == '__main__':
    main()
