#!/usr/bin/python
# -*- coding: utf-8 -*-
import os
import rospy
import rospkg
from paramiko import client
from qt_gui.plugin import Plugin
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray
from python_qt_binding import QtGui, QtCore, loadUi
from python_qt_binding.QtWidgets import QWidget, QMessageBox


#import sys
#import time
#import rospy
#import socket
#import roslaunch
#import subprocess
#from firebase import firebase
#from PyQt5 import QtGui, QtCore
#from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QLineEdit, QGridLayout, QTextEdit, QPushButton, QMessageBox

# Custom Libraries
import includes as inc
#from ros_utils import *

# Sockets connection
SERVER_PORT = 11311
SERVER_IP = ['192.168.1.11', '192.168.1.12', '192.168.1.13']
SERVER_USERNAME = ['sdvun1', 'sdvun2', 'sdvun3']
SERVER_PASSWORD = ['sdvun1', 'sdvun2', 'sdvun3']

SDVUN_LAST_COMMAND = [[], [], []]

#----------------------------------------------------
# Name: SDV
# Type: Class
# input args:
# output args: client for server connections
#----------------------------------------------------
class SDV():
    def __init__(self, name, address, username, password):
        self.name = name
        self.address = address
        self.username = username
        self.password = password
        # Objetos
        self.ssh = None
        self.sender = Sender(name)

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
    def __init__(self, name):
        self.status = ''
        self.name = name
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
        self.pub = rospy.Publisher('/%s/move_base_simple/goal'%self.name, PoseStamped, queue_size=100)
        self.subs = rospy.Subscriber('/%s/move_base/status'%self.name, GoalStatusArray, self.callbackStatus)

    def sendSDVtoHome(self):
        self.pose.header.stamp = rospy.Time().now()
        self.pose.pose.position.x = 0.0
        self.pose.pose.position.y = 0.0
        self.pose.pose.orientation.z = 0.0
        self.pose.pose.orientation.w = 1.0

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
        print self.pose
        
    def callbackStatus(self, status):
        print status
        for stat in status.status_list:
            if (stat == '1') and (self.status != stat): print '%s: CAMINANDO'%self.name#, stat.text
            #elif stat == '2': print '%s: NUEVA POSE RECIBIDA'%self.name#, stat.text
            elif (stat == '3') and (self.status != stat): print u'%s: LLEGUÉ!'%self.name#, stat.text
            # SDV no llega a la posición deseada
            elif (stat == '4') and (self.status != stat):
                self.status = stat
                print '%s: NO LLEGO'%self.name#, stat.text
                # TODO: LLamar a publicar firebase
        
    def SDVisMoving(self, stat):
        # New status detected
        if self.status != stat:

#----------------------------------------------------
# Name: SDVoice
# Type: Class
# input args: Q_MAIN_WINDOW and UI_MAIN_WINDOW
# output args: DOYViewer object
#----------------------------------------------------
class SDVoice():
    ''' Principal class in the way it contains the main GUI's objects '''
    # Create QWidget
    _comm = []
    _stack = []
    _widget = QWidget()
    
    #----------------------------------------------------
    # Name: __init__
    # Type: Constructor
    # input args: None
    # output args: None
    #----------------------------------------------------
    def __init__(self, ui_file):
        ''' Start window and declare initial values '''
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('SDVoice')
        self.EstadosDeConexion_model = QtGui.QStandardItemModel()
        self._widget.EstadosDeConexion.setModel(self.EstadosDeConexion_model)
        self.TablaInstrucciones_model = QtGui.QStandardItemModel()
        self.TablaInstrucciones_model.setHorizontalHeaderLabels(['Robot', u'Acción', u'Máquina'])
        self._widget.TablaInstrucciones.setModel(self.TablaInstrucciones_model)
        # Cargar complementos de la GUI
        self._widget.ListaMaquinas.addItems(inc.PLACES) # Cargar lista de máquinas
        self._widget.ListaInstrucciones.addItems(inc.COMMANDS) # Cargar lista de comandos
        # Definir callbacks de cada elemento de la gui
        self._widget.Conectar.clicked.connect(lambda: self.connectSSH())
        self._widget.Desconectar.clicked.connect(lambda: self.disconnectSSH())
        self._widget.Anadir.clicked.connect(lambda: self.addManualCommand())
        self._widget.Stop.clicked.connect(lambda: self.sendCommand())
        #self.btn_sdv_experimental.clicked.connect(lambda: self.sdvGoToExperimental())
        #self.btn_sdv_manufactura.clicked.connect(lambda: self.sdvGoToManufactura())
        #self.btn_sdv_industrial.clicked.connect(lambda: self.sdvGoToIndustrial())
        
        # Crear robots SDV
        self.sdvs = []
        for i, uname in enumerate(SERVER_USERNAME):
            self.sdvs.append(SDV(SERVER_USERNAME[i], SERVER_IP[i], SERVER_USERNAME[i], SERVER_PASSWORD[i]))
            #self.sdvs[i].sender = Sender(SERVER_USERNAME[i])
        # inicializar conexion
        self.initSocket()
        self._widget.statusBar.showMessage("System Status | Ready. Welcome!")
    
    
    def initSocket(self):
        #for i, name in enumerate(SERVER_USERNAME):
        #    self.sdvs[i].ssh = None
        # Agregar a ListaConexiones lista de hosts maquinas
        self._widget.ListaConexiones.addItems(SERVER_USERNAME)
    
    
    def connectSSH(self):
        i = self._widget.ListaConexiones.currentIndex()
        # Let the user know we're connecting to the server
        self._widget.statusBar.showMessage("Connecting to server.")
        self.sdvs[i].ssh = ssh(SERVER_IP[i], SERVER_USERNAME[i], SERVER_PASSWORD[i])
        # Agregar a EstadosDeConexion lista de hosts conectados
        if self.sdvs[i].ssh.client != None:
            # TODO: Lanzar el proyecto de sdvoice en el robot SDV
            #node = roslaunch.core.Node('sdvoice', 'agv_nav.launch')
            #launch = roslaunch.scriptapi.ROSLaunch()
            #launch.start()
            #process = launch.launch(node)
            #print process.is_alive()
            #self.ssh_clients[i].sendCommand("roslaunch sdvoice agv_nav.launch")
            #self.ssh_clients[i].sendCommand("ls")

            # Actualizar GUI
            item = QtGui.QStandardItem(self._widget.ListaConexiones.currentText())
            self.EstadosDeConexion_model.appendRow(item)
            # NOTA: NO NECESARIO ---- > self.ListaMaquinas.addItem(self.ListaConexiones.currentText())
            # TODO: Agregar lista de comandos
        else:
            # QDialog de Warning por error de conexión
            # TODO: Cambiar tamaño de la ventana del mensaje
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Warning)
            msg.setText("Error de Conexión:")
            msg.setInformativeText("No se pudo conectar a %s!"%(self._widget.ListaConexiones.currentText()))
            msg.setWindowTitle("Warning")
            ret = msg.exec_()
        self._widget.statusBar.showMessage("")

    def disconnectSSH(self):
        for index in self._widget.EstadosDeConexion.selectedIndexes():
            i = SERVER_USERNAME.index(index.data())
            self.sdvs[i].ssh.client.close()
            self.sdvs[i].ssh = None
            self.EstadosDeConexion_model.removeRow(index.row())
            # TODO: Remover robot de la lista
            #self.ListaMaquinas.removeItem(self.ListaConexiones.currentText())
         
    def addManualCommand(self):
        indexes = self._widget.EstadosDeConexion.selectedIndexes()
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
                self._comm = [QtGui.QStandardItem(str(sdv)), 
                        QtGui.QStandardItem(str(self._widget.ListaInstrucciones.currentText())),
                        QtGui.QStandardItem(str(self._widget.ListaMaquinas.currentText()))]
                self.TablaInstrucciones_model.appendRow(self._comm)
       
    def sendCommand(self):
        name, instr, place = sorted(self._widget.TablaInstrucciones.selectedIndexes())
        name, place = str(name.data()), str(place.data())
        
        # TODO: If robot not connected, show error
        i = 0
        if name == SERVER_USERNAME[0]:   i=0#SDVUN_LAST_COMMAND[0] = self.sender.pose
        elif name == SERVER_USERNAME[1]: i=1#SDVUN_LAST_COMMAND[1] = self.sender.pose
        elif name == SERVER_USERNAME[2]: i=2#SDVUN_LAST_COMMAND[2] = self.sender.pose
    
        
        # HOME
        if inc.PLACES[place] == 0: self.sdvs[i].sender.sendSDVtoHome()
        # MALLA
        elif inc.PLACES[place] == 1: self.sdvs[i].sender.sendSDVtoCeldaExperimental()
        # INDUSTRIAL
        elif inc.PLACES[place] == 2: self.sdvs[i].sender.sendSDVtoCeldaIndustrial()
        # MANUFACTURA
        elif inc.PLACES[place] == 3: self.sdvs[i].sender.sendSDVtoCeldaManufactura()
        else:
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Warning)
            msg.setText(u"Error de Asignación de Comando:")
            msg.setInformativeText("No se ha reconocido el destino del SDV")
            msg.setWindowTitle("Error")
            ret = msg.exec_()
        
        # TODO: If robot not connected, show error
        if name == SERVER_USERNAME[0]: self.sdvs[i].sender.publishPoseStamped()
        elif name == SERVER_USERNAME[1]: self.sdvs[i].sender.publishPoseStamped()
        elif name == SERVER_USERNAME[2]: self.sdvs[i].sender.publishPoseStamped()
    
    def closeEvent(self, event):
        # Cerrar todos los subprocesos abiertos
        #self.roscore.terminate()
        return


class Plugin(Plugin):

    def __init__(self, context):
        super(Plugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('Plugin')
        
        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create SDVoice
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_sdvoice'), 'resource', 'Plugin.ui')
        self.sdvoice = SDVoice(ui_file)
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self.sdvoice._widget.setWindowTitle(self.sdvoice._widget.windowTitle() + (' (%d)' % context.serial_number()))
            
        # TODO: Add Console
        # TODO: Add Rviz

        
        # Add widget to the user interface
        #context.add_widget(self._widget)
        context.add_widget(self.sdvoice._widget)

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
