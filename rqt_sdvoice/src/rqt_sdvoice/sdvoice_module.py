#!/usr/bin/python
# -*- coding: utf-8 -*-
import os
import time
import rospy
import rospkg
from paramiko import client
from qt_gui.plugin import Plugin
from time import gmtime, strftime
from sensor_msgs.msg import Image
from move_base_msgs.msg import MoveBaseActionGoal
from geometry_msgs.msg import PoseStamped, Twist
from actionlib_msgs.msg import GoalStatusArray, GoalID
from python_qt_binding import QtGui, QtCore, loadUi
from python_qt_binding.QtWidgets import QWidget, QMessageBox, QLineEdit    

# Custom Libraries
import includes as inc
from utils_firebase import Firebase

firebase = Firebase()
SDVUN_LAST_COMMAND = [[], [], []]

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
    def __init__(self, name, user):
        self.user = user
        self.timer, self.delta = 0., time.clock()
        self.poses, self.gids = [], []
        self.status = 0
        self.name = name
        self.key = 'moving'
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
        self.poses.append(self.pose)
        self.pub = rospy.Publisher('/%s/move_base_simple/goal'%self.name, PoseStamped, queue_size=100)
        self.cancel = GoalID()
        self.cancel.stamp = rospy.Time.now()
        self.cancel.id = '/%s/HOME'%self.name
        self.gids.append(self.cancel)
        self.pub_cancel = rospy.Publisher('/%s/move_base/cancel'%self.name, GoalID, queue_size=100)
        self.subs = rospy.Subscriber('/%s/move_base/status'%self.name, GoalStatusArray, self.callbackStatus)
        self.subs_goal = rospy.Subscriber('/%s/move_base/goal'%self.name, MoveBaseActionGoal, self.newGoalID)
        self.subs_img = rospy.Subscriber('/%s/camera/rgb/image_raw'%self.name, Image, self.newImage)

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
        self.status = 0
        self.key = firebase.addData(self.user.display_name, self.name, str(self.pose.header.stamp), self.pose.pose.position,self.pose.pose.orientation,'moving')
        print self.poses[-1]
        
    def publishCancelGoal(self):
        # Cancelar último objetivo
        print self.gids[-1]
        self.pub_cancel.publish(self.gids[-1])
        firebase.updateTask(self.user.display_name, self.name, self.key, 'canceled')
        
    def publishResetGoal(self):
        self.status = 0
        self.pub.publish(self.pose)
        self.publishPoseStamped()
        firebase.updateTask(self.user.display_name, self.name, self.key, 'reseted')
        
    def newGoalID(self, status):
        self.gids.append(status.goal_id)
        self.poses.append(status.goal.target_pose)
        
    def newImage(self, status):
        pass
        #self.gids.append(status.goal_id)
        #self.poses.append(status.goal.target_pose)
        
    def callbackStatus(self, status):
        for stats in status.status_list:
            stat = stats.status 
            # SDV ha llegado a la posición deseada
            if (stat == 3) and (self.status != stat):
                self.status = stat
                print u'%s: LLEGUÉ!'%self.name#, stat.text
                # Crear mensaje con status complete
                # ADD USERNAME AFTER LOGIN
                firebase.updateTask(self.user.display_name, self.name, self.key, 'complete')
                self.timer = 0.0
            # SDV no llega a la posición deseada
            elif (stat == 4) and (self.status != stat):
                self.status = stat
                print '%s: NO LLEGO'%self.name#, stat.text
                firebase.updateTask(self.user.display_name, self.name, self.key, 'failed')
                self.pose = self.poses[-2]
                self.publishPoseStamped()
                self.timer = 1.0
            #else: print 'ahi voy'

#----------------------------------------------------
# Name: SDV
# Type: Class
# input args:
# output args: client for server connections
#----------------------------------------------------
class SDV():
    def __init__(self, name, address, username, password, user):
        self.user = user
        self.name = name
        self.address = address
        self.username = username
        self.password = password
        # Objetos
        self.ssh = None
        self.sender = Sender(name, self.user)
      
#----------------------------------------------------
# Name: Login
# Type: Class
# input args: Q_MAIN_WINDOW and UI_MAIN_WINDOW
# output args: DOYViewer object
#----------------------------------------------------
class Login():
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
        self.user = None
        # Give QObjects reasonable names
        self._widget.setObjectName('Login')
        self._widget.Password.setEchoMode(QLineEdit.Password)
        # Definir callbacks de cada elemento de la gui
        self._widget.Entrar.clicked.connect(lambda: self.login())
        self._widget.Registrarse.clicked.connect(lambda: self.registro())
        
    def login(self):
        email = self._widget.Usuario.text()
        passw = self._widget.Password.text()
        self.user = firebase.login(email, passw, firebase)
    
    def registro(self):
        self.user = []
                         
#----------------------------------------------------
# Name: Registro
# Type: Class
# input args: Q_MAIN_WINDOW and UI_MAIN_WINDOW
# output args: DOYViewer object
#----------------------------------------------------
class Registro():
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
        self.user = None
        # Give QObjects reasonable names
        self._widget.setObjectName('Registro')
        self._widget.passw.setEchoMode(QLineEdit.Password)
        self._widget.repass.setEchoMode(QLineEdit.Password)
        # Definir callbacks de cada elemento de la gui
        self._widget.Cancelar.clicked.connect(lambda: self.cancelar())
        self._widget.Registrarse.clicked.connect(lambda: self.registro())
    
    def registro(self):
        email = self._widget.email.text()
        user_name = self._widget.userName.text()
        phone = self._widget.phone.text()
        passw = self._widget.passw.text()
        repassw = self._widget.repass.text()
        # Error de contraseñas
        if passw != repassw:
            self.user = None
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Warning)
            msg.setText(u"Error de Contraseña:")
            msg.setInformativeText(u"Las contraseñas no coinciden")
            msg.setWindowTitle("Error")
            ret = msg.exec_()
        else:
            try:
                self.user = firebase.addUser(email, passw, phone, user_name)
                firebase.newDataBase(self.user.display_name)
            # Error de registro
            except:
                self.user = None
                msg = QMessageBox()
                msg.setIcon(QMessageBox.Warning)
                msg.setText(u"Error de Registro:")
                msg.setInformativeText(u"No se ha podido registrar usuario: \n Datos erroneos, revisar:\n - Contraseña debe tener más de 6 digitos.\n - Verifique símbolos de los datos personales.")
                msg.setWindowTitle("Error")
                ret = msg.exec_()
    
    def cancelar(self):
        self.user = []
                
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
    def __init__(self, ui_file, user):
        ''' Start window and declare initial values '''
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        self.user = user
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
        self._widget.Ejectutar.clicked.connect(lambda: self.sendCommand())
        self._widget.Stop.clicked.connect(lambda: self.stopSDV())
        self._widget.Reset.clicked.connect(lambda: self.resetSDV())
        
        # LOGIN Y OTRAS COSAS
        
        # Crear robots SDV
        self.sdvs = []
        for i, uname in enumerate(inc.SERVER_USERNAME):
            self.sdvs.append(SDV(inc.SERVER_USERNAME[i], inc.SERVER_IP[i], 
                                 inc.SERVER_USERNAME[i], inc.SERVER_PASSWORD[i], self.user)
                             )
        # inicializar conexion con SDVs
        self.initSocket()
        self._widget.statusBar.showMessage("System Status | Ready. Welcome!")
    
    
    def initSocket(self):
        # Agregar a ListaConexiones lista de hosts maquinas
        self._widget.ListaConexiones.addItems(inc.SERVER_USERNAME)
    
    
    def connectSSH(self):
        i = self._widget.ListaConexiones.currentIndex()
        # Let the user know we're connecting to the server
        self._widget.statusBar.showMessage("Connecting to server.")
        self.sdvs[i].ssh = ssh(inc.SERVER_IP[i], inc.SERVER_USERNAME[i], inc.SERVER_PASSWORD[i])
        # Agregar a EstadosDeConexion lista de hosts conectados
        if self.sdvs[i].ssh.client != None:
            # Actualizar GUI
            item = QtGui.QStandardItem(self._widget.ListaConexiones.currentText())
            self.EstadosDeConexion_model.appendRow(item)
            
            # TODO: Lanzar el proyecto de sdvoice en el robot SDV
            #node = roslaunch.core.Node('sdvoice', 'agv_nav.launch')
            #launch = roslaunch.scriptapi.ROSLaunch()
            #launch.start()
            #process = launch.launch(node)
            #print process.is_alive()
            #self.ssh_clients[i].sendCommand("roslaunch sdvoice agv_nav.launch")
            #self.ssh_clients[i].sendCommand("ls")

        else:
            # QDialog de Warning por error de conexión
            # TODO: Cambiar tamaño de la ventana del mensaje
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Warning)
            msg.setText("Error de Conexión:")
            msg.setInformativeText("No se pudo conectar a %s!"%(self._widget.ListaConexiones.currentText()))
            msg.setWindowTitle("Error")
            ret = msg.exec_()
        self._widget.statusBar.showMessage("")

    def disconnectSSH(self):
        for index in self._widget.EstadosDeConexion.selectedIndexes():
            i = inc.SERVER_USERNAME.index(index.data())
            self.sdvs[i].ssh.client.close()
            self.sdvs[i].ssh = None
            self.EstadosDeConexion_model.removeRow(index.row())
         
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
        try: # Todas las columnas de la fila seleccionadas
            name, instr, place = sorted(self._widget.TablaInstrucciones.selectedIndexes())
        except:
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Warning)
            msg.setText(u"Error de Ejecución de Instrucción:")
            msg.setInformativeText("No se ha seleccionado toda la fila. Seleccione el índice de la instrucción")
            msg.setWindowTitle("Error")
            ret = msg.exec_()
            return
        # Selección correcta
        name, place = str(name.data()), str(place.data())
        
        i = 0
        if name == inc.SERVER_USERNAME[0]:   i=0#SDVUN_LAST_COMMAND[0] = self.sender.pose
        elif name == inc.SERVER_USERNAME[1]: i=1#SDVUN_LAST_COMMAND[1] = self.sender.pose
        elif name == inc.SERVER_USERNAME[2]: i=2#SDVUN_LAST_COMMAND[2] = self.sender.pose
    
        
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
            return
        
        if name == inc.SERVER_USERNAME[0]: self.sdvs[i].sender.publishPoseStamped()
        elif name == inc.SERVER_USERNAME[1]: self.sdvs[i].sender.publishPoseStamped()
        elif name == inc.SERVER_USERNAME[2]: self.sdvs[i].sender.publishPoseStamped()
        else:
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Warning)
            msg.setText(u"Error de Asignación de Comando:")
            msg.setInformativeText("No se ha reconocido el robot SDV")
            msg.setWindowTitle("Error")
            ret = msg.exec_()
            return
    
    def stopSDV(self):
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
                if sdv == inc.SERVER_USERNAME[0]: self.sdvs[0].sender.publishCancelGoal()
                elif sdv == inc.SERVER_USERNAME[1]: self.sdvs[1].sender.publishCancelGoal()
                elif sdv == inc.SERVER_USERNAME[2]: self.sdvs[2].sender.publishCancelGoal()
                else:
                    msg = QMessageBox()
                    msg.setIcon(QMessageBox.Warning)
                    msg.setText(u"Error de Asignación de Comando:")
                    msg.setInformativeText("No se ha reconocido el robot SDV")
                    msg.setWindowTitle("Error")
                    ret = msg.exec_()
                    return
       
    def resetSDV(self):
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
                if sdv == inc.SERVER_USERNAME[0]: self.sdvs[0].sender.publishResetGoal()
                elif sdv == inc.SERVER_USERNAME[1]: self.sdvs[1].sender.publishResetGoal()
                elif sdv == inc.SERVER_USERNAME[2]: self.sdvs[2].sender.publishResetGoal()
                else:
                    msg = QMessageBox()
                    msg.setIcon(QMessageBox.Warning)
                    msg.setText(u"Error de Asignación de Comando:")
                    msg.setInformativeText("No se ha reconocido el robot SDV")
                    msg.setWindowTitle("Error")
                    ret = msg.exec_()
                    return
       
    def closeEvent(self, event):
        # Cerrar todos los subprocesos abiertos
        #self.roscore.terminate()
        return


class Plugin(Plugin):

    def __init__(self, context):
        super(Plugin, self).__init__(context)
        self.context = context
        # Give QObjects reasonable names
        self.setObjectName('Plugin')

        # Create GUIs
        print strftime("%Y-%m-%d %H:%M:%S", gmtime())
        print '*** RQt SDVoice Pluggin ***'
        # Get path to UI file which should be in the "resource" folder of this package
        self.login_file = os.path.join(rospkg.RosPack().get_path('rqt_sdvoice'), 'resource', 'login.ui')
        self.registro_file = os.path.join(rospkg.RosPack().get_path('rqt_sdvoice'), 'resource', 'registro.ui')
        self.mainwindow_file = os.path.join(rospkg.RosPack().get_path('rqt_sdvoice'), 'resource', 'mainwindow.ui')
        # Create classes
        self.login = Login(self.login_file)
        self.login._widget.Entrar.released.connect(lambda: self.loginView()) # Verificar login de usuario
        self.login._widget.Registrarse.released.connect(lambda: self.registerView()) # Verificar login de usuario
        
        # Add widget to the user interface
        self.context.add_widget(self.login._widget)
       
    def loginView(self):
        # Login correcto
        if self.login.user:
            print 'Autenticado usuario: %s'%(self.login.user.display_name)
            self.sdvoice = SDVoice(self.mainwindow_file, self.login.user)
            # Show _widget.windowTitle on left-top of each plugin (when 
            # it's set in _widget). This is useful when you open multiple 
            # plugins at once. Also if you open multiple instances of your 
            # plugin at once, these lines add number to make it easy to 
            # tell from pane to pane.
            if self.context.serial_number() > 1:
                self.login._widget.setWindowTitle(self.login._widget.windowTitle() + (' (%d)' % self.context.serial_number()))
                self.sdvoice._widget.setWindowTitle(self.sdvoice._widget.windowTitle() + (' (%d)' % self.context.serial_number()))
            # Agregar widget al panel de rqt
            self.context.add_widget(self.sdvoice._widget)
            # TODO: CERRAR WIDGET LOGIN
            self.login._widget.close()
            
        # Error de datos de usuario    
        else:
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Warning)
            msg.setText(u"Error de Verificación de Usuario:")
            msg.setInformativeText(u"Usuario o contraseña no válidos!")
            msg.setWindowTitle("Error")
            ret = msg.exec_()
        
    def registerView(self):
        # Registro se presionó
        self.registro = Registro(self.registro_file)
        self.registro._widget.Registrarse.released.connect(lambda: self.registerToLogin()) # Verificar registro de usuario
        self.registro._widget.Cancelar.released.connect(lambda: self.cancelToLogin()) # Cancelar
        if self.context.serial_number() > 1:
            self.login._widget.setWindowTitle(self.login._widget.windowTitle() + (' (%d)' % self.context.serial_number()))
            self.registro._widget.setWindowTitle(self.registro._widget.windowTitle() + (' (%d)' % self.context.serial_number()))
            
        self.context.add_widget(self.registro._widget)
        
    def registerToLogin(self):
        self.user = self.registro.user
        
    def cancelToLogin(self):
        self.registro._widget.close()
        pass
            
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
