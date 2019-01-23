import sys
import time
import rospy
import socket
import roslaunch
import subprocess
from paramiko import client
from firebase import firebase
from PyQt5 import uic, QtGui, QtCore
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QLineEdit, QGridLayout, QTextEdit, QPushButton
import includes
from ros_utils import *
#load ui file
RELATIVE_DIR = './'
GUI_DIR = RELATIVE_DIR + './'
GUI_TITLE = 'Comandos SDV'
MAIN_WINDOW = GUI_DIR + 'mainwindow.ui'
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
        # Let the user know we're connecting to the server
        print("Connecting to server.")
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
            print("Connection not opened.")

#----------------------------------------------------
# Name: DOYViewer
# Type: Class
# input args: Q_MAIN_WINDOW and UI_MAIN_WINDOW
# output args: DOYViewer object
#----------------------------------------------------
class DOYViewer(Q_MAIN_WINDOW, UI_MAIN_WINDOW):
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
        self.EstadosDeConexion_model = QtGui.QStandardItemModel()
        self.EstadosDeConexion.setModel(self.EstadosDeConexion_model)
        #print socket.gethostname()
        #IP1 = socket.gethostbyname(socket.gethostname()) # local IP adress of your computer
        #IP2 = socket.gethostbyname('name_of_your_computer') # IP adress of remote computer
        # Definir callbacks de cada elemento de la gui
        self.Conectar.clicked.connect(lambda: self.connectSSH())
        self.Desconectar.clicked.connect(lambda: self.disconnectSSH())
        #self.btn_sdv_experimental.clicked.connect(lambda: self.sdvGoToExperimental())
        #self.btn_sdv_manufactura.clicked.connect(lambda: self.sdvGoToManufactura())
        #self.btn_sdv_industrial.clicked.connect(lambda: self.sdvGoToIndustrial())
        
        # Lanzar ROS Core:
        self.roscore.run()
        # Verificar lista de topicos
        #print rospy.client.get_published_topics()
        # Identificar los robots conectados
        #node = roslaunch.core.Node('sdvoice', 'sdvoice')
        
        # inicializar conexion
        self.initSocket()
        self.statusBar.showMessage("System Status | Ready. Welcome!")

    def initSocket(self):
        self.ssh_clients = [None for i in SERVER_USERNAME]
        # Agregar a ListaConexiones lista de hosts maquinas
        self.ListaConexiones.addItems(SERVER_USERNAME)

    def connectSSH(self):
        i = self.ListaConexiones.currentIndex()
        self.ssh_clients[i] = ssh(SERVER_IP[i], SERVER_USERNAME[i], SERVER_PASSWORD[i])
        # Agregar a EstadosDeConexion lista de hosts conectados
        if self.ssh_clients[i].client != None:
            item = QtGui.QStandardItem(self.ListaConexiones.currentText())
            self.EstadosDeConexion_model.appendRow(item)
        else:
            # TODO: Agregar widget de error de conexion
            self.statusBar.showMessage("No se pudo conectar a %s!"%(self.ListaConexiones.currentText()))

    def disconnectSSH(self):
        for index in self.EstadosDeConexion.selectedIndexes():
            i = SERVER_USERNAME.index(index.data())
            self.ssh_clients[i].client.close()
            self.ssh_clients[i] = None
            self.EstadosDeConexion_model.removeRow(index.row())
            
    def closeEvent(self, event):
        print ("inside the close")
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
    gui = DOYViewer()
    gui.show()
    sys.exit(app.exec_())
    
if __name__ == '__main__':
    main()
