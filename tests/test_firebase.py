import firebase_admin
from firebase_admin import db
from firebase_admin import credentials
from firebase_admin import auth
from PyQt5 import uic, QtGui, QtCore
from geometry_msgs.msg import PoseStamped
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QLineEdit, QGridLayout, QTextEdit, QPushButton, QMessageBox
from six.moves import urllib
import json

class Firebase():

    def __init__(self):
        # Generar el API Key
        self.firebase_apikey = 'AIzaSyDXHZOR2M2cU1MHTVhTLMq3hBG4sBcSsa0'
        # Autenticar conexion URL a la base de datos
        cred = credentials.Certificate("/home/manuel/Descargas/automatizacion-87dc9-firebase-adminsdk-6jz9r-407ae5fd18.json")
        default_app = firebase_admin.initialize_app(cred, {
                    'databaseURL' : 'https://automatizacion-87dc9.firebaseio.com/'
              })

    ## Metodo de creacion de la base de datos inicial de cada usuario    
    def newDataBase(self,userName):
        path = '/%s'%userName
        ref = db.reference(path) # Referencia al usuario que se creo
        ref.set({
                'SDV': 
                    {
                        'sdvun1': {
                           u'POSE0':{
                                u'time': u'1',
                                u'pos': {'X':'0',
                                    'Y':'0',
                                    'Z':'0'},
                                u'ori': {'X':'0',
                                    'Y':'0',
                                    'Z':'0',
                                    'W':'1'},
                                u'status': u'move'
                            }
                        },
                        'sdvun2': {
                            u'POSE0':{
                                u'time': u'1',
                                u'pos': {'X':'0',
                                    'Y':'0',
                                    'Z':'0'},
                                u'ori': {'X':'0',
                                    'Y':'0',
                                    'Z':'0',
                                    'W':'1'},
                                u'status': u'move'
                            }
                        },
                        'sdvun3': {
                            u'POSE0':{
                                u'time': u'1',
                                u'pos': {'X':'0',
                                    'Y':'0',
                                    'Z':'0'},
                                u'ori': {'X':'0',
                                    'Y':'0',
                                    'Z':'0',
                                    'W':'1'},
                                u'status': u'move'
                            }
                        },
                    }
                })

        
    
    ## Metodo para crear un nuevo usuario y retorna el user de firebase
    def addUser(self, correo, clave, celular, userName):     
        error = False  ## Bandera de error      
        try:           
            user = auth.create_user(
            email = correo,
            email_verified = True,
            phone_number = '+57%s'%celular,
            password = clave,
            display_name = userName,
            disabled=False)
            # Set admin privilege on the user corresponding to uid.
            auth.set_custom_user_claims(user.uid, {'admin': True})
        except:
            print('Puede que haya un usuario registrado a ese email')
            print('Revise los datos ingresados en busca de errores')
            error = True
        finally:
            if error == False:            
                return user        

    ## Metodo para obtener el user de firebase con el email
    def getUser(self,email):
        user = auth.get_user_by_email(email)
        return user

    ## Metodo para hacer login
    def login(self, email, password, firebase):             
        error = False    ## Bandera de error
        try:
            user = firebase.getUser(email)  # Si no hay usuario bota error
        except:
            print('No hay ningun usuario registrado a ese email')
            error = True
        finally:
            if error == False:        
                my_data = dict()
                my_data["email"] = email
                my_data["password"] = password
                my_data["returnSecureToken"] = True
				            
                json_data = json.dumps(my_data).encode()  # Archivo para la transferencia de datos al toolkit
                headers = {"Content-Type": "application/json"}		
                request = urllib.request.Request("https://www.googleapis.com/identitytoolkit/v3/relyingparty/verifyPassword?key="+self.firebase_apikey, data=json_data, headers=headers)  # Mensaje enviado al toolkit
              		
                try:
                    loader = urllib.request.urlopen(request)
                except urllib.error.URLError as e:
                    message = json.loads(e.read())
                    print(message["error"]["message"])
                    error = True
                finally:
                    if error == False: 
                        print(loader.read())
                        return user       
   
    ## Metodo para modificar una password
    def resetPassword(email):
        my_data = dict()
        my_data["email"] = email
        my_data["requestType"] = "PASSWORD_RESET"
				    
        json_data = json.dumps(my_data).encode()  # Archivo para la transferencia de datos al toolkit
        headers = {"Content-Type": "application/json"}		
        request = urllib.request.Request("https://www.googleapis.com/identitytoolkit/v3/relyingparty/getOobConfirmationCode?key="+self.firebase_apikey, data=json_data, headers=headers)  # Mensaje enviado al toolkit
        try:
            loader = urllib.request.urlopen(request)
        except urllib.error.URLError as e:
            message = json.loads(e.read())
            print(message["error"]["message"])
        else:
            print(loader.read())

    ## Metodo para borrar un usuario
    def deleteUser(self, id_token):		
        my_data = {"idToken": id_token}
      			
        json_data = json.dumps(my_data).encode()  # Archivo para la transferencia de datos al toolkit
        headers = {"Content-Type": "application/json"}		
        request = urllib.request.Request("https://www.googleapis.com/identitytoolkit/v3/relyingparty/deleteAccount?key="+self.firebase_apikey, data=json_data, headers=headers)  # Mensaje enviado al toolkit	
        try:
            loader = urllib.request.urlopen(request)
        except urllib.error.URLError as e:
            message = json.loads(e.read())
            print(message["error"]["message"])
        else:
            print(loader.read())        

    ## Metodo para actualizar el token de inicio de sesion
    def refreshToken(self, token):
        my_data = dict()
        my_data["grant_type"] = "refresh_token"
        my_data["refresh_token"] = token
				    
        json_data = json.dumps(my_data).encode()  # Archivo para la transferencia de datos al toolkit
        headers = {"Content-Type": "application/json"}		
        request = urllib.request.Request("https://securetoken.googleapis.com/v1/token?key="+self.firebase_apikey, data=json_data, headers=headers)
      		
        try:
            loader = urllib.request.urlopen(request)
        except urllib.error.URLError as e:
            message = json.loads(e.read())
            print(message["error"]["message"])
        else:
            print(loader.read())
      		

    ## Metodo para agregar datos a la base de datos
    def addData(self, userName, SDV, time, pose, ori, task):
        sdv = '/' + userName + '/SDV/'+SDV
        newRef = db.reference(sdv) # Referencia al tallo SDV#      
        # Agregar datos
        newKey = newRef.push({ u'time': time,
                   u'pos': {'X': pose[0],
                            'Y': pose[1],
                            'Z': pose[2]},
                   u'ori': {'X': ori[0],
                            'Y': ori[1],
                            'Z': ori[2],
                            'W': ori[3]},
                   u'task': task}
        )
        return newKey.key
    
    ## Metodo para obtener los datos de un SDV
    def getData(self, userName, SDV):      
        sdv = '/' + userName + '/SDV/'+SDV
        newRef = db.reference(sdv) # Referencia al tallo SDV#        
        # Obtener datos        
        data = newRef.order_by_key().get()
        for key in data.items():
            print(key)
    
    ## Metodo para modificar la task de uno de los datos  
    def updateTask(self, userName, SDV, key, update): 
        path = '/' + userName + '/SDV/' + SDV + '/%s'%key
        ref = db.reference(path) # Referencia al usuario que se creo
        ref.update({
            'task': update
        })

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

    firebase = Firebase() 
    
    #user = firebase.addUser('mfbejaranob@unal.edu.co', '123456', '3142183559', 'Manuel')
    #firebase.newDataBase(user.display_name)
    #user2 = firebase.addUser('saalvarezse@unal.edu.co', '123456', '3123498388', 'Santiago')
    #firebase.newDataBase(user2.display_name)       
    
    #user = firebase.addUser('sdazar@unal.edu.co', '123456', '3016541002', 'Santiago Daza')
    #firebase.newDataBase(user.display_name)    
    
    #user = firebase.login('sdazar@unal.edu.co', '123456', firebase)
    #firebase.deleteUser()
    
    #user = firebase.getUser('mfbejaranob@unal.edu.co')
    '''
    dataId = firebase.addData(user.display_name, 'sdvun1', 1, [0,0,0],[1,2,3,4], 'move')
    dataId = firebase.addData(user.display_name, 'sdvun1', 2, [0,0,0],[2,2,3,4], 'move')
    dataId = firebase.addData(user.display_name, 'sdvun1', 3, [0,0,0],[2,2,3,4], 'move')
    dataId = firebase.addData(user.display_name, 'sdvun1', 4, [0,0,0],[2,2,3,4], 'move')
    '''
    #firebase.updateTask(user.display_name, 'sdvun1', dataId, 'complete')

    '''
    dataId = firebase.addData(user.display_name, 'sdvun2', 1, [0,0,0],[1,2,3,4], 'moving')
    dataId = firebase.addData(user.display_name, 'sdvun2', 2, [0,0,0],[2,2,3,4], 'moving')
    dataId = firebase.addData(user.display_name, 'sdvun2', 3, [0,0,0],[2,2,3,4], 'moving')
    dataId = firebase.addData(user.display_name, 'sdvun2', 4, [0,0,0],[2,2,3,4], 'moving')
    '''
    #firebase.updateTask(user.display_name, 'sdvun2', '-L_MOkgcogIOLeFF4dLv', 'failed')
        
    
if __name__ == '__main__':
    main()
