import os
import scrypt
import firebase_admin
from PyQt5 import uic, QtGui, QtCore
from geometry_msgs.msg import PoseStamped
from firebase_admin import db, credentials, auth
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QLineEdit, QGridLayout, QTextEdit, QPushButton, QMessageBox
from six.moves import urllib
import json

# Custom Libraries
import includes as inc

class Firebase():

    def __init__(self):
        # Generar el API Key
        self.firebase_apikey = 'AIzaSyDXHZOR2M2cU1MHTVhTLMq3hBG4sBcSsa0'
        # Autenticar conexion URL a la base de datos
        cred = credentials.Certificate("/home/sdv/catkin_ws/src/rqt_sdvoice/src/rqt_sdvoice/automatizacion-87dc9-adminsdk.json")
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
                        inc.SERVER_USERNAME[0]: {
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
                        inc.SERVER_USERNAME[1]: {
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
                        inc.SERVER_USERNAME[2]: {
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
                   u'pos': {'X': pose.x,
                            'Y': pose.y,
                            'Z': pose.z},
                   u'ori': {'X': ori.x,
                            'Y': ori.y,
                            'Z': ori.z,
                            'W': ori.w},
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
