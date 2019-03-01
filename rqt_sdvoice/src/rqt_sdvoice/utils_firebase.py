import os
import scrypt
import firebase_admin
from PyQt5 import uic, QtGui, QtCore
from geometry_msgs.msg import PoseStamped
from firebase_admin import db, credentials, auth
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QLineEdit, QGridLayout, QTextEdit, QPushButton, QMessageBox


# Custom Libraries
import includes as inc

class Firebase():
    def __init__(self):
        # Autenticar conexion URL a la base de datos
        cred = credentials.Certificate("/home/sdv/catkin_ws/src/rqt_sdvoice/src/rqt_sdvoice/automatizacion-87dc9-adminsdk.json")
        default_app = firebase_admin.initialize_app(cred, {
                        'databaseURL' : 'https://automatizacion-87dc9.firebaseio.com/'
             })
    
    ## Metodo de creacion de la base de datos inicial de cada usuario    
    def newDataBase(self,userName):
        path = '/Users/%s'%userName
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
        user = auth.create_user(
        email = correo,
        email_verified = True,
        phone_number = '+57%s'%celular,
        password = clave,
        display_name = userName,
        disabled=False)
        #print('Sucessfully created new user: {0}'.format(user.uid)) # Se puede imprimir en donde se necesite
        # Set admin privilege on the user corresponding to uid.
        auth.set_custom_user_claims(user.uid, {'admin': True})        
        ## Agregar el pass del usuario
        path = '/ConecctionData/%s'%userName
        ref = db.reference(path) # Referencia a los datos de conexion del usuario que se creo
        ref.set({
                'email': correo,
                'Pass': clave
                })        
        return user                 

    ## Metodo para obtener el user de firebase con el email
    def getUser(self,email):
        user = auth.get_user_by_email(email)
        #print('Successfully fetched user data: {0}'.format(user.uid)) # Se puede imprimir en donde se necesite
        return user
        
    ## Metodo para agregar datos a la base de datos
    def addData(self, userName, SDV, time, pose, ori, task):
        sdv = '/Users/' + userName + '/SDV/'+SDV
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
        sdv = '/Users/' + userName + '/SDV/'+SDV
        newRef = db.reference(sdv) # Referencia al tallo SDV#
        
        # Obtener datos        
        data = newRef.order_by_key().get()
        for key in data.items():
            print(key)
            
    ## Metodo para modificar la task de uno de los datos  
    def updateTask(self, userName, SDV, key, update): 
        path = '/Users/' + userName + '/SDV/' + SDV + '/%s'%key
        ref = db.reference(path) # Referencia al usuario que se creo
        ref.update({
            'task': update
        })
        
    ## Metodo para modificar una password
    def updatePassword(self, userName, update): 
        path = '/ConecctionData/' + userName
        ref = db.reference(path) # Referencia al usuario que se creo
        ref.update({
            'Pass': update
        })

    ## Metodo para hacer login
    def login(self, email, clave, firebase):             
        try:
            user = firebase.getUser(email)  # Si no hay usuario bota error
        except:
            print('No hay ningun usuario registrado a ese email')
            return 0

        # Si no hay error continua aqui
        userName = user.display_name    # username del usuario obtenido con getUser
        path = '/ConecctionData/' + userName
        ref = db.reference(path) # Referencia a los datos de conexion del usuario
        data = ref.get() # Toma los datos de todo el path
        [email, passw] = data.items() # Guarda los datos del email y de la pass por separado
        password = passw[1] # Toma solo el valor del password
        if  password == clave:
            return user
        else:
            print('La contrasena no coincide, intentelo de nuevo.') 
            return 0
