import os
import firebase_admin
from firebase_admin import db
from firebase_admin import credentials
from firebase_admin import auth

# Custom Libraries
import includes as inc

class Firebase():
    # Autenticar conexion URL a la base de datos
    cred = credentials.Certificate("/home/sdv/catkin_ws/src/rqt_sdvoice/src/rqt_sdvoice/automatizacion-87dc9-adminsdk.json")
    default_app = firebase_admin.initialize_app(cred, {
                    'databaseURL' : 'https://automatizacion-87dc9.firebaseio.com/'
              })
    def __init__(self):
        ## Creacion de la base de datos inicial
        ref = db.reference('/') # Referencia a la raiz del proyecto
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
    

    ## Metodo para agregar datos a la base de datos
    def addData(self, SDV, time, pose, ori, task):
        newRef = db.reference('/SDV/%s'%SDV) # Referencia al tallo SDV3
      
        # Agregar datos
        newRef.push({ u'time': time,
                   u'pos': {'X': pose.x,
                            'Y': pose.y,
                            'Z': pose.z},
                   u'vel': {'X': ori.x,
                            'Y': ori.y,
                            'Z': ori.z,
                            'W': ori.w},
                   u'task': task}
        )
    def getData(self,SDV):
        if SDV == 'SDV1':
            newRef = db.reference('/SDV/SDV1') # Referencia al tallo SDV1
        elif SDV == 'SDV2':
            newRef = db.reference('/SDV/SDV2') # Referencia al tallo SDV2    
        else:
            newRef = db.reference('/SDV/SDV3') # Referencia al tallo SDV3
        
        # Obtener datos        
        data = newRef.order_by_key().get()
        for key in data.items():
            print(key)      
'''        
# Obtener el usuario por email
user = auth.get_user_by_email('mfbejaranob@unal.edu.co')
print('Successfully fetched user data: {0}'.format(user.uid))
'''

''' 
# Crear usuarios Administradores:
user = auth.create_user(
    email='mfbejaranob@unal.edu.co',
    email_verified=True,
    phone_number='+573142183559',
    password='123456',
    display_name='Manuel Bejarano',
    disabled=False)
print('Sucessfully created new user: {0}'.format(user.uid))

# Set admin privilege on the user corresponding to uid.
auth.set_custom_user_claims(user.uid, {'admin': True})

user = auth.create_user(
    email='saalvarezse@unal.edu.co',
    email_verified=True,
    phone_number='+573123498388',
    password='123456',
    display_name='Santiago Alvarez',
    disabled=False)
print('Sucessfully created new user: {0}'.format(user.uid))

# Set admin privilege on the user corresponding to uid.
auth.set_custom_user_claims(user.uid, {'admin': True})
'''
'''
# Iterate through all users. This will still retrieve users in batches,
 buffering no more than 1000 users in memory at a time.
 for user in auth.list_users().iterate_all():
    print('User: ' + user.display_name) 
'''
#----------------------------------------------------
# MAIN PROGRAM
#----------------------------------------------------
# Name: main
# Type: Main Function
# input args: None
# output args: None
#----------------------------------------------------
'''
def main():
    '' '
    Main function of execution for the application. 
    It generates the QT App and executes it 
    '' '

    firebase = Firebase()
    firebase.addData('SDV1', 1, [0,0,0],[1,2,3],'move')
    firebase.addData('SDV1', 2, [0,1,0],[1,3,3],'move')
    firebase.addData('SDV2', 4, [2,1,1],[5,4,6],'recieve')
    firebase.getData('SDV1')
if __name__ == '__main__':
    main()
#----------------------------------------------------
'''

