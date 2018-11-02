import firebase_admin
from firebase_admin import db
from firebase_admin import credentials
from firebase_admin import auth

# Autenticar conexion URL a la base de datos
cred = credentials.Certificate('../automatizacion-87dc9-adminsdk.json')
default_app = firebase_admin.initialize_app(cred, {
                    'databaseURL' : 'https://automatizacion-87dc9.firebaseio.com/'
              })
#print default_app.name
'''
# Referenciar tabla de sdv
SDV1 = db.reference('/SDV/SDV1')
# Agregar datos
SDV1.push({ u'time': u'1',
           u'pos': {'X':'1',
                    'Y':'1',
                    'Z':'1'},
           u'vel': {'X':'1',
                    'Y':'1',
                    'Z':'1'},
           u'task': u'move'}
)

# Referenciar tabla de sdv
SDV2 = db.reference('/SDV/SDV2')
# Agregar usuario nuevo
SDV2.push({ u'time': u'1',
           u'pos': {'X':'1',
                    'Y':'1',
                    'Z':'1'},
           u'vel': {'X':'1',
                    'Y':'1',
                    'Z':'1'},
           u'task': u'move'}
)
'''
# Referenciar tabla de sdv
SDV3 = db.reference('/SDV/SDV3')
# Agregar usuario nuevo
key = SDV3.push({ u'time': u'1',
           u'pos': {'X':'1',
                    'Y':'1',
                    'Z':'1'},
           u'vel': {'X':'1',
                    'Y':'1',
                    'Z':'1'},
           u'task': u'move'}
).key
print key

#Actualizacion
SDV3_CHILD = db.reference('/SDV/SDV3/%s'%(key))
SDV3_CHILD.update({ u'time': u'1',
           u'pos': {'X':'1',
                    'Y':'1',
                    'Z':'1'},
           u'vel': {'X':'1',
                    'Y':'2',
                    'Z':'1'},
           u'task': u'move'})
'''
# Imprimir nuevamente lista de datos
datos = SDV1.get()
print datos
'''
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

# Iterate through all users. This will still retrieve users in batches,
# buffering no more than 1000 users in memory at a time.
for user in auth.list_users().iterate_all():
    print('User: ' + user.display_name) 
    


