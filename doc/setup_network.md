# Configuración de las variables ROS_MASTER_URI de robot SDV
Fecha: 23/Enero/2019
---

## En los robots SDV:

* En los archivos `~/.bashrc` de cada robot SDV declarar la siguiente variable:

`ROS_MASTER_URI = http://192.168.1.14:11311`

* Ejecutar el comando:

source ~/.bashrc

## En el servidor local:

* En el archivo `~/.bashrc` del servidor local declarar la siguiente variable:

`ROS_MASTER_URI = http://localhost:11311`

Ejecutar el comando:

* source ~/.bashrc

**NOTA: No olvidar reiniciar estos archivos al terminar las prácticas experimentales con este proyecto**
