# Laboratorio 4 Robótica- Cinemática Directa con robot Phantom x Pincher
La cinemática directa usa ecuaciones cinemáticas para determinar la posición del actuador 
final a partir de la longitud de los eslabones y el ángulo que forman en sus articulaciones.
En una cadena cinemática serial como el robot Pincher, un conjunto de vectores de posición siempre 
tiene única solución, es decir un solo punto en el espacio.

## variables de los eslabones:

![pincher](https://user-images.githubusercontent.com/20913010/196850773-730bd781-cd5c-484d-932f-6ba7e6c11571.PNG)
d1= 45 mm;
d2= 105 mm;
d3= 105 mm;
d4= 40 mm;

## Tabla DH:

![dh pincher](https://user-images.githubusercontent.com/20913010/196851289-b248f673-c7ce-4791-9a60-b52783aa7214.PNG)


<h1 align="center"> Programas desarrollados en python</h1>
<h2>Requisitos previos </h2>

Para que los programas hechos en Python sean funcionales se requiere hacer varios pasos previos, los cuales serán listados a continuación:
- Instalar Catkin Build: Catkin build es un programa complementario a ROS, el cual permite la administración de paquetes o librerías que facilitan bastante el uso de ROS y su integración con los motores Dynamixel, presentes en el Pincher Phantom X.
- Instalar el paquete px_robot: Este paquete puede ser administrado mediante Catkin Build. Este paquete realiza la comunicación con los 5 motores Dynamixel AX-12 mediante ROS, crea los nodos necesarios y permite acceder a los topicos y servicios ofrecidos por estos nodos. El paquete se encuentra disponible en el enlace:  https://github.com/felipeg17/px_robot.

- Instalar la librería Robotics Toolbox de Peter Corke para Python: Esta libreria es una implementación de la librería de MATLAB en Python, de esta manera se pueden ingresar los parámetros del robot Pincher Panthom X y de esta manera obtener de manera gráfica el resultado esperado para cada movimiento y posición articular esperado en la realidad. Para su instalación se utiliza el siguiente comando:

```
  
	pip3 install roboticstoolbox-python
    
```

<h2>Código para generar el robot y graficarlo en una posición. </h2>

Para crear un robot y graficarlo mediante el Toolbox de Peter Corke para Python, lo primero que se debe hacer es importar la librería mediante la siguiente línea, también puede ser util importar las librerias numpy y time.

``` python
  
    import roboticstoolbox as rtb
    import numpy as np
    import time as t
    
```
A continuación se crea el robot mediante el método DHRobot(), al cual se le ingresa como parámetro un arreglo que contiene los eslabones del robot y como estos son de revolución, se utiliza el método RevoluteDH(), teniendo como argumentos cada uno de los parámetros DH que sean diferentes de 0, ángulos en radianes, para eso es útil la librería numpy. Además de los offset necesarios para la posición home determinada inicialmente. El otro argumento es el nombre del robot. Para el caso presente:

``` python
  
    robot= rtb.DHRobot([
      rtb.RevoluteDH(d=0.045, alpha= -np.pi/2, offset= 0),
      rtb.RevoluteDH(a= 0.105, offset= -np.pi/2),
      rtb.RevoluteDH(a=0.105, offset= np.pi/2),
      rtb.RevoluteDH(a=0.110, offset= np.pi/2)
    ], name= "Pincher")
    
```
Finalmente, con el método plot() Se puede graficar el robot para una posición angular deseada. Para ello se ingresa como parámetro un arreglo con la posición angular de cada articulación en radianes. El metodo hold() sirve para mantener la gráfica.

``` python
  
    robot.plot([np.pi/6, -np.pi/3, 0, 0]).hold()
    
```
Para este caso se tendrá como resultado:

![Display](https://user-images.githubusercontent.com/49238418/196843652-fa9cd21e-4b8e-473b-b6d3-72803508a9d0.png)


<h2>Código para suscribirse a las posiciones del robot. </h2>

Para suscribirse a las posiciones del robot y poderlas ver en consola, lo primero que se requiere es ejecutar el paquete de px_robot, esto se logra ejecutando los siguientes comandos desde la carpeta de Catkin:

``` 
  
   catkin build px_robot
   source devel/setup.bash
   roslaunch px_robot px_controllers.launch
    
```

A continuación ya es posible ejecutar el programa suministrado en este repositorio con el nombre de Suscriptor.py.

En el código, lo primero siempre es importar las librerías necesarias:

```  python
  
  import rospy
  import numpy as np
  import roboticstoolbox as rtb
  from std_msgs.msg import String 
  from sensor_msgs.msg import JointState
  import time as t
    
```

Para el funcionamiento de este código se tienen 2 partes:

- Creación de una función listener() que permite suscribirse al tópico.
- Creación de una función callback() que permite dar formato a lo mostrado en consola.

<h3>Función listener()</h3>

Para esta función se tienen tres líneas de código, la primera permite inicializar el nodo para suscribirse, la segunda permite suscribirse al tópico especificado como "/dynamixel_workbench/joint_states" y se especifica una función a ejecutar para dar formato a la salida, la tercera línea crea un bucle infinito del que solo se podrá salir si se envía una señal de terminación.

```  python
  
  def listener():
    rospy.init_node('joint_listener', anonymous=True)
    rospy.Subscriber("/dynamixel_workbench/joint_states", JointState, callback)
    rospy.spin()
    
```

<h3>Función callback()</h3>

Esta función se utiliza para dar formato a la salida. Para ello, se tiene un argumento data que contiene la información obtenida. Su atributo position consiste en un arreglo que contiene las posiciones de cada una de las articulaciones. Estas se separan en variables diferentes, se convierten a grados y se les adiciona el offset necesario para mantener correspondencia con el HOME definido, de manera que si el robot está en posición de HOME todos los águlos serán 0. Finalmente se muestra la posición de cada una de estas en consola. El código es el siguiente:

```  python
  def callback(data):
    art1=(data.position[0])*180/np.pi
    art2=-(data.position[1])*180/np.pi
    art3=-(data.position[2]+np.pi/2)*180/np.pi
    art4=-(data.position[3]+np.pi/2)*180/np.pi
    art5=-data.position[4]
    print("art1= ", art1, ", art2= ", art2, ", art3= ", art3, ", art4= ", art4,", art5= ", art5)
    
```
Por tanto asi se verá la salida con el robot estando en posicion de home:

![HOMEsus](https://user-images.githubusercontent.com/49238418/196846495-46b9e2f2-fbdc-4662-b9bd-a04569c9b1f7.png)
![SusOut](https://user-images.githubusercontent.com/49238418/196846736-16020f5d-869f-4000-b232-b082d39d0aa3.png)


<h2>Código para publicar las posiciones del robot. </h2>

Para este programa tambien se requiere inicializar el paquete px_robot. A continuación se importan las librerías necesarias:

```  python
  import rospy
  import numpy as np
  import time
  import roboticstoolbox as rtb
  from std_msgs.msg import String
  from sensor_msgs.msg import JointState
  from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    
```

En este caso, además de la creación del robot con el Toolbox, se define un único metodo que se llama joint_publisher(). En este se sigue la siguiente secuencia:

- Se crea un publicador con rospy.Publisher y se ingresa como argumento un tópico y un tipo de mensaje.
- Se iniclializa un nodo publicador.
- Se crea el arreglo posicionActual, el cual contiene inicialmente las posiciones de HOME.
- Se crea un bucle while infinito.

Dentro del bucle while se crea un mensaje de tipo JointTrajectory, se le pone el tiempo actual el encabezado del mensaje en el atributo header.stamp y se le pone un nombre a cada una de las articulaciones con el atributo joint_names. Se crea un nuevo mensaje de tipo JointTrajectoryPoint. A continuación el programa le pedirá al usuario especificar la articulación que se quiere modificar y su nueva posición. Dependiendo de la articulación, se harán los cáculos necesarios para modificar esta articulación teniendo en cuenta los rangos articulares y convirtiendo la posición a radianes. Si la posición se sale del rango se irá a la posición mas cercana. Una vez hecha la conversión, la nueva posición se añade al arreglo posicionActual. 

Acto seguido, se grafica la nueva posición mediante el toolbox. Se agrega el arreglo de posiciones al mensaje de puntos y luego este al mensaje de estado. Finalmente se publica el mensaje y en consola se muestra cual es la articulación editada con su nueva posición. El código es el siguiente:

```  python
 def joint_publisher():
    pub = rospy.Publisher('/joint_trajectory', JointTrajectory, queue_size=0)
    rospy.init_node('joint_publisher', anonymous=False)


    posicionActual=[0, 0, -np.pi/2, -np.pi/2, 0]

    while True:


        state = JointTrajectory()
        state.header.stamp = rospy.Time.now()
        state.joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]
        point = JointTrajectoryPoint()
        actual=int(input("Ingrese la articulación que desea manipular: "))
        if actual<1 or actual>5:
            break
        else:
            angDes=int(input("Ingrese el angulo deseado para la articulacion: "))

            if actual==1:
                ang=angDes+0
                if ang>-180 and ang<180:
                    posicionActual[0]=ang*np.pi/180
                elif ang<=-180:
                    angDes=-179
                    print("Posicion no admitida, moviendo a la mas cercana posible.")
                    posicionActual[0]=-179*np.pi/180
                else:
                    angDes=179
                    print("Posicion no admitida, moviendo a la mas cercana posible.")
                    posicionActual[0]=179*np.pi/180
            elif actual==2:
                ang=angDes+0
                if ang>-100 and ang<100:
                    posicionActual[1]=-ang*np.pi/180
                elif ang<=-100:
                    angDes=-99
                    print("Posicion no admitida, moviendo a la mas cercana posible.")
                    posicionActual[1]=99*np.pi/180
                else:
                    angDes=100
                    print("Posicion no admitida, moviendo a la mas cercana posible.")
                    posicionActual[1]=-99*np.pi/180
            elif actual==3:
                ang=angDes+90
                if ang>-130 and ang<130:
                    posicionActual[2]=-ang*np.pi/180
                elif ang<=-130:
                    angDes=-219
                    print("Posicion no admitida, moviendo a la mas cercana posible.")
                    posicionActual[2]=129*np.pi/180
                else:
                    angDes=40
                    print("Posicion no admitida, moviendo a la mas cercana posible.")
                    posicionActual[2]=-129*np.pi/180
            elif actual==4:
                ang=angDes+90
                if ang>-100 and ang<100:
                    posicionActual[3]=-ang*np.pi/180
                elif ang<=-100:
                    angDes=-190
                    print("Posicion no admitida, moviendo a la mas cercana posible.")
                    posicionActual[3]=89*np.pi/180
                else:
                    angDes=10
                    print("Posicion no admitida, moviendo a la mas cercana posible.")
                    posicionActual[3]=-99*np.pi/180
            elif actual==5:
                if ang>-90 and ang<90:
                    posicionActual[4]=-ang*np.pi/180
                elif ang<=-90:
                    ang=-89
                    print("Posicion no admitida, moviendo a la mas cercana posible.")
                    posicionActual[4]=89*np.pi/180
                else:
                    ang=89
                    print("Posicion no admitida, moviendo a la mas cercana posible.")
                    posicionActual[4]=-89*np.pi/180


            robot.plot([posicionActual[0],-np.pi/2+posicionActual[1],posicionActual[2],posicionActual[3]])
            point.positions = posicionActual  
            point.time_from_start = rospy.Duration(0.5)
            state.points.append(point)
            pub.publish(state)
            print('Articulacion editada: ', actual, ", ", "Nuevo angulo: ", angDes)
            rospy.sleep(1)

    
```
Asi se verá la consola:

![ConsolaPub](https://user-images.githubusercontent.com/49238418/196849575-5df228a6-33dd-485d-a8d5-29cdaa54741b.png)

Así la gráfica hecha mediante el toolbox: 

![GrafPub](https://user-images.githubusercontent.com/49238418/196849799-234a4d79-6846-4563-8234-ab15bc9d547b.png)

Y así quedará la posición del robot en la realidad:

![RobotPub](https://user-images.githubusercontent.com/49238418/196849845-3d73a999-6baa-4c64-8191-e087d16883fa.png)


<h2>Código para lograr las posiciones del robot mediante servicios </h2>

Al igual que en el anterior, se requiere tener inicializado el paquete px_robot. En este programa se importan las librerías:

```  python
  import rospy
  import time
  import roboticstoolbox as rtb
  import numpy as np
  from dynamixel_workbench_msgs.srv import DynamixelCommand
    
```

Se crea el robot en el Toolbox como se explicó anteriormente. Se crean dos métodos que permitirán utilizar el servicio y calcular el valor que debe ser enviado.

El primer método es jointCommand(), el cual recibe como argumentos un tipo de comando, un identificador de articulación, una dirección, un valor y un tiempo. Este espera a que haya disponibilidad del servicio, a continuación lo utiliza para enviar el nuevo comando con los nuevos parámetros y comprueba si funcionó correctamente. El código es:


```  python
  def jointCommand(command, id_num, addr_name, value, time):
    #rospy.init_node('joint_node', anonymous=False)
    rospy.wait_for_service('dynamixel_workbench/dynamixel_command')
    try:        
        dynamixel_command = rospy.ServiceProxy(
            '/dynamixel_workbench/dynamixel_command', DynamixelCommand)
        result = dynamixel_command(command,id_num,addr_name,value)
        rospy.sleep(time)
        return result.comm_result
    except rospy.ServiceException as exc:
        print(str(exc))
    
```

Por otro lado, el método calculaAngulo() se utiliza para calcuular el valor que debe ser enviado a cada motor, esto debido a que mediante el servicio se envía una señal de entre 0 y 1023  en vez de un ángulo en radianes, para ello por tanto, se determina la posición de HOME para cada articulación en estos nuevos valores. A continuación, con el ángulo en grados que se desea se hace la conversión a la nueva señal siguiendo una sencilla regla de tres. Para la articulación 4 se deben tener en cuenta los rangos articulares, esto debido a que la posición HOME definida no permite que se alcancen los ángulos solicitados en los casos 4 y 5. El código es el siguiente:


```  python
def calculaAngulo(angulo, ID):
    if ID==1:
        salidaA=int(512+angulo/180*512)
    elif ID==2:
        salidaA=int(512-angulo/130*512)
    elif ID==3:
        salidaA=int(220-angulo/130*512)
    elif ID==4:
        salidaA=int(230-angulo/1301*512)
        if salidaA <200:
            salidaA=200
    return salidaA
    
```

Finalmente se tiene una porción main de código, en la cual lo primero que se busca es establecer un límite de torque para evitar sobrecarga de corriente en los motores:

```  python
        jointCommand('', 1, 'Torque_Limit', 500, 0)
        jointCommand('', 2, 'Torque_Limit', 500, 0)
        jointCommand('', 3, 'Torque_Limit', 400, 0)
        jointCommand('', 4, 'Torque_Limit', 400, 0)
```

A continuación se tiene un bucle while infinito el cual solicita la posición deseada. A partir de esta utiliza el servicio para enviar la nueva posición de cada articulación y calcula el valor de señal a enviar mediante el método calculaAngulo. Finalmente grafica la posición mediante el Toolbox. El código es:

```  python
       while(True):
            posicion=int(input("Ingrese un numero de posicion: "))
            if(posicion== 1):
                jointCommand('',1, 'Goal_Position', calculaAngulo(0,1), 0)
                jointCommand('',2, 'Goal_Position', calculaAngulo(0,2), 0)
                jointCommand('',3, 'Goal_Position', calculaAngulo(0,3), 0)
                jointCommand('',4, 'Goal_Position', calculaAngulo(0,4), 0)
                robot.plot([0, 0, 0, 0])
            elif(posicion==2):
                jointCommand('',1, 'Goal_Position', calculaAngulo(-20,1), 0)
                jointCommand('',2, 'Goal_Position', calculaAngulo(20,2), 0)
                jointCommand('',3, 'Goal_Position', calculaAngulo(-20,3), 0)
                jointCommand('',4, 'Goal_Position', calculaAngulo(20,4), 0)
                robot.plot([np.pi/180*-20, np.pi/180*20, np.pi/180*-20, np.pi/180*20])
            elif(posicion==3):
                jointCommand('',1, 'Goal_Position', calculaAngulo(30,1), 0)
                jointCommand('',2, 'Goal_Position', calculaAngulo(-30,2), 0)
                jointCommand('',3, 'Goal_Position', calculaAngulo(30,3), 0)
                jointCommand('',4, 'Goal_Position', calculaAngulo(-30,4), 0)
                robot.plot([np.pi/180*30, np.pi/180*-30, np.pi/180*30, np.pi/180*-30])
            elif(posicion==4):
                jointCommand('',1, 'Goal_Position', calculaAngulo(-90,1), 0)
                jointCommand('',2, 'Goal_Position', calculaAngulo(15,2), 0)
                jointCommand('',3, 'Goal_Position', calculaAngulo(-55,3), 0)
                jointCommand('',4, 'Goal_Position', calculaAngulo(10,4), 0)
                robot.plot([np.pi/180*-90, np.pi/180*15, np.pi/180*-55, np.pi/180*10])
            elif(posicion==5):
                jointCommand('',1, 'Goal_Position', calculaAngulo(-90,1), 0)
                jointCommand('',2, 'Goal_Position', calculaAngulo(45,2), 0)
                jointCommand('',3, 'Goal_Position', calculaAngulo(-55,3), 0)
                jointCommand('',4, 'Goal_Position', calculaAngulo(10,4), 0)
                robot.plot([np.pi/180*-90, np.pi/180*45, np.pi/180*-55, np.pi/180*10])
```

Por ejemplo, para la posición 1 se logra:

![Sim1](https://user-images.githubusercontent.com/49238418/196853416-4401cb20-6177-499a-b7a9-7f17952f1d7d.png)

![Pos1](https://user-images.githubusercontent.com/49238418/196853425-8a4645b1-2492-4b5e-b24e-23dfcd6cb774.png)

Para la 2:

![Sim2](https://user-images.githubusercontent.com/49238418/196853482-dbda6f9e-9230-40aa-95fc-2c5cab1977b8.png)


![Pos2](https://user-images.githubusercontent.com/49238418/196853454-44f6fe09-1913-45ab-9628-adae5fb3825e.png)

Para la 3:

![Sim3](https://user-images.githubusercontent.com/49238418/196853516-c9b846e5-e2b9-4128-b0f0-fae1749c26c3.png)

![Pos3](https://user-images.githubusercontent.com/49238418/196853527-aef8e9fd-4c7f-47e9-bc4c-bac9ba6095db.png)

Para la 4, el angulo de la articulación 4 no puede ser alcanzado, se sustituye por 10:

![Sim4](https://user-images.githubusercontent.com/49238418/196853638-0a270d19-0ea3-450a-82b8-d064db0d5a03.png)

![Pos4](https://user-images.githubusercontent.com/49238418/196853650-022406d1-0aaf-4a21-acea-72fbd098e7da.png)

Para la 5 ocurre lo mismo que en la anterior:

![Sim5](https://user-images.githubusercontent.com/49238418/196853688-bbf2f3b4-94f0-4844-8946-ff615866eec6.png)

![Pos5](https://user-images.githubusercontent.com/49238418/196853695-c563c7f8-e18a-4800-bc7e-a16e94001065.png)


