# Introducción a ROS 2 para desarrolladores

La filosofía básica de ROS 2 sigue siendo la misma que en ROS 1: distribuir el código en nodos que intercambian mensajes, pero a la hora de programar, las herramientas de desarrollo, la API y la organización del código fuente en paquetes ha cambiado en algunos aspectos. Vamos a ver aquí una pequeña introducción a la programación en ROS2 con Python para que nos sirva de base para el desarrollo de las prácticas.


## 1. Workspaces y packages

En ROS 2 los conceptos de workspace y packages son iguales que en ROS 1 pero hay bastantes diferencias prácticas en cuanto a los comandos para compilar, crear paquetes, etc.

|       | ROS 1 (catkin) | ROS 2 (ament + colcon) |
|------|----------------|------------------------|
|nombre del workspace: | `catkin_ws` por convención | `ros2_ws` por convención |
|compilar con: | `catkin_make` | `colcon build` |
|"instalar" el workspace en el shell: | `source devel/setup.bash` | `source install/setup.bash` |
|crear un paquete: | `catkin_create_pkg` | `ros2 pkg create` |

### Crear un workspace

```bash
# 1. Crea el workspace y su src
mkdir -p mi_ros2_ws/src
cd mi_ros2_ws

# 2. Compila aunque esté vacío para generar la estructura de directorios (build, install, log)
colcon build
```

Cada vez que abras una terminal para trabajar con este workspace necesitas hacer un `source` del *script* `install/setup.bash` para poder acceder desde la terminal a los paquetes y a los nodos ROS que crees en el workspace.

```bash
source mi_ros2_ws/install/setup.bash
```

> Puedes añadir la línea anterior al final de tu ~/.bashrc si siempre usas el mismo workspace.

### Crear un package

Los paquetes residen el el directorio `src` del *workspace*:


1. **Métete en el directorio `src`** del *workspace* que creaste 

    ```bash
    cd mi_ros2_ws/src
    ```

2. **Ejecuta la orden `ros2 pkg create`** para crear el paquete. Hace falta darle un nombre y decir si es de tipo python o C++ y de qué otros paquetes depende (en nuestro caso `rclpy`, que contiene los APIs básicos para trabajar con Python y `std_msgs` que contiene los tipos de mensaje básicos como cadenas, enteros, arrays, etc)

    ```bash
    ros2 pkg create practica0 --build-type ament_python --dependencies rclpy std_msgs
    ```


Si todo es correcto, el comando anterior habrá creado un nuevo directorio `practica0` con una estructura de subdirectorios que de momento no veremos con detalle.



## 2. Ejemplo productor/consumidor

Como ROS está basado en el paso de mensajes, algunos nodos publicarán mensajes y otros los consumirán (aunque puede haber alguno que haga las dos cosas). Por eso uno de los ejemplos más típicos de introducción a ROS es el productor/consumidor: un nodo produce mensajes (que para simplificar son de texto plano) y otro nodo los consume (y los muestra en pantalla).

Vamos a crear el nodo que produce los mensajes, generando 1 mensaje de texto cada segundo. El código de los nodos de ROS 2 debe ir dentro de una carpeta que se habrá creado dentro de la principal del paquete y también con el nombre del paquete (en nuestro caso la carpeta será `practica0/practica0`, y si miras dentro de ella verás que tiene un archivo llamado `__init__.py`, que en python se usa para "marcar" que una carpeta es un paquete python). Al archivo lo podemos llamar `productor.py`.

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Productor(Node):
    def __init__(self):
        super().__init__('productor')
        self.pub = self.create_publisher(String, 'saludo', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)  # 1 Hz
        self.count = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'saludo numero {self.count}'
        self.pub.publish(msg)
        self.get_logger().info(msg.data)
        self.count += 1

def main():
    rclpy.init()
    node = Productor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

#solo sirve por si queremos ejecutarlo con "python productor.py"
if __name__ == '__main__':
    main()    
```


consumidor.py (suscriptor)

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Consumidor(Node):
    def __init__(self):
        super().__init__('consumidor')
        self.sub = self.create_subscription(
            String,
            'saludo',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        self.get_logger().info(f'Recibido: {msg.data}')

def main():
    rclpy.init()
    node = Consumidor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
Ejecución

En ROS 2 no hay roscore: el DDS viene integrado.

# terminal 1
ros2 run practica1 productor

# terminal 2
ros2 run practica1 consumidor
Herramientas de introspección:

ros2 topic list
ros2 topic info /saludo
ros2 topic echo /saludo
3. Leyendo mensajes de sensores y publicando al robot
Supondremos que ejecutas TurtleBot 3 (oficialmente soportado en ROS 2) en Gazebo:

# carga el entorno de los paquetes TurtleBot 3
source ~/turtlebot3_ws/install/setup.bash

# lanza un mundo simple
export TURTLEBOT3_MODEL=burger     # waffle, waffle_pi, burger…
ros2 launch turtlebot3_gazebo empty_world.launch.py
Leyendo el laser scan

En ROS 2 el topic suele seguir siendo /scan y el mensaje sensor_msgs/msg/LaserScan.

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class ReadScan(Node):
    def __init__(self):
        super().__init__('read_scan')
        self.sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.cb,
            10)

    def cb(self, msg):
        # filtra los “inf” generados cuando no hay eco
        rangos = [r for r in msg.ranges if not math.isinf(r)]
        if not rangos:
            return
        self.get_logger().info(
            f'más cercano: {min(rangos):.3f} m  | más lejano: {max(rangos):.3f} m')

def main():
    rclpy.init()
    rclpy.spin(ReadScan())
    rclpy.shutdown()
Añade sensor_msgs como dependencia:

ros2 pkg create --build-type ament_python --dependencies rclpy std_msgs sensor_msgs practica1
(o simplemente abre package.xml y añade sensor_msgs dentro de <depend> y vuelve a compilar).

4. Publicando comandos de velocidad
En ROS 2 los TurtleBot 3 usan /cmd_vel (Twist). Ejemplo de teleoperación mínima:

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class Teleop(Node):
    def __init__(self):
        super().__init__('teleoperacion')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 5)

    def run(self):
        try:
            while rclpy.ok():
                c = input("Comando (f:forward, l:left, r:right) > ")
                cmd = Twist()
                if c == 'f':
                    cmd.linear.x = 0.25
                elif c == 'l':
                    cmd.linear.x = 0.25
                    cmd.angular.z = 0.75
                elif c == 'r':
                    cmd.linear.x = 0.25
                    cmd.angular.z = -0.75
                else:
                    continue
                self.pub.publish(cmd)
        except (KeyboardInterrupt, EOFError):
            pass  # CTRL-C limpia

def main():
    rclpy.init()
    Teleop().run()
    rclpy.shutdown()
Añade geometry_msgs como dependencia y recompila.

5. Nodo que recibe scan y publica cmd_vel (productor + consumidor)
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class Evitacion(Node):
    def __init__(self):
        super().__init__('evitacion_obstaculos')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 5)
        self.sub = self.create_subscription(LaserScan, '/scan', self.cb, 10)

    def cb(self, msg):
        # distancia mínima frontal (±15°)
        center_indices = range(len(msg.ranges)//2 - 15, len(msg.ranges)//2 + 15)
        front = [msg.ranges[i] for i in center_indices if msg.ranges[i] < msg.range_max]
        min_front = min(front) if front else msg.range_max

        cmd = Twist()
        if min_front < 0.4:          # obstáculo cerca
            cmd.angular.z = 1.0      # gira in-place
        else:
            cmd.linear.x = 0.22      # avanza

        self.pub.publish(cmd)

def main():
    rclpy.init()
    rclpy.spin(Evitacion())
    rclpy.shutdown()
Resumen de comandos equivalentes ROS 1 → ROS 2

Acción	ROS 1	ROS 2
Compilar workspace	catkin_make	colcon build
Añadir script Python	marcar ejecutable + editar CMakeLists	declarar en setup.cfg → console_scripts
Lanzar nodo	rosrun pkg nodo.py o roslaunch	ros2 run pkg nodo o ros2 launch
Listar topics	rostopic list	ros2 topic list
Ver un topic	rostopic echo /topic	ros2 topic echo /topic
Información de un topic	rostopic info /topic	ros2 topic info /topic
Ver la definición de un mensaje	rosmsg show geometry_msgs/Twist	ros2 interface show geometry_msgs/msg/Twist
Últimas buenas prácticas ROS 2 (2025)
Python 3.12 es la versión predeterminada en Ubuntu 24.04.
Por defecto los ejecutables Python van en console_scripts; evita usar “scripts sueltos” fuera de empaquetado.
Usa --symlink-install al compilar para iterar rápido (no necesita reinstalar).
El “ros2 daemon” suele iniciarse automáticamente; fuerzalo con ros2 daemon start si notas que no encuentra nuevos tópicos.
Revisa los QOS si trabajas con sensores de alta frecuencia (DepthImage, rmw_qos_profile_sensor_data).
¡Con esto ya tienes tus apuntes totalmente migrados a ROS 2!