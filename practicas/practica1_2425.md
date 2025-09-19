# Práctica 1 de Robots Móviles: Construcción de mapas basados en landmarks

En esta práctica vamos a implementar un algoritmo de mapeado sencillo para construir mapas basados en *landmarks*. 

Un *landmark* es un lugar o un objeto que se puede distinguir más o menos fácilmente del resto del entorno. Qué consideremos un *landmark* dependerá del tipo de sensor que estemos usando. Por ejemplo para un laser 2D los *landmarks* pueden ser las esquinas de las paredes, ya que son sitios donde la dirección de los puntos detectados cambia bruscamente. Para una cámara pueden ser objetos de un color especial, o códigos QR colocados en las paredes.
Como para simplificar no tendremos en cuenta los errores de odometría, el mapa tendrá errores acumulativos con el tiempo.

## Proyecto de ejemplo

Os dejamos un mundo simulado y un fichero de configuración de `rviz` para que podáis probar vuestro algoritmo.

Por simplicidad el simulador usado es *stage*, que no es ni siquiera 3D, ni implementa físicas realistas, pero consume pocos recursos computacionales, es fácil crear y configurar mundos simulados en él y tampoco necesitamos más "realismo" para este tipo de algoritmo. 

Primero deberíais crear un *workspace* de ROS salvo que queráis reutilizar uno ya hecho:

```bash
mkdir robots_moviles_ws
cd robots_moviles_ws
mkdir src
```

A continuación descargamos y compilamos los ficheros de ejemplo:

```bash
#hay que hacerlo en la carpeta  "src" del workspace
cd src
git clone https://github.com/ottocol/mapeado-landmarks
catkin_make
cd ..
#para actualizar las variables de entorno y que encuentre el paquete
source devel/setup.bash
```

Para probar el simulador, lanzar:

```bash
roslaunch mapeado_landmarks mapeado_landmarks.launch
```

Se abrirán dos ventanas: una con el simulador *stage* y otra con rviz. En *stage* el robot viene representado por el cuadrado amarillo. Si seleccionas la opción de menú de `View > Data` podrás ver el campo de visión del laser. En la ventana de `rviz` verás un *warning* relativo al mapa ya que todavía no hay ningún programa que lo esté publicando (¡es el que debes implementar!).

El *.launch* también pone en marcha un proceso de *teleop* que permite mover al robot con el teclado. Las teclas de control se mostrarán en la terminal. Recuerda que la ventana del *teleop* debe tener el foco de teclado para que la teleoperación funcione.

 En el proyecto de ejemplo tenéis además un nodo de ROS en el archivo `crear_mapa.py` que os puede servir como plantilla para comenzar a desarrollar vuestro código. Este código no se pone en marcha automáticamente con el `.launch`, tenéis que ejecutarlo a mano (por ejemplo con `python crear_mapa.py`).

## Landmarks en ROS

En nuestro caso, en el entorno que se mueve el robot están las paredes del fondo y hay objetos relativamente pequeños de forma circular que os podéis imaginar como postes vistos desde arriba. Los landmarks van a ser estos postes. Tendréis que dar con algún método que os permita distinguir los postes de las paredes del fondo. Fijaos que en los objetos habrá unas pocas lecturas a una distancia similar y luego si el laser detecta el fondo cambiará mucho la distancia. Las paredes son segmentos a distancia similar entre sí pero mucho más largos que los objetos circulares.

Como cuando el robot se vaya moviendo irá detectando varias veces el mismo landmark solo deberíais considerarlo como nuevo si está "a una distancia suficiente" de cualquiera detectado hasta ahora. Este parámetro de distancia lo podéis fijar experimentalmente.

Los landmarks en ROS pueden representarse con objetos de tipo `Marker`. En `crear_mapa.py` tenéis un ejemplo de cómo publicar un `MarkerArray`, que no es más que una lista de `Marker` para visualizarlo en RViz. La configuración de RViz para el proyecto ya debería mostrar los landmarks de ejemplo (podéis ejecutarlo con `python crear_mapa.py`). Tendréis que cambiar estas coordenadas por los landmarks que detectéis.

## Sistemas de coordenadas en ROS. Transformación entre sistemas.

En cualquier robot móvil hará falta en general más de un sistema de coordenadas. Por ejemplo, sensores como las cámaras 3D o los láseres, cuando detectan información lo hacen en su propio sistema de referencia (los ejes coinciden con la posición física del sensor), pero típicamente no coincidirán con los ejes del cuerpo del robot. Por otro lado, el sistema de referencia del cuerpo del robot se mueve conforme se mueve éste, pero necesitamos también sistemas externos "fijos". Por ejemplo, el sistema de coordenadas de un mapa del entorno.

Esto hace que habitualmente sea necesario transformar coordenadas de un sistema a otro. Por ejemplo, en nuestro caso **necesitamos transformar las coordenadas de los landmarks detectados por el laser a coordenadas del mapa de landmarks**, para poder ir construyéndolo. Afortunadamente como veremos ROS nos va a ayudar mucho en esta tarea.

En el [REP (ROS Enhancement Proposal) 105](https://www.ros.org/reps/rep-0105.html) se definen una serie de sistemas de coordenadas estándar para robots móviles. Los que nos interesan de momento son los siguientes:

- `base_link`: El sistema de coordenadas de la plataforma base del robot. Es un sistema local al robot, que se mueve cuando este se mueve. Típicamente se coloca en el centro de rotación del robot.
- `odom`: Este sistema de coordenadas está fijo en el mundo (no se mueve cuando se mueve el robot) y su origen y orientación 0 coincide con la posición de partida del robot. Es lo que se conoce habitualmente como *odometría*: conforme el robot se va moviendo también va estimando su posición actual con respecto a este sistema.
- `map`: Es un sistema de coordenadas asociado a un mapa del entorno. Está fijo en el mundo (no se mueve cuando se mueve el robot) y su origen y orientación 0 es arbitrario y depende de quien haya creado el mapa. El robot puede estimar su posición con respecto a este sistema comparando el mapa con lo que perciben actualmente los sensores. Esto se conoce como *localización* (lo veremos en la práctica 2).

Además, como usaremos un laser para detectar obstáculos, tendremos un sistema asociado a él: `base_laser_link` es el nombre típico que se le suele dar, y es en el que se obtienen las medidas del laser.

Si tenéis lanzado el `mapeado_naive.launch` de ejemplo podéis ver estos sistemas y las relaciones entre ellos ejecutando en una terminal:

```bash
rosrun rqt_tf_tree rqt_tf_tree
```

![Figura 2: Grafo de transformaciones en ROS](tf_frames.png)

Debería aparecer una figura similar a la figura 2. Vemos que es un grafo en el que los nodos son los diferentes sistemas de coordenadas. Se conoce como *grafo de transformaciones*. Que aparezca una arista que va de un nodo A a un nodo B indica que ROS conoce la matriz que transforma el sistema A en el B. Si conocemos la transformación de A a B también tenemos la de B a A, ya que es la matriz inversa, de modo que la dirección del arco no es importante.

En un grafo de transformaciones **podemos calcular la transformación entre dos sistemas cualesquiera A y B siempre que haya un camino entre ambos**, sin importar la dirección de las flechas. 

Aunque parezca antiintuitivo, **la transformación que lleva de un sistema A al sistema B, en realidad nos sirve para pasar puntos dados en el sistema B al sistema A**. Tenéis más información sobre la razón de esto en el apéndice 1.

Por ejemplo, en nuestro caso queremos pasar un punto dado en el sistema `base_laser_link` al sistema `map` y por tanto necesitamos la transformación de `map` a `base_laser_link`. Vemos que hay un camino que une ambos nodos, de modo que podemos pedirle a ROS la transformación, vamos a ver cómo hacerlo.

El código básico en Python para transformar entre `map` y `base_laser_link` sería el siguiente:

```python
import tf2_ros

tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)
try:
	#Obtener la transformación entre un sistema "padre" e "hijo". 
	#Time(0) indica que queremos la última disponible
    trans = tfBuffer.lookup_transform('map', 'base_laser_link', rospy.Time(0))
#si se da alguna de estas excepciones no se ha podido encontrar la transformación    
except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException):
    rospy.logerr("No se ha podido encontrar la transformación")
```

Nótese que al comienzo de la ejecución es normal que salte la excepción, ya que las transformaciones se hacen gracias a mensajes publicados por ROS y es posible que inicialmente no haya ninguno disponible.

La transformación podemos aplicarla a un punto con coordenadas `(x,y,z)` como sigue:

```python
import tf2_geometry_msgs
#supongamos que tenemos las coordenadas (x,y) de una lectura del laser
#el tercer parámetro es z que no nos interesa en este caso
punto = Point(x,y,0)
ps = PointStamped(point=punto)
#"trans" sería la transformación obtenida anteriormente
punto_trans = tf2_geometry_msgs.do_transform_point(ps, trans)
```

Nota: en nuestro caso los sistemas `map` y `odom` ocupan la misma posición. Recordemos que `odom` es la posición inicial del robot "cuando se pone en marcha". Eso quiere decir que el (0,0,0) del mapa corresponderá con esta posición. Esta información se la damos a ROS en el `mapeado_naive.launch` mediante un `static_transform_publisher`, que es el tipo de nodo que se usa para publicar transformaciones estáticas (que no cambian con el tiempo) entre dos sistemas. Mira la línea 8 del archivo y la wiki de ROS sobre el [`static_transform_publisher`](http://wiki.ros.org/tf#static_transform_publisher) para más detalles.

## Evaluación de la práctica y fecha de entrega

Desarrollando correctamente todo lo anterior podéis obtener hasta un 7 en la nota. En la entrega debéis incluir:

- Todo el código fuente que realicéis. El código debe estar comentado
- Las pruebas del funcionamiento que hayáis realizado. Debéis probarlo en al menos el entorno que se os proporciona y otro que hagáis vosotros. En el apéndice 2 tenéis cómo modificar el mundo simulado en Stage. Para documentar las pruebas debéis poner una imagen con los landmarks detectados junto con los "reales" y opcionalmente un video del proceso de mapeado.

Si implementáis además las siguientes funcionalidades tendréis puntos adicionales:

- (Hasta 1 punto) Detectar otro tipo de landmarks que no sean postes, como esquinas o cualquier otro que se os ocurra
- (Hasta 1 punto) En lugar de mover al robot con el teclado podéis implementar algún algoritmo que haga que se mueva aleatoriamente sin chocar con los obstáculos. Podéis por ejemplo ver la media de las lecturas del laser a la izquierda y a la derecha del robot y girar en la dirección en que la media sea mayor, es decir, la que tiene más espacio libre. Podéis usar cualquier otro enfoque que se os ocurra.
- (hasta 1 punto) Leer/analizar un artículo científico sobre mapeado (estado actual de la investigación, algoritmos alternativos...) haciendo un resumen del contenido. Si no sabéis cuál elegir, preguntad al profesor

La práctica se podrá entregar **hasta las 23:59 horas del martes 15 de octubre**. La entrega se hará por moodle, comprimiendo todos los archivos en un único .zip

## Apéndice 1: Transformaciones y grafos de transformaciones

Esta sección es un complemento teórico por si queréis conocer más detalles sobre las transformaciones de sistemas de coordenadas, aunque de modo muy resumido. Seguiremos la notación usada por Alonzo Kelly en el libro [1] y algunos ejemplos del mismo. Os recomiendo que consultéis dicho libro para más información.

En general, en un robot móvil necesitaremos tener definidos diversos sistemas de coordenadas. Para poder pasar de un sistema a otro necesitaremos las matrices de transformación. Sin entrar en demasiados detalles, si trabajamos en 3D (como en ROS) son matrices 4x4 que encapsulan la traslación y rotación entre 2 sistemas. 

> Nota: En ROS en general no se trabaja con los coeficientes de la matriz de transformación directamente, sino que se especifica la traslación y rotación por separado, esta última en forma de *quaternion*.

Por ejemplo supongamos dos sistemas de coordenadas A y B como los que aparecen en la siguiente figura. Denotaremos por $T^A_B$ la matriz *que transforma el sistema A en el B*, es decir la matriz que aplicada a los ejes de A los hace coincidir con los de B. 

![Relación entre 2 sistemas de coordenadas (p. 48 de [1])](T_A_B.png)


Aunque parezca contradictorio a primera vista, **la matriz que transforma A en B nos sirve para pasar los puntos "al contrario", los expresados en B al sistema A**. Esto es lo que se denomina **dualidad operador-transformación** (el "operador" sería lo que nos permite convertir puntos de un sistema a otro). Formalmente si tenemos un punto expresado en coordenadas de B, $r^B$ y queremos obtener el punto en coordenadas de A:

$$r^A = T_B^A*r^B$$

Por ejemplo, supongamos para simplificar que la transformación entre A y B es una traslación de $(1,1,1)$ sin rotación, es decir que cogiendo los ejes de A y desplazándolos 1 unidad en cada eje obtenemos los ejes de B. Es fácil darse cuenta que el punto $p^B=(0,0,0)$ en coordenadas de B, es el $p^A=(1,1,1)$ en coordenadas de A. Es decir, la transformación que lleva A a B es la misma que sirve para pasar puntos de B a A. Nótese además que si quisiéramos pasar puntos de A a B bastaría con aplicar la inversa de $T_B^A$, en este caso la traslación $(-1,-1,-1)$.

En un caso general tendremos más de dos sistemas de coordenadas y las transformaciones T entre pares de sistemas. Con dichos sistemas como nodos y las transformaciones como arcos dirigidos (un arco de A a B para $T_B^A$) podemos montar un *grafo de transformaciones*.

En este grafo, la matriz de transformación entre los dos nodos cualesquiera para los que exista un camino en el grafo será la multiplicación de las matrices de transformación entre los pares de nodos que están por el camino, invirtiendo la matriz en caso de que vayamos en dirección contraria a la flecha.

![Ejemplo de grafo de transformación. Tomado de [1], p. 92](kelly_92.png)

La figura anterior contiene un ejemplo de grafo de transformaciones y de conversión de puntos usando el grafo. Supongamos que tenemos un punto en coordenadas de la caja $p^{box}$. Si queremos controlar la base del brazo para moverlo a dicho punto, necesitaremos las coordenadas del punto en el sistema de la base, $p^{base}$. Para hacer dicha conversión necesitamos la matriz que transforma *base* en *box* o con la notación que veníamos usando, $T^{base}_{box}$.

> Nótese que la idea en ROS es similar, en este caso le tendríamos que pedir a ROS un `trans = tfBuffer.lookup_transform('base', 'box', rospy.Time(0))`.y luego hacer un `do_transform_point(p_box, trans)`.

## Apéndice 2: Modificar los mundos en stage

El mundo simulado en *stage* que os dejamos como ejemplo está en la carpeta `world` y "dividido" en 2 archivos: `ejemplo.world` y `ejemplo.pgm`. 

En nuestro caso el mundo simulado es cargado por el `mapeado_naive.launch`, que pone en marcha un nodo ros de tipo `stageros`. Si quisierais hacerlo manualmente la orden sería `rosrun stage_ros stageros <nombre_del_world>` donde el nombre del fichero .world *debe incluir la trayectoria completa desde la raíz del sistema de archivos*.

El `world` es el fichero de configuración donde se definen los parámetros del mundo, el robot y los sensores que tiene, en la documentación de Stage podéis ver [más información sobre su sintaxis](https://player-stage-manual.readthedocs.io/en/latest/WORLDFILES/). El `world` carga a su vez el `pgm`.

Si abrís el `ejemplo.pgm` notaréis que es realmente el mapa del mundo (`pgm` es un formato gráfico para imágenes en escala de grises, aunque no es probable que os suene porque no se usa mucho actualmente).

Fijaos en que crear un nuevo mundo simulado es tan sencillo como crear una imagen en blanco y negro en cualquier aplicación (negro para los obstáculos/paredes, blanco para espacio vacío), guardarla en formato `.pgm` (casi todos los programas de dibujo/apps gráficas son compatibles) y en el fichero `.world` ajustar el tamaño en metros si lo deseáis. Para cambiar los sensores/propiedades del robot tendríais que miraros la documentación de Stage. En el caso del `ejemplo.world` el mundo simulado se carga de las líneas 61 a la 67

```bash
# load an environment bitmap
floorplan
( 
  name "plano"
  bitmap "rm.pgm"
  size [40.0 20.0 1.0] #tamaño del mundo en metros en x,y,z. z sería el "alto de las paredes"
)
```

> ROS suele estar instalado físicamente en `/opt/ros/_nombre-de-la-version_`, por ejemplo `/opt/ros/noetic`. Dentro de esta carpeta, en `/share/stage/worlds` y `share/stage_ros/world` tenéis muchos otros ficheros de mundos de ejemplo, algunos multirobot o con sensores adicionales como cámaras (aunque simplificadas porque la simulación de Stage no es 3D sino 2.5D).


## Apéndice 3: Referencias

[1] Alonzo Kelly, *Mobile Robotics: Mathematics, Models, and Methods*, Cambridge University Press, 2013.