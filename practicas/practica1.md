# Robots Móviles curso 25-26
# Práctica 1: Mapeado "simple"

La práctica a desarrollar consiste en implementar un par de algoritmos de mapeado simplificados: 

- El primero de ellos creará un mapa como una nube de puntos global en la que se irán fusionando las lecturas del laser 2D. 
- El segundo será un mapa de *landmarks*, o características distintivas del entorno. 

El título de mapeado "simple" viene de que no vamos a tener en cuenta los errores de la odometría del robot, que en el mundo real generarían mapas con demasiado error para ser útiles salvo que introdujéramos algún mecanismo de corrección.

> El objetivo pedagógico de la práctica es trabajar conceptos como la transformación entre sistemas de coordenadas y el manejo de datos de sensores como los laser 2D. También familiarizarse con la estructura básica de un nodo en ROS y la suscripción/publicación de mensajes.

> Para  ayudaros con las prácticas de la asignatura podéis usar un LLM o chatbot como ChatGPT, Claude, Grok, Gemini,... De hecho **os animo a que lo uséis** ya que son herramientas muy útiles no solo para escribir código en ROS sino también para descubrir métodos, enfoques o algoritmos que no conocíais. Otra cuestión es que no sería muy pedagógico que usárais la IA simplemente para copiar/pegar el enunciado y copiar/pegar el resultado a entregar. Siempre que entendáis lo que habéis conseguido con su ayuda (aunque sea la propia IA la que os lo explique) no habrá problema.

## Entornos simulados

Os dejamos un entorno simulado blablabla...

## Transformaciones de coordenadas en ROS 2

En cualquier robot móvil hará falta en general más de un sistema de coordenadas. Por ejemplo, sensores como las cámaras 3D o los láseres, cuando detectan información lo hacen en su propio sistema de referencia (los ejes coinciden con la posición física del sensor), pero típicamente no coincidirán con los ejes del cuerpo del robot. Por otro lado, el sistema de referencia del cuerpo del robot se mueve conforme se mueve éste, pero necesitamos también sistemas externos "fijos". Por ejemplo, el sistema de coordenadas de un mapa del entorno.

Esto hace que habitualmente sea necesario transformar coordenadas de un sistema a otro. Por ejemplo, en nuestro caso **necesitamos transformar las coordenadas de los puntos detectados por el laser a coordenadas del mapa**, para poder ir construyéndolo. Afortunadamente ROS nos va a ayudar mucho en esta tarea.

En el [REP (ROS Enhancement Proposal) 105](https://www.ros.org/reps/rep-0105.html) se definen una serie de sistemas de coordenadas estándar para robots móviles. Los que nos interesan de momento son los siguientes:

- `base_link`: El sistema de coordenadas de la plataforma base del robot. Es un sistema local al robot, que se mueve cuando este se mueve. Típicamente se coloca en el centro de rotación del robot.
- `odom`: Este sistema de coordenadas está fijo en el mundo (no se mueve cuando se mueve el robot) y su origen y orientación cero coincide con la posición de partida del robot. Es lo que se conoce habitualmente como *odometría*: conforme el robot se va moviendo también va estimando su posición actual con respecto a este sistema.
- `map`: Es un sistema de coordenadas asociado a un mapa del entorno. Está fijo en el mundo (no se mueve cuando se mueve el robot) y su origen y orientación cero es arbitrario y depende de quien haya creado el mapa. El robot puede estimar su posición con respecto a este sistema comparando el mapa con lo que perciben actualmente los sensores. Esto se conoce como *localización* (lo veremos en la práctica 2).

> Como nosotros somos los que vamos a crear el mapa, podemos poner su origen en el punto que queramos. Lo más sencillo es ponerlo en el $(0,0,0)$ de `odom`.

Además, como usaremos un laser para detectar obstáculos, tendremos un sistema asociado a él: `base_laser_link` es el nombre típico que se le suele dar, y es en el que se obtienen las medidas del laser.

## Mapas basados en nubes de puntos

Como sabemos, cada "barrido" del laser 2D nos da un *scan* con las distancias a los objetos más cercanos cada cierto número de grados. El objetivo en este apartado es convertir el *scan* en una nube de puntos y acumular la nube actual en una nuble "global".

1. Con trigonometría básica, sabiendo el ángulo al que apunta cada rayo y la distancia detectada, podemos convertir cada punto $(ángulo, distancia)$ a coordenadas $(x,y)$ en el sistema de referencia del laser.
2. Acumulando todos los puntos del *scan* podemos obtener una nube de puntos "local". En general las nubes de puntos tendrán coordenadas $(x.y.z)$ pero al ser el laser 2D solo tenermos las dos primeras.
3. Convirtiendo la nube de puntos local a un sistema de coordenadas global externo al robot ("map" u "odom") podemos ir acumulando todas las nubes locales en una global.

Pistas para la implementación:

- Las nubes de puntos en ROS se representan con mensajes de tipo `sensor_msgs/msg/PointCloud2`. Se pueden visualizar en RViz con un display de tipo `PointCloud2`.
- Los pasos 1 y 2 los podéis realizar en una sola operación si os ayudáis de la clase `LaserProjection`. Investigad su uso. Cuidado: está en el paquete `ros-jazzy-laser-geometry` (o en lugar de `jazzy` la versión que sea). Este paquete debería estar instalado en los laboratorios y la máquina virtual de la asignatura pero si usáis otra distribución aseguráos primero de que lo instaláis.
- Como sistema de coordenadas global y externo al robot os recomiendo que uséis "odom", nos evita el "problema" de dónde fijar el origen de "map".
- Conforme la nube global se vaya haciendo más grande, mantener todos los puntos será ineficiente. Podéis *voxelizar* los puntos, lo que quiere decir dividir el espacio en pequeños cubos y guardar solo un punto por cada uno de ellos (como es un laser 2D en realidad serán cuadrados en el plano, pero es la  misma idea). Investigad el tema.

## Mapas basados en landmarks

Un *landmark* es un lugar o un objeto que se puede distinguir más o menos fácilmente del resto del entorno. Qué consideremos un *landmark* dependerá del tipo de sensor que estemos usando. Por ejemplo para un laser 2D los *landmarks* pueden ser las esquinas de las paredes, ya que son sitios donde la dirección de los puntos detectados cambia bruscamente. Para una cámara pueden ser objetos de un color especial, o códigos QR colocados en las paredes.

En los mundos simulados que os pasamos, los *landmarks* más evidentes son los conos y los contenedores de basura, pero podéis intentar detectar otros como las esquinas de las paredes del fondo.

pistas para la implementación:

- En ROS 2 los landmarks se pueden representar mediante mensajes del tipo `visualization_msgs/Marker`. En lugar de publicarlos uno a uno podéis publicar un conjunto de *markers*:  creáis cada uno y los añadís todos en un `visualization_msgs/MarkerArray`. En RViz2 se pueden visualizar con un *display* de los mismos nombres (`Marker` si es uno solo o `MarkerArray` si es un conjunto).
- Cuando el robot se vaya moviendo irá detectando varias veces el mismo *landmark*. Solo deberíais considerarlo como nuevo si está "a una distancia suficiente" de cualquier *landmark* detectado hasta el momento. Este parámetro de distancia lo tendréis que fijar experimentalmente.








