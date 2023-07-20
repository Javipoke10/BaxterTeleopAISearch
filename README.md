**Funcionamiento**:

Primero, se conecta un háptico por ethernet al ordenador y se ejecuta el docker y sus respectivos comandos. Esto hace que existan los topics del haptico en ros1.

Mediante el bridge, este se comunica con el programa en ros2 para recibir los movimientos del mismo y enviarlos mediante otro topic al baxter(ros1) que mediante un escalado de las variables, usa el metodo de cinematica inversa para moverse.

Como el objeto que devuelve el topic de los botones del háptico no existe en ros2, este se recibe en ros1, se procesa y se envia por otro topic a ros2 con el fin de usarlo en la interfaz.

También recibe de la interfaz mediante el mismo topic del movimiento la llamada para la parada y vuelta al punto de origen.

Por último, las cámaras se reciben en ros2 mediante el bridge.

Además, a través de un boton en la interfaz es posible comenzar con la runtina autónoma.

**Requisitos previos**:

-Ubuntu 20.04 LTS (No testeado en otras versiones)

-Ros1 Noetic (No testeado en otras versiones)

-Ros2 Foxy (No testeado en otras versiones)

-[Ros bridge](https://github.com/mgonzs13/ros2_utils_scripts)

-Docker con el contenedor citado en el apartado de [Docker](https://github.com/jdrew1303/ros_geomagic_touch_phantom_omni)

-Conexión por cable (Opcional, necesario para las cámaras)

-Código [Simple node](https://github.com/uleroboticsgroup/simple_node)

-[Baxter](https://github.com/maymohan/baxter_simulator/wiki/Installation-Instructions)

-Python3

En caso de querer usar visión:

-Intel Realsense (No testeado en otras cámaras)

-[Más requisitos](https://github.com/amigueldiez/vision-artificial-tiago-ros2)

**Vias futuras**:

* Añadir retroalimentación háptica.
* Reentrenar red con más datos.
* Bridges custom para los topic.


**Comandos ejecución baxter controller (Cada paso, distinta terminal):**

**DOCKER** ([source](https://github.com/jdrew1303/ros_geomagic_touch_phantom_omni))

sudo docker run -ti --rm --privileged -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix --net=host --env QT_X11_NO_MITSHM=1 gscar:PhantomOmni

export ROS_MASTER_URI="URL del Baxter"

roslaunch omni_driver ethernet.launch


**BAXTER** ([source](https://github.com/maymohan/baxter_simulator/wiki/Installation-Instructions))

cd "workspace de ros 1"

./baxter.sh

rosrun baxter_tools enable_robot.py -e

catkin_make

rosrun baxter_cont controller.py 


**BRIDGE** ([source](https://github.com/mgonzs13/ros2_utils_scripts))

rosconfig -d foxy -w "workspace de ros 2"

rosconfig -d noetic -w "workspace de ros 1" -m "ip del baxter"  -i "ip de tu red conectada al baxter"

ros2 run ros1_bridge dynamic_bridge 

**HAPTICO**

cd "workspace de ros 2"

rosconfig -d foxy -w "workspace de ros 2"

colcon build && ros2 run baxter_controller hapticBaxterControllerUi

**VISION**

Deteccion:([source](https://github.com/amigueldiez/vision-artificial-tiago-ros2))

rosconfig -d foxy -w "workspace de ros 2"

cd "workspace de ros 2"

ros2 launch cv_basics image_visualizer_launch.py

Camara:([source](https://github.com/IntelRealSense/realsense-ros#installation-instructions))

rosconfig -d foxy -w "workspace de ros 2"

ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true pointcloud.ordered_pc:=true align_depth.enable:=true enable_sync:=true

**TRANSFORMADA CAMARA A BASE**

rosconfig -d foxy -w "workspace de ros 2"

ros2 run tf2_ros static_transform_publisher 0.27 0.02 0.80 0.0 0.95 0.0 base camera_link

**SERVICIO TRANSFORMADA**

rosconfig -d foxy -w "workspace de ros 2"

cd "workspace de ros 2"

ros2 run tf_baxter_realsense calculate_position_node



