
Tutoriales:
       - Paquetes y codigo usado en diferentes tutoriales.
============================================================================================================================================
VehiculoVigilanciaAutonomo_v0.1:
       - Vehiculo autonomo
	   - Usa Gmapping y AMCL
	   - Tiene la configuración usada en cliente (laptop) y servdor (RbPi) para estalecer una VPN usando OpenVPN.
	   - Tiene scripts usados para enviar notificaciones por Email, estos scripts se llaman desde el crontab de la RbPi.
	   - Paquetes ROS incluidos:
	        vva_base_controller - Control de los motores de las ruedas
            vva_base_odom - Calcula la odometría y la TF del Odom, y la publica.
            vva_depth2laserscan - Convierte las imagenes de profundidad del Kinect en laserscan, y lo publica en un topic.
            vva_description - Contiene el modelo URDF y los launch files para rviz y para la publicación de las TF del modelo URDF.
            vva_gazebo - Mundos de Gazebo
            vva_navigation - Configuración y launchfiles para Gmapping y AMCL
            vva_nav_test - Control por el teclado del robot
            vva_raspi_consolidado - Launchfile consolidado que inicia: odometría, OpenNI Kinect, depthimage to laserscan y base controller
============================================================================================================================================
VehiculoVigilanciaAutonomo_v0.2:
       - Vehiculo autonomo, corrige muchos problemas de la navegación de la v0.1
	   - El local costmap funciona correctamente lo que le permite hacer obstacle avoidance.
	   - Usa rtabmap para el mapping y localization
	   - Usa icp_odometry para producir la odometría a partir del laserScan del RPLIDAR
	   - Incluye la misma VPN de la v0.1
	   - Incluye los mismos scripts para notificaciones por Email de la v0.1
	   - Pre-requisitos:
	        Usar la ultima versión de rtabmap, la cual se descarga de github y se compila. La versión que viene en los repositorios de
	        Ubuntu Bionic no tiene el modulo "rgbd_relay".
	   - Folders y archivos incluidos:
	        ConfiguracionOpenVPN - Archivos de configuración de OpenVPN para cliente y servidor
	        SMTPNotificationScript - Scripts de crontab que se encargan de las notificaciones de eventos por email.
	        20200121_CompraDeLIDAR.ods - Tabla de evaluación de opciones usada para la adquisición del LIDAR.
	        20200131_LIDARsimulado.png - Imagen que muestra el LIDAR luego de parametrizarlo en Gazebo para que se parezca lo mas posible al RPLIDAR
	        20200131_rplidar_real.png - Imagen de muestra del RPLIDAR
	        20200225_CostosVehiculoVigilanciaAutonomo.ods - Resumen de los costos incurridos hasta el momento en hardware para la construcción del vehiculo
	        20200220_MapeoVelocidadVehiculo_ComandosMotor.ods - Mediciones para calibrar los motores y el vva_base_controller.
	   - Paquetes ROS incluidos:
	        vva_base_controller - Control de los motores de las ruedas, hay varios cambios con respecto a la v0.1.
	                              Esta versión hace un manejo diferencial de la velocidad de los motores de la izquierda y de la derecha para adaptarse
	                              mejor al navigation stack de ROS.
            vva_base_odom - [Deprecated] No usado en esta versión. Se usó sólo para hacer pruebas de odometría mientras se modificaba el vva_base_controller.
            vva_depth2laserscan - [Deprecated] No usado en esta versión.
            vva_description - Contiene el modelo URDF y los launch files para rviz y para la publicación de las TF del modelo URDF.
            vva_gazebo - Mundos de Gazebo.
            vva_lidar_odom - Odometría basada en LaserScan, permite usar los modulos hector_slam o icp_odometry (recomendado).
            vva_navigation - Contiene los modulos relativos a mapping y navigation stack, incluyendo rtabmap y move_base.
            vva_nav_test - Control por el teclado del robot y también por scripting.
            vva_raspi_consolidado - Launchfile consolidado que inicia en la RbPi: rplidar, kinect, control de los motores y rgbd_sync.
            vva_rplidar - [Deprecated] Modulo usado para probar el RPLIDAR de forma aislada, no necesario si se usa el vva_raspi_consolidado.
       - Comandos para probarlo:
            # En la RbPi: Validar que la sincronización NTP con el laptop este funcionando bien (ntpq -p), si no lo está:
               sudo systemctl stop ntp
               sudo ntpdate <Dir_IP_de_la_Laptop> # Ejecutar varias veces
               sudo systemctl start ntp
               ntpq -p
            # En la RbPi: Iniciar los modulos: rplidar, kinect, control de los motores y rgbd_sync
               roslaunch vva_raspi_consolidado vva_raspi_consolidado_rtabmap.launch
            # En la Laptop: Iniciar rviz y los state_publishers:
               roslaunch vva_description rviz_rtabmap.launch
            # En la Laptop: Iniciar odometria basada en laserscan:
               roslaunch vva_lidar_odom vva_lidar_odom.launch
            # En la Laptop: Iniciar los modulos: rgbd_relay y el rtabmap:
               roslaunch vva_navigation vva_rtabmap.launch simulation:=false localization:=[false|true]
            # En la laptop: Iniciar el modulo de navegación (move_base):
               roslaunch vva_navigation vva_move_base.launch simulation:=false
            # En la Laptop: Usar el teleop (se puede usar la navegación autonoma e ir descubriendo el mapa al tiempo si se quiere):
               roslaunch vva_nav_test vva_teleop2.launch
============================================================================================================================================
VehiculoVigilanciaAutonomo_v0.3:
     - Implementa lo mismo del v0.2, con el mismo hardware de la v0.2, con las siguientes modificaciones:
     - Cambio en el hardware y en el base controller: se adicionó una tarjeta driver para los motores, ahora hay dos tarjetas cada una con dos motores,
       para evitar un cuello de botella en la corriente hacia los motores.
     - Se incluye un módulo para realizar patrullaje autónomo, que envía coordenadas "goal" en el mapa de forma secuencial.
     - Se corrije error en la simulación de Gazebo que no premitía mostrar el Lidar debido a incompatibilidad con la tarjeta gráfica del laptop.
       Se cambió el plugin usado en vva.gazebo
     - Se adiciona hoja electrónica "20200304_CalculosSeleccionNuevosMotoresYDiseñoHW.ods" con la planeación del diseño del nuevo hardware
       y estimación de costos.
     - Pre-requisitos:
          Usar la ultima versión de rtabmap, la cual se descarga de github y se compila. La versión que viene en los repositorios de
          Ubuntu Bionic no tiene el modulo "rgbd_relay".

	   - Folders y archivos incluidos:
          20200220_MapeoVelocidadVehiculo_ComandosMotor.ods - Mediciones para calibrar los motores y el vva_base_controller.
          20200225_CostosVehiculoVigilanciaAutonomo.ods - Resumen de los costos incurridos hasta el momento en hardware para la construcción del vehiculo
          20200304_CalculosSeleccionNuevosMotoresYDiseñoHW.ods - Estimaciones de las variables necesarias para el nuevo hardware: motores, baterias, etc
                                                                 propuestas de diferentes diseños para el nuevo hardware y estimaciones de costos.
          ConfiguracionOpenVPN - Archivos de configuración de OpenVPN para cliente y servidor
          SMTPNotificationScript - Scripts de crontab que se encargan de las notificaciones de eventos por email.
	   - Paquetes ROS incluidos:
          vva_base_controller - Control de los motores de las ruedas, tiene sólo un cambio con respecto a la v0.2: ahora maneja dos tarjetas
                                controladoras de motores
          vva_description - Contiene el modelo URDF y los launch files para rviz y para la publicación de las TF del modelo URDF. Sólo tiene un pequeño
                            cambio con respecto a la v0.2 en vva.gazebo para reemplazar el plugin de simulación del lidar.
          vva_gazebo - Mundos de Gazebo.
          vva_lidar_odom - Odometría basada en LaserScan, permite usar los modulos hector_slam o icp_odometry (recomendado).
          vva_navigation - Contiene los modulos relativos a mapping y navigation stack, incluyendo rtabmap y move_base.
          vva_nav_test - Control por el teclado del robot y también por scripting.
          vva_patrolling - Envía goals de manera secuencial a move_base para que el robot realice un itinerario.
          vva_raspi_consolidado - Launchfile consolidado que inicia en la RbPi: rplidar, kinect, control de los motores y rgbd_sync.
     - Comandos para probarlo:
          # En la RbPi: Validar que la sincronización NTP con el laptop este funcionando bien (ntpq -p), si no lo está:
             sudo systemctl stop ntp
             sudo ntpdate <Dir_IP_de_la_Laptop> # Ejecutar varias veces
             sudo systemctl start ntp
             ntpq -p
          # En la RbPi: Iniciar los modulos: rplidar, kinect, control de los motores y rgbd_sync
             roslaunch vva_raspi_consolidado vva_raspi_consolidado_rtabmap.launch
          # En la Laptop: Iniciar rviz y los state_publishers:
             roslaunch vva_description rviz_rtabmap.launch
          # En la Laptop: Iniciar odometria basada en laserscan:
             roslaunch vva_lidar_odom vva_lidar_odom.launch
          # En la Laptop: Iniciar los modulos: rgbd_relay y el rtabmap:
             roslaunch vva_navigation vva_rtabmap.launch simulation:=false localization:=false
          # En la Laptop: Usar el teleop:
             roslaunch vva_nav_test vva_teleop2.launch
             
          # Mover el robot para generar el mapa, dar vueltas de 360° en diferentes lugares,
          # una vez este listo, cerrar rtabmap y el teleop.

          # En la Laptop: Iniciar los modulos: rgbd_relay y el rtabmap:
             roslaunch vva_navigation vva_rtabmap.launch simulation:=false localization:=true
          # En la laptop: Iniciar el modulo de navegación (move_base):
             roslaunch vva_navigation vva_move_base.launch simulation:=false
          # En la laptop: Lanzar el modulo de patrullaje autonomo
             roslaunch vva_patrolling vva_patrolling.launch
============================================================================================================================================
VehiculoVigilanciaAutonomo_v0.4:
     - Desarrollado en el Sprint 3.
     - Se adicionan encoders y control PID para las ruedas
     - Se adicionan los archivos en vva_base_controller/src/: vva_base_encoders_publisher.py y vva_wheels_calibration.py
     - Se reescribe por completo el archivo vva_base_controller/src/vva_base_controller.py
     - Se adiciona calibración automática para encontrar la velocidad máxima y mínima de los motores de las ruedas.
     - Se adiciona la capacidad de cambiar el tilt del kinect a través de un topic "vva_kinect/tilt_angle"
     - Se adiciona la capacidad de cambiar los modos del LED del Kinect a través de un topic "vva_kinect/led_option"
     - Se adiciona el registro de la incinación del vehículo con respecto a los ejes "x" y "y", usando los acelerómetros
       del Kinect
     
     - Pre-requisitos:
          - Usar la ultima versión de rtabmap, la cual se descarga de github y se compila. La versión que viene en los repositorios de
            Ubuntu Bionic no tiene el modulo "rgbd_relay". Esta se encuentra en: "$ROS_HOME_WS/ROSCompiledPackages/ROSCompiledPackages_ws/src/"

	   - Folders y archivos incluidos:
          20200225_CostosVehiculoVigilanciaAutonomo.ods - Resumen de los costos incurridos hasta el momento en hardware para la construcción del vehiculo
          20200304_CalculosSeleccionNuevosMotoresYDiseñoHW.ods - Estimaciones de las variables necesarias para el nuevo hardware: motores, baterias, etc
                                                                 propuestas de diferentes diseños para el nuevo hardware y estimaciones de costos.
          20200504_MapeoVelocidadVehiculo_ComandosMotor.ods - Calculos sobre los ticks de los encoders de las ruedas y sobre los limites de los motores para
                                                              giros en curva (no sobre su propio eje).
          ConfiguracionOpenVPN - Archivos de configuración de OpenVPN para cliente y servidor
          diagnosticos_tf_tree_y_rqt_graph - Imagenes de los diagramas del arbol de transformadas TF-Tree y del rqt_graph
          EstadisticasControlPID - Estadísticas tabuladas y graficadas de los componentes Proporcional, Integral y Derivativo del controlador PID para definir
                                   los valores de los coeficientes Kp, Ki y Kd.
          SMTPNotificationScript - Scripts de crontab que se encargan de las notificaciones de eventos por email.
          VVA_ws/src/subir_archivos.sh - Script para copiar los paquetes necesarios de la Laptop a la RbPi.
     
	   - Paquetes ROS incluidos:
          vva_base_controller - Control de los motores de las ruedas, cambio total con respecto a la v0.3: Incluye el manejo de los encoders y calibración.
          vva_description - Contiene el modelo URDF y los launch files para rviz y para la publicación de las TF del modelo URDF. Cambios con respecto
                            a la v0.3: Se incluyen articulaciones moviles para el Kinect y para el balanceo del vehículo.
          vva_gazebo - Mundos de Gazebo.
          vva_kinect_aux - Paquete nuevo. Encargado de cambiar el "tilt" del Kinect y de publicar la posición de las articulaciones de acuerdo con el IMU
                           del Kinect.
          vva_lidar_odom - Odometría basada en LaserScan, permite usar los modulos hector_slam o icp_odometry (recomendado).
          vva_navigation - Contiene los modulos relativos a mapping y navigation stack, incluyendo rtabmap y move_base.
          vva_nav_test - Control por el teclado del robot y también por scripting.
          vva_patrolling - Envía goals de manera secuencial a move_base para que el robot realice un itinerario.
          vva_raspi_consolidado - Launchfile consolidado que inicia en la RbPi: OpenNI Kinect, kinect_aux, rgbd_sync, base encoders publisher
                                  y base controller.
     
     - Problemas conocidos:
          - El kinect no es visible cuando se simula en Gazebo.
          - El SLAM no funciona bien en sitios oscuros, debido a que Rtabmap usa la cámara RGB del Kinect.
          - El RPlidar a veces no inicia si hay otros nodos corriendo en la RbPi debido a problemas de voltaje.
            Se debe ejecutar de primero.
     
     - Comandos para probarlo en simulación:
     
          # Desplegar el modelo en Gazebo:
             roslaunch vva_gazebo vva_world.launch
          # Iniciar rviz y los state_publishers:
             roslaunch vva_description rviz_rtabmap.launch
          # Iniciar el rtabmap (mapping and localization):
             roslaunch vva_navigation vva_rtabmap.launch simulation:=true localization:=false
          # Usar el teleop:
             roslaunch vva_nav_test vva_teleop2.launch
          # -------------------------------------------------------------------------------------------------------   
          # Mover el robot para generar el mapa, dar vueltas de 360° en diferentes lugares,
          # una vez este listo, cerrar rtabmap y el teleop.
          # -------------------------------------------------------------------------------------------------------   
          # Iniciar el rtabmap (mapping and localization):
             roslaunch vva_navigation vva_rtabmap.launch simulation:=true localization:=true
          # Iniciar el modulo de navegación (move_base):
             roslaunch vva_navigation vva_move_base.launch simulation:=true
          # Actualizar los goals en vva_patrolling.launch y lanzar el modulo de patrullaje autonomo
             roslaunch vva_patrolling vva_patrolling.launch

     
     - Comandos para probarlo en el hardware:
     
          # En la RbPi: En caso de ser necesario. Para apagar los puertos USB: LIDAR y Kinect (Esto deshabilita todos los puertos):
             # Apagar los puertos USB:
               sudo su -
               echo '1-1' > /sys/bus/usb/drivers/usb/unbind
             # Encender los puertos USB:
               sudo su -
               echo '1-1' > /sys/bus/usb/drivers/usb/bind
          # En la RbPi: Validar que la sincronización NTP con el laptop este funcionando bien (ntpq -p), si no lo está:
             sudo systemctl stop ntp
             sudo ntpdate <Dir_IP_de_la_Laptop> # Ejecutar varias veces
             sudo systemctl start ntp
             ntpq -p
          # En la RbPi: Si es necesario, calibrar los motores de las ruedas
             rm /home/ubuntu/ROS/VehiculoVigilanciaAutonomo_v0.4/VVA_ws/src/vva_base_controller/wheels_calibration/wheels_calibration.txt
             roslaunch vva_base_controller vva_wheels_calibration.launch
          # -------------------------------------------------------------------------------------------------------   
          # En la RbPi: Iniciar el modulo: rplidar (se separó del consolidado porque fallaba al iniciar el escaneo)
             roslaunch vva_raspi_consolidado vva_raspi_rplidar.launch
          # En la RbPi: Iniciar los modulos: kinect, kinect_aux, rgbd_sync, encoders publisher y base controller
             roslaunch vva_raspi_consolidado vva_raspi_consolidado_rtabmap.launch
          # -------------------------------------------------------------------------------------------------------   
          # En la Laptop: Iniciar rviz y los state_publishers:
             roslaunch vva_description rviz_rtabmap.launch
          # En la Laptop: Iniciar odometria basada en laserscan:
             roslaunch vva_lidar_odom vva_lidar_odom.launch
          # En la Laptop: Iniciar los modulos: rgbd_relay, el rtabmap y el rtabmapviz:
             roslaunch vva_navigation vva_rtabmap.launch simulation:=false localization:=false
          # En la Laptop: Usar el teleop:
             roslaunch vva_nav_test vva_teleop2.launch
          # -------------------------------------------------------------------------------------------------------   
          # Mover el robot para generar el mapa, dar vueltas de 360° en diferentes lugares,
          # una vez este listo, cerrar rtabmap y el teleop.
          # -------------------------------------------------------------------------------------------------------   
          # En la Laptop: Iniciar los modulos: rgbd_relay y el rtabmap:
             roslaunch vva_navigation vva_rtabmap.launch simulation:=false localization:=true
          # En la laptop: Iniciar el modulo de navegación (move_base):
             roslaunch vva_navigation vva_move_base.launch simulation:=false
          # En la laptop: Actualizar los goals en vva_patrolling.launch y lanzar el modulo de patrullaje autonomo
             roslaunch vva_patrolling vva_patrolling.launch
============================================================================================================================================
VehiculoVigilanciaAutonomo_v0.5:
     - Desarrollado en el Sprint 4.
     - Esta versión tiene importantes mejoras en la navegación autónoma, además de incorporar la detección de obstáculos bajos y de huecos.
     - Se cambia la configuración del stack de navegación para implementar el uso de costmaps basados en capas por medio de plugins.
     - Se deja probada la configuración de los plugins "voxel_grid" y "spatio_temporal_voxel_layer" que implementan costmaps en 2.5D.
       Estos se pueden habilitar opcionalmente para la detección de obstáculos que están por debajo de la línea de visión del Lidar,
       por medio del uso del Kinect.
     - Se deja probado el uso del nodo "costmap_2d/costmap_2d_cloud" para poder mostrar en rviz el "voxel_grid" como un PointCloud.
       "spatio_temporal_voxel_layer" no requiere este plugin para mostrar el voxel grid en gazebo.
     - Se adiciona la deteccioń de obstáculos que están por debajo de la línea de visión del Lidar, usando el Kinect y el nodo
       "depth_nav_tools/laserscan_kinect", invocado desde "vva_navigation/vva_move_base.launch".
     - Se adiciona la detección de huecos, gradas o "abismos" por medio del Kinect, usando el nodo "vva_cliff_detector/vva_cliff_detector",
       invocado desde "vva_navigation/vva_move_base.launch". También se usa el plugin "vva_cliff_detector_layer::VVACliffDetectorLayer".
     - Se modificó "vva_kinect_aux" para corregir el salto producido al cambiar el tilt del Kinect y se dejó la articulación de la base con
       respecto al eje “y” siempre en 0 grados.
     - Se deja probada la configuración del módulo opcional "rtabmap_ros/point_cloud_xyz" para generar PointClouds a partir del depth-image.
       Para probarlo usar: "roslaunch vva_navigation vva_depthimage_2_pointcloud.launch simulation:=false"
     - Se crea script personalizable para generación opcional de estadísticas a partir de topics.
       Para probarlo usar: "rosrun vva_nav_test vva_topic_analyzer.py"
     - Se desarrolló un nodo para la corrección de la odometría "vva_lidar_odom/vva_odom_correction.py", el cual reinicia la odometría de forma
       periódica.
     - Se desarrolló un nodo para la corrección de la navegación "vva_navigation/vva_navigation_correction.py", el cual fija un goal más
       cercano para desbloquear al robot.
     
     - Pre-requisitos:
          - Usar la ultima versión de rtabmap, la cual se descarga de github y se compila. La versión que viene en los repositorios de
            Ubuntu Bionic no tiene el modulo "rgbd_relay". Esta se encuentra en: "$ROS_HOME_WS/ROSCompiledPackages/ROSCompiledPackages_ws/src/rtabmap_ros/"
          - Usar el paquete depth_nav_tools/laserscan_kinect, el cual ya está descontinuado en Melodic pero se puede descagar y compilar. Se encuentra en:
            "$ROS_HOME_WS/ROSCompiledPackages/ROSCompiledPackages_ws/src/depth_nav_tools/laserscan_kinect/"

	   - Folders y archivos incluidos:
          20200225_CostosVehiculoVigilanciaAutonomo.ods - Resumen de los costos incurridos hasta el momento en hardware para la construcción del vehiculo
          20200304_CalculosSeleccionNuevosMotoresYDiseñoHW.ods - Estimaciones de las variables necesarias para el nuevo hardware: motores, baterias, etc
                                                                 propuestas de diferentes diseños para el nuevo hardware y estimaciones de costos.
          20200504_MapeoVelocidadVehiculo_ComandosMotor.ods - Calculos sobre los ticks de los encoders de las ruedas y sobre los limites de los motores para
                                                              giros en curva (no sobre su propio eje).
          20200524_Optimization_CliffDetector_LaserscanKinect.ods - Resultados pruebas de optimización de los parámetros de los nodos vva_cliff_detector y
                                                                    laserscan_kinect.
          ConfiguracionOpenVPN - Archivos de configuración de OpenVPN para cliente y servidor
          diagnosticos_tf_tree_y_rqt_graph - Imagenes de los diagramas del arbol de transformadas TF-Tree y del rqt_graph
          EstadisticasControlPIDYAcelerac - Estadísticas tabuladas y graficadas del controlador PID para definir los valores de los coeficientes Kp, Ki y Kd.
                                            Estadísticas para determinar los límites de aceleración del vehículo para la configuración del base_local_planner.
                                            Estadísticas de la cantidad de loop closures reportados por rtabmap en diferentes condiciones de luz.
          SMTPNotificationScript - Scripts de crontab que se encargan de las notificaciones de eventos por email.
          VVA_ws/src/subir_archivos.sh - Script para copiar los paquetes necesarios de la Laptop a la RbPi.
     
	   - Paquetes ROS incluidos:
          vva_base_controller - Control de los motores de las ruedas, no hay muchos cambios con respecto a la v0.4.
          vva_cliff_detector - Módulo que es una apropiación del código que viene con depth_nav_tools para cambiar ligeramente el comportamiento
                               referente a los topics. Se encarga de detectar huecos o "abismos".
          vva_cliff_detector_layer - Apropiación de código que viene con depth_nav_tools para disminuir la verbosidad del módulo. Se encarga de
                                     incluir en el costmap los huecos o "abismos".
          vva_description - Contiene el modelo URDF y los launch files para rviz y para la publicación de las TF del modelo URDF.
          vva_gazebo - Mundos de Gazebo.
          vva_kinect_aux - Encargado de cambiar el "tilt" del Kinect y de publicar la posición de las articulaciones de acuerdo con el IMU
                           del Kinect.
          vva_lidar_filter - Filtro para LaserScans usado para disminuir el ángulo que abarca el laserscan_kinect, y así remover ruido generado
                            por el Kinect en un angulo específico de su visión.
          vva_lidar_odom - Odometría basada en LaserScan, usa el módulo icp_odometry de rtabmap.
          vva_msgs - Paquete donde están las definiciones de los mensajes personalizados usados por VVA.
          vva_navigation - Contiene los modulos relativos a mapping, localization y navigation stack, incluyendo rtabmap y move_base. En esta versión
                           se adicionó un nodo llamado vva_navigation_correction.
          vva_nav_test - Control por el teclado del robot y también por scripting. En esta versión se adicionó un script para generar estadísticas a
                         partir de diferentes topics.
          vva_patrolling - Envía goals de manera secuencial para que el robot realice un itinerario. En esta versión se migró de C++ a Python.
          vva_raspi_consolidado - Launchfile consolidado que inicia en la RbPi: OpenNI Kinect, kinect_aux, rgbd_sync, base encoders publisher
                                  y base controller.
          vva_topics_sync - Paquete que usa message_filters para sincronizar los topics image_raw y camera_info generados por el Kinect.
     
     - Problemas conocidos:
          - El modelo no se muestra en Gazebo, debido a las joints tipo "revolute" del Kinect y de la base.
          - vva_cliff_detector no funciona con la simulación en Gazebo, muestra el error: "Could not perform stairs detection:
            Depth image has unsupported encoding: 32FC1"
          - A veces la transmisión del depth image tiene interrupciones o simplemente deja de llegar por lo cual el stack de navegación se
            bloquea. Todavía no se sabe la causa de esto, tal vez es por debilidad de la señal WiFi en algunos lugares.
          - El módulo vva_navigation_correction no funciona en simulación debido a que depende de un topic generado por vva_odom_correction
            el cual no es usado en simulación.
     
     - Comandos para probarlo en simulación:
     
          # Desplegar el modelo en Gazebo:
             roslaunch vva_gazebo vva_world.launch
          # Iniciar rviz y los state_publishers:
             roslaunch vva_description rviz_rtabmap.launch simulation:=true
          # -------------------------------------------------------------------------------------------------------   
          # Mapeo:
          # -------------------------------------------------------------------------------------------------------   
          # Iniciar el rtabmap (mapping and localization):
             roslaunch vva_navigation vva_rtabmap.launch simulation:=true localization:=false
          # Usar el teleop:
             roslaunch vva_nav_test vva_teleop2.launch
          # Mover el robot para generar el mapa, dar vueltas de 360° en diferentes lugares,
          # En la Laptop: En caso de querer reiniciar el mapa ejecutar:
             rosservice call /rtabmap_laptop/reset
          # -------------------------------------------------------------------------------------------------------   
          # una vez este listo, cerrar rtabmap y el teleop.
          # Localización:
          # -------------------------------------------------------------------------------------------------------   
          # Iniciar el rtabmap (mapping and localization):
             roslaunch vva_navigation vva_rtabmap.launch simulation:=true localization:=true
          # En la laptop: Iniciar los modulos: navegación (move_base), image-camera_info sync, vva_cliff_detector, laserscan_kinect y laserscan_kinect_filter:
             roslaunch vva_navigation vva_move_base.launch simulation:=true
          # REQUIERE MODIFICACION PARA LA SIMULACIÓN -- En la laptop: Iniciar el módulo vva_navigation_correction:
             roslaunch vva_navigation vva_navigation_correction.launch
          # Actualizar los goals en vva_patrolling.launch y lanzar el modulo de patrullaje autonomo
             roslaunch vva_patrolling vva_patrolling.launch
     
     - Comandos para probarlo en el hardware:
     
          # En la RbPi: En caso de ser necesario. Para apagar los puertos USB: LIDAR y Kinect (Esto deshabilita todos los puertos):
             # Apagar los puertos USB:
               sudo su -
               echo '1-1' > /sys/bus/usb/drivers/usb/unbind
             # Encender los puertos USB:
               sudo su -
               echo '1-1' > /sys/bus/usb/drivers/usb/bind
          # En la RbPi: Validar que la sincronización NTP con el laptop este funcionando bien (ntpq -p), si no lo está:
             sudo systemctl stop ntp
             sudo ntpdate <Dir_IP_de_la_Laptop> # Ejecutar varias veces
             sudo systemctl start ntp
             ntpq -p
          # En la RbPi: Si es necesario, calibrar los motores de las ruedas
             rm /home/ubuntu/ROS/VehiculoVigilanciaAutonomo_v0.5/VVA_ws/src/vva_base_controller/wheels_calibration/wheels_calibration.txt
             roslaunch vva_base_controller vva_wheels_calibration.launch
          # -------------------------------------------------------------------------------------------------------   
          # En la RbPi: Iniciar el modulo: rplidar (se separó del consolidado porque fallaba al iniciar el escaneo)
             roslaunch vva_raspi_consolidado vva_raspi_rplidar.launch
          # En la RbPi: Iniciar los modulos: kinect, kinect_aux, rgbd_sync, encoders publisher y base controller
             roslaunch vva_raspi_consolidado vva_raspi_consolidado_rtabmap.launch
          # -------------------------------------------------------------------------------------------------------   
          # En la Laptop: Iniciar rviz y los state_publishers:
             roslaunch vva_description rviz_rtabmap.launch simulation:=false
          # En la Laptop: Iniciar odometria basada en laserscan:
             roslaunch vva_lidar_odom vva_lidar_odom.launch
          # -------------------------------------------------------------------------------------------------------   
          # Mapeo:
          # -------------------------------------------------------------------------------------------------------   
          # En la Laptop: Iniciar los modulos: rgbd_relay, el rtabmap y el rtabmapviz, en modo mapeo:
             roslaunch vva_navigation vva_rtabmap.launch simulation:=false localization:=false
          # En la Laptop: Usar el teleop:
             roslaunch vva_nav_test vva_teleop2.launch
          # Mover el robot para generar el mapa, dar vueltas de 360° en diferentes lugares,
          # En la Laptop: En caso de querer reiniciar el mapa ejecutar:
             rosservice call /rtabmap_laptop/reset
          # -------------------------------------------------------------------------------------------------------   
          # una vez este listo, cerrar rtabmap y el teleop.
          # Localización:
          # -------------------------------------------------------------------------------------------------------   
          # En la Laptop: Iniciar el nodo de corrección de la odometría:
             roslaunch vva_lidar_odom vva_odom_correction.launch
          # En la Laptop: Iniciar los modulos: rgbd_relay, el rtabmap y el rtabmapviz, en modo localizacion:
             roslaunch vva_navigation vva_rtabmap.launch simulation:=false localization:=true
          # En la laptop: Iniciar los modulos: navegación (move_base), image-camera_info sync, vva_cliff_detector, laserscan_kinect y laserscan_kinect_filter:
             roslaunch vva_navigation vva_move_base.launch simulation:=false
          # En la laptop: Iniciar el módulo vva_navigation_correction:
             roslaunch vva_navigation vva_navigation_correction.launch
          # En la laptop: Actualizar los goals en vva_patrolling.launch y lanzar el modulo de patrullaje autonomo:
             roslaunch vva_patrolling vva_patrolling.launch
============================================================================================================================================
VehiculoVigilanciaAutonomo_v0.6:
     - Developed in the Sprint 5.
     - Supports the use of the microphones array of the Kinect by detecting a wake-word after which the user can record a 4 seconds audio clip
       with a voice command.
     - Introduces a Mobile Client App that supports the use of the microphone of the mobile phone to record voice commands.
     - The voice commands are processed by the Speech Recognition node in the package vva_voice_interact_laptop. This node is based on
       Mozilla DeepSpeech v0.6.1 and currently supports only american english language.
     - All the audio clips captured throught the microphones array of the Kinect are saved and preserved for future use as datasets for
       training DeepSpeech. The audio clips captured through the Mobile App are not preserved.
     - Introduces an architecture to implement the Intent Recognition, to interpret the Voice Commands. Several Intent Recognition nodes can be
       deployed in the package vva_user_intents, each node specialized in different tasks. Currently, there is only one node in charge of the
       navigation related tasks.
     - The Intent Recognition node in charge of the navigation tasks (vva_navigation_intent.py) supports the following commands: "navigate to",
       "start patrolling" and "stop navigation".
     - The known issue of v0.5 regading vva_navigation_correction not working on simulation is now fixed.
     - The launch files "vva_odom_correction.launch", "vva_move_base.launch" and "vva_navigation_correction.launch" were consolidated in
       one launch file called "vva_consolidated_nav.launch".
     
     - Pre-requisites:
          - Use the last version of rtabmap, which is downloaded from github and compile. The version that comes in the Ubuntu Bionic repos doesn't
            have the module "rgbd_relay". This is found in: "$ROS_HOME_WS/ROSCompiledPackages/ROSCompiledPackages_ws/src/rtabmap_ros/"
          - Use the package depth_nav_tools/laserscan_kinect, it is not available in Melodic but it can be downloaded and compiled. It is found in:
            "$ROS_HOME_WS/ROSCompiledPackages/ROSCompiledPackages_ws/src/depth_nav_tools/laserscan_kinect/"
          - If you want to make a modification of the request or the response of the ROS Service used in the Mobile App, then rosjava is needed to
            generate the .jar file. It is found in: "$ROS_HOME_WS/ROSCompiledPackages/rosjava_minimal"

	   - Included folders and files:
          20200225_CostosVehiculoVigilanciaAutonomo.ods - Summary of the costs of hardware used so far to build the vehicle
          20200504_MapeoVelocidadVehiculo_ComandosMotor.ods - Calculations about the ticks of the wheels encoders and about the motor limits to turn
                                                              in curves (not in place rotations)
          20200524_Optimization_CliffDetector_LaserscanKinect.ods - Results of optimization tests of the parameters of the nodes vva_cliff_detector and
                                                                    laserscan_kinect.
          20200620_CalculosSeleccionNuevosMotoresYDiseñoHW.ods - Estimations of the required specifications for the new hardware: motors, batteries, etc
                                                                 proposals of different designs for the new hardware and estimations of the costs.
          20200725_TestsOfHARKAudioSourceLocation.ods - Tests of HARK library about the accuracy to identify the location of the source of the sound.
          20200729_TestsOfSpeechRecognitionModels.ods - Tests about the accuracy of Speech Recognition with DeepSpeech v0.6.1 (English) and
                                                        PocketSphinx 5 pre-alpha (Spanish).
          Android - Contains the Android Studio project (VVA_Mobile_Client) of the Mobile App and a rosjava workspace (vva_rosjava_ws) to generate
                    the necessary .jar files to allow the Mobile App to communicate with the ROS service.
          ConfiguracionOpenVPN - Configuration files of OpenVPN for client and server
          Design - Design document and diagrams of VVA
          diagnosticos_tf_tree_y_rqt_graph - Images of diagrams of the TF-Tree and rqt_graph
          EstadisticasControlPIDYAcelerac - Tabulated statistics and graphs of the PID controller to define the values of the coeficients Kp, Ki y Kd.
                                            Statistics to determine the acceleration limits of the vehicle for the configuration of the base_local_planner.
                                            Statistics of the quantity of loop closures reported by rtabmap in different light conditions.
          OtherScripts - Currently, only has a script to prepare the datasets for training a DeepSpeech model.
          SMTPNotificationScript - Crontab scripts in charge of email notifications of events.
          VVA_ws/src/upload_files_to_rbpi.sh - Script to copy selected packages from the Laptop to the RbPi.
          ../Backup/create_backup.sh - Creates a backup upon its execution
     
	   - Included ROS packages:
          vva_base_controller - Control of the motors of the wheels. Runs on the RbPi.
          vva_cliff_detector - This module is a fork of the code of depth_nav_tools to slightly change the behavior regarding the topics.
                               It is in charge of detecting holes or cliffs.
          vva_cliff_detector_layer - Fork of the code of depth_nav_tools to reduce the verbosty of the module. It is in charge of including
                                     the holes and cliffs in the costmap.
          vva_description - Contains the URDF model and the launch files for rviz and for publishing the TF of the URDF model.
          vva_gazebo - Gazebo worlds.
          vva_kinect_aux - In charge of changing the tilt of the Kinect and of publishing the position of the joints according with the
                           Kinect's IMU. Runs on the RbPi.
          vva_lidar_filter - LaserScans filter used to reduce the angle covered by the laserscan_kinect and hence remove the noise generated
                             by the Kinect in a small section of the field of view.
          vva_lidar_odom - LaserScan based odometry, uses the icp_odometry module of rtabmap.
          vva_msgs - Package that contains the defintions of the customized messages and services used in VVA.
          vva_navigation - Contains the modules related to the mapping, localization and navigation stack, including rtabmap and move_base.
          vva_nav_test - Manual control of the robot based on the keyboard arrow keys and also by scripting. It also contains a node to
                         generate customized statistics in tables based on selected topics.
          vva_raspi_consolidated - Consolidated launch file that starts in the RbPi: OpenNI Kinect, kinect_aux, rgbd_sync, base encoders publisher
                                   and base controller. Besides there is a launch file to start the RP-Lidar.
          vva_rplidar_ros - (New in v0.6) Fork of the rplidar_ros package. It is modified to add options that enable the change of the values of the
                            Lidar intensities. Runs on the RbPi.
          vva_topics_sync - Package that uses message_filters to synchronize the image_raw and camera_info topics generated by the Kinect.
          vva_user_intents - (New in v0.6) This package contains nodes in charge of the execution of actions identified during
                             the Intent Recognition. Besides is responsible for the high level behaviors of the robot.
          vva_voice_interact_laptop - (New in v0.6) This package contains the Speech Recognition module, which is based on DeepSpeech v0.6.1.
                                      It also performs the mapping between transcripts and intent services.
          vva_voice_interact_rbpi - (New in v0.6) This node is in charge of interfacing with the microphones array of the Kinect and perform the
                                    wake-word detection, which is based on Porcupine. Runs on the RbPi.
     
     - Known issues:
          - The model is not shown in Gazebo, when the joints are of type "revolute". For example, this happened with the joints between the
            Kinect and the base.
          - vva_cliff_detector doesn't work when simulated in Gazebo, it shows the error: "Could not perform stairs detection:
            Depth image has unsupported encoding: 32FC1"
          - Sometimes the depth image transmission has interruptions or it just doesn't arrive any more, hence the navigation stack
            stops working. The cause of this issue is unknown so far, maybe it is due to the weakness of the WiFi signal in some locations.
          - When voice commands are captured using the microphones array of the Kinect, the Speech Recognition doesn't work accurately. Maybe
            due to the environmental noise and because the microphones are far from the person speaking.
     
     - Commands to test it in simulation:
     
          # Deploy the model in Gazebo:
             roslaunch vva_gazebo vva_world.launch
          # Start rviz and the state_publishers:
             roslaunch vva_description rviz_rtabmap.launch simulation:=true
          # -------------------------------------------------------------------------------------------------------   
          # Mapping:
          # -------------------------------------------------------------------------------------------------------   
          # Start rtabmap:
             roslaunch vva_navigation vva_rtabmap.launch simulation:=true localization:=false
          # Use the teleop:
             roslaunch vva_nav_test vva_teleop2.launch
          # Move the robot to generate the map, turn 360° in different places,
          # In the Laptop: If you want to restart the map execute:
             rosservice call /rtabmap_laptop/reset
          # -------------------------------------------------------------------------------------------------------   
          # Once done, close rtabmap and the teleop.
          # Localization:
          # -------------------------------------------------------------------------------------------------------   
          # Start rtabmap:
             roslaunch vva_navigation vva_rtabmap.launch simulation:=true localization:=true
          # Start the modules: navigation (move_base), image-camera_info sync, laserscan_kinect, laserscan_kinect_filter and vva_navigation_correction:
             roslaunch vva_navigation vva_consolidated_nav.launch simulation:=true
          # Update the location coordinates in navigation_intent_params_simulation.yaml and start the navigation-intent node:
             roslaunch vva_user_intents vva_user_intents.launch simulation:=true
          # Start the Voice Interaction Speech Recognition module:
             roslaunch vva_voice_interact_laptop vva_voice_interact_laptop.launch
     
     - Commands to test it in the hardware:
     
          # In the RbPi: If it is required, turn off the USB ports: LIDAR and Kinect (This will disable all the ports):
             # Turn off USB ports:
               sudo su -
               echo '1-1' > /sys/bus/usb/drivers/usb/unbind
             # Turn on USB ports:
               sudo su -
               echo '1-1' > /sys/bus/usb/drivers/usb/bind
            # If it is required only to stop or start the RP-Lidar motor:
               rosservice call /stop_motor
               rosservice call /start_motor
          # In the RbPi: Check that NTP synchronization with the laptop is working (ntpq -p), if it is not execute:
             sudo systemctl stop ntp
             sudo ntpdate <Dir_IP_de_la_Laptop> # Execute several times
             sudo systemctl start ntp
             ntpq -p
          # In the RbPi: If it is required, calibrate the wheels motors:
             rm /home/ubuntu/ROS/VehiculoVigilanciaAutonomo_v0.6/VVA_ws/src/vva_base_controller/wheels_calibration/wheels_calibration.txt
             roslaunch vva_base_controller vva_wheels_calibration.launch
          # -------------------------------------------------------------------------------------------------------
          # In the RbPi: Start the module: rplidar (it was separated from the consolidated launch file because it failed to start scanning):
             roslaunch vva_raspi_consolidated vva_raspi_rplidar.launch
          # In the RbPi: Start the modules: kinect, kinect_aux, rgbd_sync, encoders publisher y base controller:
             roslaunch vva_raspi_consolidated vva_raspi_consolidated_rtabmap.launch
          # In the RbPi: (Optional) Start the Kinect microphone Voice Interaction wake word detection module:
             roslaunch vva_voice_interact_rbpi vva_voice_interact_rbpi.launch
          # -------------------------------------------------------------------------------------------------------
          # In the Laptop: Start rviz and the state_publishers:
             roslaunch vva_description rviz_rtabmap.launch simulation:=false
          # In the Laptop: Start the odometry based on laserscan:
             roslaunch vva_lidar_odom vva_lidar_odom.launch
          # -------------------------------------------------------------------------------------------------------   
          # Mapping:
          # -------------------------------------------------------------------------------------------------------   
          # In the Laptop: Start the modules: rgbd_relay, el rtabmap y el rtabmapviz, in mapping mode:
             roslaunch vva_navigation vva_rtabmap.launch simulation:=false localization:=false
          # In the Laptop: Use the teleop:
             roslaunch vva_nav_test vva_teleop2.launch
          # Move the robot to generate the map, turn 360° in different places,
          # In the Laptop: If you want to restart the map execute:
             rosservice call /rtabmap_laptop/reset
          # -------------------------------------------------------------------------------------------------------   
          # Once done, close rtabmap and the teleop.
          # Localization:
          # -------------------------------------------------------------------------------------------------------   
          # In the Laptop: Start the modules: rgbd_relay, rtabmap and rtabmapviz, in localization mode:
             roslaunch vva_navigation vva_rtabmap.launch simulation:=false localization:=true
          # In the Laptop: Start the modules: odometry correction, navigation (move_base), image-camera_info sync,
          # vva_cliff_detector, laserscan_kinect, laserscan_kinect_filter and vva_navigation_correction:
             roslaunch vva_navigation vva_consolidated_nav.launch simulation:=false
          # In the Laptop: Update the location coordinates in navigation_intent_params.yaml and launch the navigation-intent node:
             roslaunch vva_user_intents vva_user_intents.launch simulation:=false
          # In the Laptop: Start the Voice Interaction Speech Recognition module:
             roslaunch vva_voice_interact_laptop vva_voice_interact_laptop.launch
             




