#build da pasta principal
colcon build --symlink-install

#inicia ambiente de simulacao
parrot-ue4-empty
parrot-ue4-forest
parrot-ue4-office

#iniciar firmaware
sudo systemctl start firmwared.service
sphinx "/opt/parrot-sphinx/usr/share/sphinx/drones/anafi4k.drone"::firmware="https://firmware.parrot.com/Versions/anafi/pc/%23latest/images/anafi-pc.ext2.zip"

#pacote  drone simulado
ros2 launch anafi_autonomy anafi_autonomy_launch.py ip:='10.202.0.1' model:='4k'
#drone real pelo controle
ros2 launch anafi_autonomy control_anafi_launch.py ip:='192.168.53.1' model:='4k'
#drone real por wifi
ros2 launch anafi_autonomy control_anafi_launch.py ip:='192.168.42.1' model:='4k'

#pose do simulador (para o codigo que publica a pose)
. /opt/parrot-sphinx/usr/bin/parrot-sphinx-setenv.sh
ros2 run anafi_ros_nodes sphinx --ros-args -r __ns:=/anafi -p drone_name:=anafi

#iniciar joystick
ros2 launch teleop_twist_joy teleop-launch.py joy_config:='xbox'
ros2 run control_parrot control_parrot_joystick

#codigos para navegaçao automona
ros2 run control_parrot control_parrot_servovisual
ros2 run control_parrot control_parrot_indoor
ros2 run control_parrot link_analizer

#codigos para localização
ros2 run xyz_local_publish gps_read
ros2 run xyz_local_publish optitrack_read
ros2 run xyz_local_publish pose_read











#Liberar altitude e distancia
ros2 param set /anafi/anafi drone/max_altitude 1000.0
ros2 param set /anafi/anafi drone/max_distance 1000.0
#Tilt_max e vel_max
ros2 param set /anafi/anafi drone/max_horizontal_speed 15.0 #max_15 m/s
ros2 param set /anafi/anafi drone/max_vertical_speed 4.0 #max_4 m/s
ros2 param set /anafi/anafi drone/max_pitch_roll 40.0 #max_40 deg
#passar de controle manual para controle onboard
ros2 service call /anafi/skycontroller/offboard std_srvs/srv/SetBool "{data: True}"

#mover gimbal para baixo
ros2 topic pub /anafi/gimbal/command anafi_ros_interfaces/msg/GimbalCommand "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, mode: 0, frame: 1, roll: 0.0, pitch: 90, yaw: 0.0}" --once


#Começar a gravar
ros2 service call /anafi/camera/recording/start anafi_ros_interfaces/srv/Recording
#Parar de gravar
ros2 service call /anafi/camera/recording/stop anafi_ros_interfaces/srv/Recording

#Download
ros2 service call /anafi/storage/download std_srvs/srv/SetBool
#Formatar
ros2 service call /anafi/storage/format std_srvs/srv/Trigger







#comando que move o drone de verdade
ros2 topic pub /anafi/keyboard/command anafi_autonomy/msg/KeyboardCommand "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, drone_action: 0, drone_x: 0, drone_y: 0, drone_z: 0, drone_yaw: 1, gimbal_roll: 0, gimbal_pitch: 0, gimbal_yaw: 0, camera_action: 0, zoom: 0}" --times 1000 --rate 100

#pousar e levantar o drone
ros2 service call /anafi/drone/takeoff std_srvs/srv/Trigger
ros2 service call /anafi/drone/land std_srvs/srv/Trigger

#controle pelo teclado
ros2 run anafi_autonomy keyboard --ros-args -r __ns:=/anafi






#MEU PACK PARA SIMULACAO (transforma gps em um x,y,z que devera vir do aruco)
ros2 run xyz_local_publish gps_read 
ros2 run xyz_local_publish pose_read
ros2 topic echo /anafi/drone/position_global

#MEU PACK PARA CONTROLE DO ANAFI
ros2 run control_parrot control_parrot_angle2
ros2 run control_parrot control_parrot

