**parrot\_anafi\_dev\_guide**

<p align="center">
  <iframe width="560" height="315" src="https://www.youtube.com/embed/3iV7Za-PGTo?autoplay=1" frameborder="0" allow="autoplay; encrypted-media" allowfullscreen></iframe>
</p>

---

## Sumário

1. [Descrição](#descrição)
2. [Downloads](#downloads)
3. [Pré-requisitos](#pré-requisitos)
4. [Build](#build)
5. [Ambiente de Simulação](#ambiente-de-simulação)
6. [Firmware](#firmware)
7. [Pacote de Drone Simulado](#pacote-de-drone-simulado)
8. [Controle de Drone Real](#controle-de-drone-real)
9. [Publicação de Pose](#publicação-de-pose)
10. [Joystick](#joystick)
11. [Navegação Autônoma](#navegação-autônoma)
12. [Localização](#localização)

---

## Descrição

Este repositório contém os pacotes ROS do Parrot Anafi modificados para eliminar funções que atrapalham a navegação do VANT. Inclui versões para simulação e controle em drones reais.

## Downloads

* **Pacotes personalizados (modificados):**

  * Navegação ROS Parrot Anafi (modificado):

    * [Drive Link 1](https://drive.google.com/drive/folders/1jUU4yFxlOtcuWu5GtcwgfWp2B4o7v7I4?usp=drive_link)
    * [Drive Link 2](https://drive.google.com/drive/folders/1pYquwr-zUXXykhDIMdIO-0kJU7EajOc0?usp=drive_link)

* **Pacotes originais:**

  * [anafi\_autonomy](https://github.com/andriyukr/anafi_autonomy)
  * [anafi\_ros](https://github.com/andriyukr/anafi_ros)

* **Vídeo de experimento:**

  * Ambiente interno e externo: [https://www.youtube.com/watch?v=3iV7Za-PGTo](https://www.youtube.com/watch?v=3iV7Za-PGTo)

## Pré-requisitos

* ROS 2 instalado (verifique compatibilidade com distribuição).
* parrot-sphinx (simulador UE4) instalado.
* Dependências Python para ROS nodes (ver `package.xml`).

## Build

```bash
# Build da pasta principal
colcon build --symlink-install
```

## Ambiente de Simulação

Inicie um dos ambientes UE4:

```bash
parrot-ue4-empty   # Mundo vazio
parrot-ue4-forest  # Floresta
parrot-ue4-office  # Escritório
```

## Firmware

Inicie o serviço de firmware e carregue a imagem do drone:

```bash
sudo systemctl start firmwared.service
sphinx "/opt/parrot-sphinx/usr/share/sphinx/drones/anafi4k.drone"::firmware="https://firmware.parrot.com/Versions/anafi/pc/%23latest/images/anafi-pc.ext2.zip"
```

## Pacote de Drone Simulado

```bash
ros2 launch anafi_autonomy anafi_autonomy_launch.py \
  ip:='10.202.0.1' model:='4k'
```

## Controle de Drone Real

* **Via controle USB**

```bash
ros2 launch anafi_autonomy control_anafi_launch.py \
  ip:='192.168.53.1' model:='4k'
```

* **Via Wi-Fi**

```bash
ros2 launch anafi_autonomy control_anafi_launch.py \
  ip:='192.168.42.1' model:='4k'
```

## Publicação de Pose

Configure o ambiente e publique a pose do simulador:

```bash
# Ajusta variáveis de ambiente para sphinx
. /opt/parrot-sphinx/usr/bin/parrot-sphinx-setenv.sh

# Executa nó de publicação de pose
ros2 run anafi_ros_nodes sphinx \
  --ros-args -r __ns:=/anafi -p drone_name:=anafi
```

## Joystick

```bash
# Inicializa teleop com configuração Xbox
ros2 launch teleop_twist_joy teleop-launch.py joy_config:='xbox'

# Executa nó de controle via joystick
ros2 run control_parrot control_parrot_joystick
```

## Navegação Autônoma

Comandos para execução de modos autônomos:

```bash
ros2 run control_parrot control_parrot_servovisual   # Servo Visual
ros2 run control_parrot control_parrot_indoor       # Indoor
ros2 run control_parrot link_analizer               # Análise de link
```

## Localização

Leitura de diferentes fontes de posicionamento:

```bash
ros2 run xyz_local_publish gps_read         # Leitura GPS
ros2 run xyz_local_publish optitrack_read   # Leitura OptiTrack
ros2 run xyz_local_publish pose_read        # Leitura de Pose
```
