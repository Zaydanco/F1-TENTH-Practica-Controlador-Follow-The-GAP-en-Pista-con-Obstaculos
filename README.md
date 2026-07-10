# F1-TENTH-Practica-Controlador-Follow-The-GAP-en-Pista-con-Obstaculos


## Descripción

Este proyecto implementa un controlador reactivo **Follow the Gap** para el simulador F1TENTH utilizando ROS 2 Humble.

La práctica se desarrolla sobre el circuito **BrandsHatch**, modificado para incluir obstáculos estáticos. También se incorpora un vehículo oponente autónomo que se desplaza a una velocidad menor que el vehículo principal.

El vehículo principal debe:

- Recorrer el circuito sin colisionar con las paredes.
- Detectar y evitar los obstáculos estáticos.
- Identificar al vehículo lento que circula delante.
- Seleccionar el costado con mayor espacio disponible.
- Preparar anticipadamente la maniobra.
- Adelantar al vehículo lento.
- Regresar al comportamiento normal de Follow the Gap.
- Registrar el número y tiempo de las vueltas completadas.

## Características principales

- Procesamiento y limpieza de datos LiDAR.
- Análisis de los 180 grados frontales.
- Burbuja de seguridad alrededor del obstáculo más cercano.
- Búsqueda del espacio libre continuo más amplio.
- Selección de un punto central dentro del gap.
- Suavizado del ángulo de dirección.
- Velocidad dinámica según el espacio y la curvatura.
- Detección de la posición relativa del oponente.
- Elección automática del lado de adelantamiento.
- Fase anticipada de preparación del rebase.
- Control de velocidad durante el adelantamiento.
- Contador y cronómetro de vueltas.

## Estructura del repositorio

```text
.
├── README.md
├── .gitignore
├── config/
│   └── sim.yaml
├── maps/
│   ├── BrandsHatch_map_obs.png
│   └── BrandsHatch_map_obs.yaml
└── gap_following/
    ├── package.xml
    ├── setup.py
    ├── setup.cfg
    ├── resource/
    │   └── gap_following
    ├── test/
    └── gap_following/
        ├── __init__.py
        ├── gap_follower.py
        └── slow_opponent.py
```

## Archivos principales

### `gap_follower.py`

Controlador del vehículo principal.

Se suscribe a:

```text
/scan
/ego_racecar/odom
/ego_racecar/opp_odom
```

Publica en:

```text
/drive
```

### `slow_opponent.py`

Controlador del vehículo oponente lento.

Se suscribe a:

```text
/opp_scan
```

Publica en:

```text
/opp_drive
```

## Tópicos utilizados

| Tópico | Tipo de mensaje | Función |
|---|---|---|
| `/scan` | `sensor_msgs/msg/LaserScan` | LiDAR del vehículo principal |
| `/drive` | `ackermann_msgs/msg/AckermannDriveStamped` | Control del vehículo principal |
| `/ego_racecar/odom` | `nav_msgs/msg/Odometry` | Odometría del vehículo principal |
| `/ego_racecar/opp_odom` | `nav_msgs/msg/Odometry` | Posición del oponente observada por el vehículo principal |
| `/opp_scan` | `sensor_msgs/msg/LaserScan` | LiDAR del vehículo oponente |
| `/opp_drive` | `ackermann_msgs/msg/AckermannDriveStamped` | Control del vehículo oponente |
| `/opp_racecar/odom` | `nav_msgs/msg/Odometry` | Odometría del vehículo oponente |

## Funcionamiento de Follow the Gap

El controlador realiza los siguientes pasos:

1. Recibe los datos del LiDAR.
2. Reemplaza valores inválidos y limita las distancias máximas.
3. Suaviza las lecturas para reducir variaciones.
4. Selecciona la región frontal del vehículo.
5. Detecta el obstáculo más cercano.
6. Genera una burbuja de seguridad.
7. Encuentra el gap libre más grande.
8. Selecciona un punto dentro del centro del gap.
9. Convierte el punto seleccionado en un ángulo de dirección.
10. Calcula una velocidad según la curvatura y el espacio disponible.
11. Publica el comando Ackermann en `/drive`.

## Lógica de adelantamiento

El vehículo principal obtiene la posición global del oponente mediante:

```text
/ego_racecar/opp_odom
```

La posición se transforma al sistema de referencia local del vehículo principal para calcular:

- Distancia frontal al oponente.
- Posición lateral del oponente.
- Velocidad del oponente.

Cuando el vehículo lento se encuentra delante, el controlador:

1. Mide el espacio disponible a la izquierda y a la derecha.
2. Selecciona el costado con mayor espacio libre.
3. Inicia una fase anticipada de preparación.
4. Se desplaza progresivamente hacia el costado elegido.
5. Mantiene una velocidad superior a la del oponente.
6. Finaliza el adelantamiento cuando el oponente queda detrás.
7. Regresa al funcionamiento normal de Follow the Gap.

## Velocidades de referencia

En una recta despejada, el vehículo principal puede alcanzar aproximadamente:

```text
5.0 m/s
```

El vehículo oponente está limitado aproximadamente a:

```text
2.0 m/s
```

Durante el adelantamiento, el vehículo principal utiliza límites específicos para conservar una diferencia de velocidad suficiente sin hacer la maniobra demasiado brusca.

## Velocidad angular

La velocidad angular del vehículo no posee un valor único fijo a lo largo del recorrido, esto puesto que se ve afectado por los cambios en la velocidad lineal, el ángulo de dirección, los cambios en la pista como por ejemplo curvas abiertas y cerradas y por último las maniobras que se realicen para evadir obstáculos o adelantar al auto más lento.

No obstante, podemos obtener esta información del componente:
```python
odom_msg.twist.twist.angular.z
```

Representada en radianes por segundo (`rad/s`) nos indica dependiendo de su valor:

 - Si es un valor cercano a cero, el vehículo se desplaza prácticamente en línea recta.
 - Si refleja un valor mayor el giro es más pronunciado.
 - El signo positivo o negativo permite distinguir el sentido del giro.
 
Debido a que el vehículo utiliza un modelo de dirección Ackermann, el
controlador publica en `/drive` la velocidad lineal y el ángulo de dirección.
La velocidad angular resultante no se establece como una constante, sino que
se obtiene dinámicamente mediante el tópico de odometría:

```text
/ego_racecar/odom
```

Por lo tanto, no se puede asignar una sola velocidad angular para toda la
simulación, pero sí se puede medir su valor instantáneo durante cada momento
del recorrido.

## Requisitos

- Ubuntu 22.04.
- ROS 2 Humble.
- Python 3.
- NumPy.
- Simulador F1TENTH Gym ROS instalado.
- Mapa BrandsHatch.
- Paquetes ROS:

```text
rclpy
sensor_msgs
nav_msgs
ackermann_msgs
```

## Instalación

Clonar el repositorio:

```bash
git clone https://github.com/Zaydanco/F1-TENTH-Practica-Controlador-Follow-The-GAP-en-Pista-con-Obstaculos.git
```

Copiar el paquete al workspace del simulador:

```bash
cp -r \
F1-TENTH-Practica-Controlador-Follow-The-GAP-en-Pista-con-Obstaculos/gap_following \
~/F1Tenth-Repository/src/
```

Copiar el mapa modificado:

```bash
cp \
F1-TENTH-Practica-Controlador-Follow-The-GAP-en-Pista-con-Obstaculos/maps/BrandsHatch_map_obs.* \
~/F1Tenth-Repository/src/f1tenth_gym_ros/maps/
```

Crear una copia de seguridad del archivo de configuración actual:

```bash
cp \
~/F1Tenth-Repository/src/f1tenth_gym_ros/config/sim.yaml \
~/F1Tenth-Repository/src/f1tenth_gym_ros/config/sim_backup.yaml
```

Copiar la configuración utilizada en esta práctica:

```bash
cp \
F1-TENTH-Practica-Controlador-Follow-The-GAP-en-Pista-con-Obstaculos/config/sim.yaml \
~/F1Tenth-Repository/src/f1tenth_gym_ros/config/sim.yaml
```

Compilar:

```bash
cd ~/F1Tenth-Repository

source /opt/ros/humble/setup.bash

rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install

source install/setup.bash
```

## Ejecución

### Terminal 1: iniciar el simulador

```bash
cd ~/F1Tenth-Repository

source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

### Terminal 2: iniciar el vehículo oponente

```bash
cd ~/F1Tenth-Repository

source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run gap_following slow_opponent
```

### Terminal 3: iniciar el vehículo principal

```bash
cd ~/F1Tenth-Repository

source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run gap_following gap_following
```

## Contador de vueltas

El controlador registra:

- Número de vueltas completadas.
- Tiempo de cada vuelta.
- Mejor tiempo registrado.

Ejemplo:

```text
Vuelta 1 completada | Tiempo: 95.42 s | Mejor vuelta: 95.42 s
Vuelta 2 completada | Tiempo: 93.80 s | Mejor vuelta: 93.80 s
```

Al detener el nodo con `Ctrl + C`, se muestra un resumen final.

## Videos de demostración

Agregar aquí los enlaces correspondientes a:

- Prueba del circuito con obstáculos:           https://www.youtube.com/watch?v=NLCyREkMed0
- Prueba con vehículo lento y adelantamiento:   https://www.youtube.com/watch?v=fT2BSfMI8qA

## Limitaciones actuales

- La versión incluida utiliza un vehículo principal y un vehículo oponente.
- La estabilidad del adelantamiento depende del espacio disponible y de la posición de los obstáculos.
- Los parámetros fueron ajustados específicamente para BrandsHatch.
- Agregar un segundo vehículo oponente requiere ampliar la configuración o el puente del simulador.

## Conclusión

El controlador combina el algoritmo reactivo Follow the Gap con información de odometría del vehículo oponente.

La solución permite que el vehículo principal recorra BrandsHatch, evite obstáculos estáticos y realice maniobras de adelantamiento sobre un vehículo más lento, manteniendo el control de velocidad, dirección y conteo de vueltas.

