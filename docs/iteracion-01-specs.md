# Iteración 1 — Simulación del robot en entorno digital

## Objetivo

Demostrar que el robot puede patrullar autónomamente un datacenter simulado y
recolectar lecturas de sensores en puntos de inspección definidos. Esta iteración
prueba el loop completo de navegación + adquisición de datos, sin integración con
InfluxDB ni modelos ML.

---

## Alcance

### Incluido

- Entorno Gazebo con plano simplificado del datacenter (racks como obstáculos estáticos)
- Robot diferencial como proxy del cuadrúpedo Spot
- Navegación autónoma con Nav2, siguiendo waypoints predefinidos por rack
- Sensores virtuales: lecturas sintéticas de temperatura, humedad y potencia en
  cada punto de inspección (generadas por el mismo módulo de `01_data_generator`)
- Outputs: `patrol_log.csv` + `patrol_video.mp4`

### Excluido (próximas iteraciones)

| Componente | Motivo |
|---|---|
| Modelo 3D del cuadrúpedo (Spot) | Complejidad de locomoción — usar robot diferencial como placeholder |
| Integración con InfluxDB / Grafana | Los datos quedan en archivo local; la integración va en iteración 2 |
| Detección de anomalías (ML) | Iteración 3 |
| Mapa dinámico (personas, obstáculos móviles) | Iteración 4+ |
| Múltiples robots | Fuera de MVP |

---

## Decisiones de diseño

### Robot model

Se usa un **robot diferencial simple** (estilo TurtleBot3) en lugar de un
cuadrúpedo como Spot. Razones:

- La simulación de locomoción cuadrúpeda requiere controladores de joints
  complejos que no aportan valor en esta iteración.
- El stack de navegación Nav2 funciona igual sobre cualquier base diferencial.
- Se puede intercambiar el URDF en iteración 2 sin cambiar la lógica de
  navegación ni los sensores.

### Simulador

**Gazebo Classic (Gazebo 11)** en lugar de Gazebo Sim (Ignition/Harmonic).
Razones:

- Mejor soporte en entornos headless/Ubuntu 22.04 sin GPU.
- Paquetes de ROS 2 Humble + Gazebo Classic son más estables en Colab.
- Menor consumo de RAM (~600 MB vs ~1.5 GB de Ignition con plugins completos).

### Visualización en Colab

Colab no tiene display físico. Approach estándar:

```
Xvfb :99 -screen 0 1280x720x24 &   # display virtual
export DISPLAY=:99
gzserver + gzclient → DISPLAY :99
ffmpeg -f x11grab ... patrol_video.mp4
```

La sesión de Colab es efímera — todos los outputs se copian a Google Drive al
final del notebook.

---

## Arquitectura

```
notebooks/04_robot_simulation.ipynb
│
├── [Setup]   Instalar ROS 2 Humble + Gazebo 11 + Nav2
├── [World]   Levantar datacenter.world (Gazebo headless)
├── [Robot]   Publicar sentinel_robot.urdf en /robot_description
├── [Nav]     Iniciar Nav2 stack + cargar waypoints.yaml
├── [Sensor]  Levantar virtual_sensor_node.py
├── [Patrol]  Ejecutar patrol_node.py → navega rack a rack
└── [Output]  Guardar patrol_log.csv + patrol_video.mp4 en Drive
```

### Nodos ROS 2

| Nodo | Responsabilidad |
|---|---|
| `patrol_node.py` | Envía waypoints al Nav2 action server en secuencia, espera confirmación antes de avanzar |
| `virtual_sensor_node.py` | Cuando el robot llega a un waypoint, invoca el generador sintético y publica las lecturas en `/sensor_readings` |
| `sensor_logger_node.py` | Suscribe a `/sensor_readings`, acumula y vuelca a `patrol_log.csv` |

---

## Estructura de archivos nueva

```
datacenter-sentinel/
├── sim/
│   ├── worlds/
│   │   └── datacenter.world          # Gazebo SDF con racks y pasillos
│   ├── urdf/
│   │   └── sentinel_robot.urdf       # Robot diferencial con LIDAR 2D
│   ├── config/
│   │   ├── waypoints.yaml            # Coords (x, y, yaw) por rack
│   │   ├── nav2_params.yaml          # Parámetros del stack Nav2
│   │   └── costmap_params.yaml
│   └── nodes/
│       ├── patrol_node.py
│       ├── virtual_sensor_node.py
│       └── sensor_logger_node.py
├── notebooks/
│   └── 04_robot_simulation.ipynb    # Orquestador principal (Colab)
└── docs/
    └── iteracion-01-specs.md         # Este archivo
```

---

## Mapa del datacenter (Gazebo world)

Layout minimalista basado en el `datacenter_layout.json` que se definirá:

```
+------------------------------------------+
|  ZONA A (Hot Aisle)                       |
|  [ RACK-01 ] [ RACK-02 ] [ RACK-03 ]     |
|  --------- pasillo 1.5m --------          |
|  [ RACK-04 ] [ RACK-05 ] [ RACK-06 ]     |
|  ZONA B (Cold Aisle)                      |
|                                           |
|  → Robot arranca en (0,0), ruta fija     |
+------------------------------------------+
```

Cada rack se modela como un box de `0.6m × 1.0m × 2.0m`. La sala es de
`~15m × 10m`. El robot navega por los pasillos entre filas.

---

## Schema del CSV de salida

```
patrol_log.csv
──────────────────────────────────────────────────────
timestamp, rack_id, zone, temperature_c, humidity_pct,
power_draw_kw, noise_db, arrival_time_s, anomaly_injected
──────────────────────────────────────────────────────
```

Una fila por visita a cada rack. `anomaly_injected` es booleano (para
validación en demos).

---

## Criterios de éxito

- [ ] El notebook corre de principio a fin en Colab (runtime CPU) sin intervención manual
- [ ] El robot navega de rack en rack sin colisiones en al menos una ronda completa
- [ ] Se genera `patrol_log.csv` con una fila por punto de inspección
- [ ] Se genera `patrol_video.mp4` mostrando el patrullaje en Gazebo
- [ ] El tiempo total de patrullaje (6 racks) es menor a 3 minutos de simulación

---

## Riesgos y mitigaciones

| Riesgo | Probabilidad | Mitigación |
|---|---|---|
| RAM insuficiente en Colab para Gazebo + Nav2 | Alta | Deshabilitar gzclient en headless-only mode; usar mapa 2D simple |
| Instalación de ROS 2 en Colab tarda > 10 min | Media | Imagen Docker pre-built en fallback; cache de apt |
| Nav2 falla en entorno headless sin GPU | Baja | Usar navegación más simple (pure-pursuit sobre waypoints) como fallback |
| Sesión de Colab expira a mitad del run | Media | Checkpoint: guardar en Drive al finalizar cada rack |

---

## Dependencias

- ROS 2 Humble
- Gazebo Classic 11
- Nav2 (`navigation2`)
- `robot_state_publisher`, `joint_state_publisher`
- Python: `rclpy`, `geometry_msgs`, `sensor_msgs`, `action_msgs`
- `ffmpeg` (grabación de video)
- `xvfb` (display virtual en Colab)

---

## Iteración 2 (preview)

Una vez validada la navegación básica, la siguiente iteración agrega:

- Integración con InfluxDB Cloud: `virtual_sensor_node.py` escribe directamente
  via `influxdb-client-python`
- Dashboard Grafana con posición del robot en tiempo real (panel de mapa)
- Reemplazo del URDF por modelo de cuadrúpedo simplificado
