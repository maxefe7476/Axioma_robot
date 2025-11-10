# 📐 Modelo Matemático del Robot Axioma 4WD

## Descripción General

Este documento presenta el modelo matemático completo del robot autónomo Axioma, una plataforma móvil de **4 ruedas motrices con configuración skid-steer** (differential drive de 4 ruedas). El modelo abarca cinemática, control y parámetros físicos del sistema.

> **NOTA IMPORTANTE**: Todos los parámetros en esta documentación son valores REALES extraídos directamente del código fuente (`model.sdf`, `nav2_params.yaml`, plugins de Gazebo).

---

## 📑 Contenido

1. **[Cinemática](./cinematica.md)**
   - Modelo cinemático de differential drive
   - Cinemática directa e inversa
   - Odometría

2. **[Control](./control.md)**
   - Plugin Gazebo diff_drive
   - Límites de velocidad y aceleración
   - Sistema de navegación Nav2

3. **[Parámetros](./parametros.md)**
   - Parámetros geométricos
   - Características dinámicas
   - Sensores (LiDAR)

4. **[Diagrama Excalidraw](./modelo-axioma.excalidraw)**
   - Representación visual del modelo
   - Ecuaciones fundamentales
   - Parámetros clave

---

## Notación Matemática

### Sistemas de Coordenadas

| Símbolo | Descripción | Frame ROS2 |
|---------|-------------|------------|
| $\\{W\\}$ | Sistema de coordenadas mundial | `map` / `odom` |
| $\\{R\\}$ | Sistema de coordenadas del robot | `base_link` |

### Variables de Estado

| Variable | Descripción | Unidad |
|----------|-------------|--------|
| $q = [x, y, \\theta]^T$ | Pose del robot en $\\{W\\}$ | $[m, m, rad]$ |
| $\\dot{q} = [v, \\omega]^T$ | Velocidad del robot | $[m/s, rad/s]$ |
| $v_L, v_R$ | Velocidad lineal de ruedas izquierda/derecha | $m/s$ |
| $\\omega_L, \\omega_R$ | Velocidad angular de ruedas izquierda/derecha | $rad/s$ |

---

## Ecuaciones Fundamentales

### Cinemática Diferencial

Para un robot differential drive de 4 ruedas:

$$
v = \\frac{v_R + v_L}{2} = \\frac{r(\\omega_R + \\omega_L)}{2}
$$

$$
\\omega = \\frac{v_R - v_L}{W} = \\frac{r(\\omega_R - \\omega_L)}{W}
$$

Donde:
- $r = 0.0381$ m (radio de rueda)
- $W = 0.1725$ m (separación entre ruedas)

### Cinemática Inversa

$$
\\omega_L = \\frac{v - \\omega \\cdot W/2}{r}
$$

$$
\\omega_R = \\frac{v + \\omega \\cdot W/2}{r}
$$

---

## Parámetros del Robot Axioma

### Geometría

| Parámetro | Símbolo | Valor Real | Fuente |
|-----------|---------|------------|--------|
| Radio de rueda | $r$ | 0.0381 m | `model.sdf:72` |
| Separación de ruedas | $W$ | 0.1725 m | `model.sdf:441` |
| Distancia entre ejes | $L$ | 0.1356 m | Calculado de posiciones |
| Masa total | $m$ | 5.525 kg | Suma de masas en SDF |

### Límites Operacionales

| Parámetro | Valor | Fuente |
|-----------|-------|--------|
| Velocidad lineal máxima | 0.26 m/s | `nav2_params.yaml:120` |
| Velocidad angular máxima | 1.0 rad/s | `nav2_params.yaml:122` |
| Aceleración lineal máxima | 2.5 m/s² | `nav2_params.yaml:129` |
| Aceleración angular máxima | 3.2 rad/s² | `nav2_params.yaml:130` |
| Torque máximo por rueda | 20 N·m | `model.sdf:446` |

---

## Estructura del Modelo

```
Sistema 4WD Skid-Steer
│
├─ Cinemática Directa: (ω_L, ω_R) → (v, ω)
│
├─ Cinemática Inversa: (v, ω) → (ω_L, ω_R)
│
├─ Control Gazebo: Plugin libgazebo_ros_diff_drive.so
│  ├─ Input: /cmd_vel (Twist)
│  └─ Output: /odom (Odometry), TF (odom → base_link)
│
└─ Navegación: Nav2 Stack
   ├─ AMCL (Localización)
   ├─ DWB Local Planner
   └─ NavFn Global Planner
```

---

## Implementación

- **Plugin de Control**: `libgazebo_ros_diff_drive.so` (`model.sdf:427-454`)
- **Frecuencia de actualización**: 50 Hz
- **Odometría**: Publicada en `/odom` con TF
- **Stack de Navegación**: Nav2 Humble

---

## Convenciones

1. **Sistema de coordenadas**: Derecha (x adelante, y izquierda, z arriba)
2. **Ángulos positivos**: Sentido antihorario
3. **Velocidades**: Expresadas en el frame del robot $\\{R\\}$
4. **Configuración**: 4 ruedas (2 pares), sin dirección independiente

---

**Autor**: Mario David Alvarez Vallejo
**Proyecto**: Robot Axioma - Logística Industrial Autónoma
**Fecha**: 2025
**Versión**: 1.0.0
