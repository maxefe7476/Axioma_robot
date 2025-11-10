# 🔄 Cinemática del Robot Axioma 4WD

> **NOTA**: Este documento presenta el modelo cinemático de differential drive (4WD skid-steer). Todos los parámetros son valores REALES extraídos de `model.sdf` y plugins de Gazebo.

## 1. Introducción

El robot Axioma utiliza una configuración de **4 ruedas motrices con cinemática diferencial** (skid-steer). Aunque tiene 4 ruedas, se controla como un differential drive tradicional:
- Las ruedas izquierdas (1 y 2) giran a la misma velocidad
- Las ruedas derechas (3 y 4) giran a la misma velocidad
- El control diferencial entre lados permite rotación

---

## 2. Geometría del Robot

### 2.1 Configuración de Ruedas

El robot tiene **4 ruedas** distribuidas en configuración rectangular:

**Posiciones en Frame {R}** (extraídas de `model.sdf`):

| Rueda | Nombre | Posición $(x, y, z)$ [m] | Joint |
|-------|--------|--------------------------|-------|
| 1 | Frontal Izquierda | (0.0644, 0.0738, 0.0407) | `base_to_wheel1` |
| 2 | Trasera Izquierda | (-0.0712, 0.0738, 0.0407) | `base_to_wheel2` |
| 3 | Trasera Derecha | (-0.0712, -0.0625, 0.0407) | `base_to_wheel3` |
| 4 | Frontal Derecha | (0.0644, -0.0625, 0.0407) | `base_to_wheel4` |

**Fuente**: `model.sdf` líneas 38, 115, 192, 269

### 2.2 Parámetros Geométricos

**✅ Valores REALES de `model.sdf`**:

```python
wheel_radius (r) = 0.0381 m        # líneas 72, 149, 226, 303
wheel_diameter   = 0.0762 m        # línea 443 (2 × radius)
wheel_separation = 0.1725 m        # línea 441
wheel_base       = 0.135644 m      # Calculado: |0.0644-(-0.0712)| m
wheel_width      = 0.03 m          # líneas 73, 150, 227, 304
```

### 2.3 Sistema de Coordenadas

```
        y (izquierda)
        ↑
        |     W1 ●━━━━━● W4
        |        |     |
        |        | [R] |  (robot frame)
        |        |     |
        |     W2 ●━━━━━● W3
        |
        └─────────────────→ x (adelante)

             θ (yaw, antihorario)
```

**Distancias**:
- Wheelbase (L): Distancia frontal-trasera = 0.1356 m
- Track width (W): Distancia izquierda-derecha = 0.1725 m

---

## 3. Modelo Cinemático Differential Drive

### 3.1 Velocidad del Robot

El vector de velocidad del robot en su propio frame $\\{R\\}$ es:

$$
\\mathbf{v}_R = \\begin{bmatrix} v \\\\ \\omega \\end{bmatrix}
$$

Donde:
- $v$: Velocidad lineal en el eje $x$ (adelante/atrás)
- $\\omega$: Velocidad angular alrededor del eje $z$ (rotación)

**NOTA**: Un differential drive NO tiene movilidad lateral ($v_y = 0$), solo puede moverse hacia adelante/atrás y rotar.

### 3.2 Velocidad de las Ruedas

Para un sistema de 4 ruedas agrupadas en 2 pares (izquierda/derecha):

**Par izquierdo** (ruedas 1 y 2):
$$
\\omega_L = \\omega_1 = \\omega_2
$$

**Par derecho** (ruedas 3 y 4):
$$
\\omega_R = \\omega_3 = \\omega_4
$$

Donde $\\omega_i$ es la velocidad angular de la rueda $i$ en $rad/s$.

---

## 4. Cinemática Directa

### 4.1 De Velocidades de Rueda a Velocidad del Robot

Dadas las velocidades angulares de las ruedas $\\omega_L$ y $\\omega_R$, la velocidad del robot se calcula como:

**Velocidad lineal**:
$$
v = \\frac{r(\\omega_R + \\omega_L)}{2}
$$

**Velocidad angular**:
$$
\\omega = \\frac{r(\\omega_R - \\omega_L)}{W}
$$

Donde:
- $r = 0.0381$ m (radio de rueda)
- $W = 0.1725$ m (separación entre ruedas)

### 4.2 Derivación

La velocidad lineal del robot es el promedio de las velocidades lineales de las ruedas:

$$
v = \\frac{v_R + v_L}{2} = \\frac{r\\omega_R + r\\omega_L}{2} = \\frac{r(\\omega_R + \\omega_L)}{2}
$$

La velocidad angular se obtiene de la diferencia de velocidades entre los lados:

$$
\\omega = \\frac{v_R - v_L}{W} = \\frac{r\\omega_R - r\\omega_L}{W} = \\frac{r(\\omega_R - \\omega_L)}{W}
$$

### 4.3 Forma Matricial

$$
\\begin{bmatrix} v \\\\ \\omega \\end{bmatrix} =
\\begin{bmatrix}
\\frac{r}{2} & \\frac{r}{2} \\\\
\\frac{r}{W} & -\\frac{r}{W}
\\end{bmatrix}
\\begin{bmatrix} \\omega_L \\\\ \\omega_R \\end{bmatrix}
$$

Con valores reales:
$$
\\begin{bmatrix} v \\\\ \\omega \\end{bmatrix} =
\\begin{bmatrix}
0.01905 & 0.01905 \\\\
0.2209 & -0.2209
\\end{bmatrix}
\\begin{bmatrix} \\omega_L \\\\ \\omega_R \\end{bmatrix}
$$

---

## 5. Cinemática Inversa

### 5.1 De Velocidad del Robot a Velocidades de Rueda

Dadas las velocidades deseadas del robot $(v, \\omega)$, las velocidades angulares de las ruedas se calculan como:

**Ruedas izquierdas**:
$$
\\omega_L = \\frac{v - \\omega \\cdot \\frac{W}{2}}{r}
$$

**Ruedas derechas**:
$$
\\omega_R = \\frac{v + \\omega \\cdot \\frac{W}{2}}{r}
$$

### 5.2 Derivación

Para que el robot se mueva con velocidad $v$ y rote con velocidad angular $\\omega$:

- La rueda izquierda debe moverse a: $v_L = v - \\omega \\cdot \\frac{W}{2}$
- La rueda derecha debe moverse a: $v_R = v + \\omega \\cdot \\frac{W}{2}$

Convirtiendo velocidades lineales a angulares:

$$
\\omega_L = \\frac{v_L}{r} = \\frac{v - \\omega \\cdot W/2}{r}
$$

$$
\\omega_R = \\frac{v_R}{r} = \\frac{v + \\omega \\cdot W/2}{r}
$$

### 5.3 Forma Matricial

$$
\\begin{bmatrix} \\omega_L \\\\ \\omega_R \\end{bmatrix} =
\\begin{bmatrix}
\\frac{1}{r} & -\\frac{W}{2r} \\\\
\\frac{1}{r} & \\frac{W}{2r}
\\end{bmatrix}
\\begin{bmatrix} v \\\\ \\omega \\end{bmatrix}
$$

Con valores reales:
$$
\\begin{bmatrix} \\omega_L \\\\ \\omega_R \\end{bmatrix} =
\\begin{bmatrix}
26.247 & -2.263 \\\\
26.247 & 2.263
\\end{bmatrix}
\\begin{bmatrix} v \\\\ \\omega \\end{bmatrix}
$$

---

## 6. Odometría

### 6.1 Integración de Pose

La pose del robot en el frame mundial $\\{W\\}$ evoluciona según:

$$
\\begin{bmatrix} \\dot{x} \\\\ \\dot{y} \\\\ \\dot{\\theta} \\end{bmatrix}_W =
\\begin{bmatrix}
v \\cos\\theta \\\\
v \\sin\\theta \\\\
\\omega
\\end{bmatrix}
$$

### 6.2 Integración Numérica (Euler)

Con período de muestreo $\\Delta t = 0.02$ s (50 Hz):

$$
\\begin{aligned}
x_{k+1} &= x_k + v \\cos\\theta_k \\cdot \\Delta t \\\\
y_{k+1} &= y_k + v \\sin\\theta_k \\cdot \\Delta t \\\\
\\theta_{k+1} &= \\theta_k + \\omega \\cdot \\Delta t
\\end{aligned}
$$

### 6.3 Implementación en Gazebo

**Plugin**: `libgazebo_ros_diff_drive.so` (`model.sdf:427-454`)

**Configuración**:
```xml
<update_rate>50</update_rate>
<num_wheel_pairs>2</num_wheel_pairs>
<publish_odom>true</publish_odom>
<publish_odom_tf>true</publish_odom_tf>
<odometry_frame>odom</odometry_frame>
<robot_base_frame>base_link</robot_base_frame>
```

**Fuente**: `model.sdf` líneas 439-451

---

## 7. Restricciones y Límites

### 7.1 Límites Cinemáticos

**✅ Valores REALES de `nav2_params.yaml`**:

$$
\\begin{aligned}
|v| &\\leq v_{max} = 0.26 \\text{ m/s} \\quad \\text{(línea 120)} \\\\
|\\omega| &\\leq \\omega_{max} = 1.0 \\text{ rad/s} \\quad \\text{(línea 122)} \\\\
|\\dot{v}| &\\leq a_{max} = 2.5 \\text{ m/s}^2 \\quad \\text{(línea 129)} \\\\
|\\dot{\\omega}| &\\leq \\alpha_{max} = 3.2 \\text{ rad/s}^2 \\quad \\text{(línea 130)}
\\end{aligned}
$$

### 7.2 Límites de Ruedas

De la cinemática inversa, las velocidades angulares máximas de las ruedas son:

**Movimiento recto** ($\\omega = 0$):
$$
\\omega_{wheel,max} = \\frac{v_{max}}{r} = \\frac{0.26}{0.0381} = 6.82 \\text{ rad/s}
$$

**Rotación pura** ($v = 0$):
$$
\\omega_{wheel,max} = \\frac{\\omega_{max} \\cdot W}{2r} = \\frac{1.0 \\times 0.1725}{2 \\times 0.0381} = 2.263 \\text{ rad/s}
$$

**Velocidad máxima combinada**:
$$
\\omega_{wheel,max} = \\frac{v_{max} + \\omega_{max} \\cdot W/2}{r} = \\frac{0.26 + 0.08625}{0.0381} = 9.09 \\text{ rad/s}
$$

### 7.3 Radio de Giro Mínimo

Para rotación en el lugar ($v = 0, \\omega \\neq 0$):
$$
R_{min} = 0 \\text{ m}
$$

El robot puede girar sobre su propio eje (giro de punto cero).

Para movimiento a velocidad máxima con giro máximo:
$$
R = \\frac{v_{max}}{\\omega_{max}} = \\frac{0.26}{1.0} = 0.26 \\text{ m}
$$

---

## 8. Implementación Computacional

### 8.1 Pseudocódigo: Cinemática Inversa

```python
def compute_wheel_velocities(v, omega):
    \"\"\"
    Calcula velocidades angulares de ruedas desde velocidad del robot

    Args:
        v: Velocidad lineal [m/s]
        omega: Velocidad angular [rad/s]

    Returns:
        omega_left, omega_right: Velocidades angulares [rad/s]
    \"\"\"
    r = 0.0381  # wheel_radius
    W = 0.1725  # wheel_separation

    # Saturar comandos
    v = clip(v, -0.26, 0.26)
    omega = clip(omega, -1.0, 1.0)

    # Cinemática inversa
    omega_left = (v - omega * W/2) / r
    omega_right = (v + omega * W/2) / r

    return omega_left, omega_right
```

### 8.2 Pseudocódigo: Cinemática Directa

```python
def compute_robot_velocity(omega_left, omega_right):
    \"\"\"
    Calcula velocidad del robot desde velocidades de ruedas

    Args:
        omega_left: Velocidad angular rueda izquierda [rad/s]
        omega_right: Velocidad angular rueda derecha [rad/s]

    Returns:
        v, omega: Velocidad lineal y angular del robot
    \"\"\"
    r = 0.0381  # wheel_radius
    W = 0.1725  # wheel_separation

    # Cinemática directa
    v = r * (omega_right + omega_left) / 2
    omega = r * (omega_right - omega_left) / W

    return v, omega
```

---

## 9. Referencias

**Implementación**:
- Plugin Gazebo: `libgazebo_ros_diff_drive.so` (`model.sdf:427-454`)
- Parámetros geométricos: `model.sdf:441-444`
- Límites de velocidad: `nav2_params.yaml:120-130`

**Documentación externa**:
- [Gazebo Diff Drive Plugin](http://gazebosim.org/tutorials?tut=ros2_installing&cat=connect_ros)
- [Differential Drive Kinematics](https://www.cs.columbia.edu/~allen/F17/NOTES/icckinematics.pdf)

---

**Autor**: Mario David Alvarez Vallejo
**Fecha**: 2025
**Versión**: 1.0.0
