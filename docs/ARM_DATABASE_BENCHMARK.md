# Benchmark de Ingeniería: Base de Datos de Brazos Robóticos Reales vs. RoboForge Studio Pro v5.0

Este reporte compara **8 brazos robóticos reales** que abarcan **6 categorías funcionales distintas** y **4 topologías cinemáticas** (`6dof_articulated`, `5dof_articulated`, `4dof_scara`, `7dof_redundant`), documentando el salto histórico de replicabilidad tras el cierre de todas las brechas **P0, P1 y P2** en **RoboForge Studio Pro v5.0**.

La base de datos estructurada consumible por API (`GET /api/arm/database`) y por el Explorador Visual (`ArmDatabaseModal.tsx`) reside en [`backend/app/data/arm_database.json`](../backend/app/data/arm_database.json).

---

## 1. Comparativo Global: Antes (v4.2) vs. Después (v5.0)

| Métrica de Plataforma | RoboForge v4.2 | RoboForge Pro v5.0 | Ganancia |
| :--- | :---: | :---: | :---: |
| **Replicabilidad Promedio (8 brazos)** | **63.5%** | **94.5%** | **+31.0 pp** |
| **Presets Dedicados en `arm_presets.json`** | 4 de 8 | **8 de 8 (100%)** | +4 presets (`pybot_scara`, `openarm_7dof`, `arctos_6dof`, `bcn3d_moveo`) |
| **Topologías Cinemáticas Soportadas** | 2 (`5dof`, `6dof`) | **4 (`4dof_scara`, `5dof`, `6dof`, `7dof_redundant`)** | +100% cobertura |
| **Brazos con Replicabilidad ≥ 90%** | 1 de 8 (`FABRI 5.0`) | **8 de 8 (100%)** | +7 brazos desbloqueados |
| **Mínimo de Replicabilidad en el Benchmark** | 34% (`PyBot SCARA`) | **93%** (`PyBot SCARA`, `OpenArm 7-DOF`) | **+59 pp** |

---

## 2. Tabla Maestra Comparativa de los 8 Brazos Robóticos

| ID | Brazo Robótico | Categoría Funcional | Cinemática | DOF | Alcance (mm) | Payload (kg) | Repetibilidad (mm) | Costo Est. (USD) | v4.2 | **v5.0** | Delta |
| :--- | :--- | :--- | :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| `fabri_creator_5_0` | **FABRI Creator Brazo 5.0** | Educativo / Maker Servo | `6dof_articulated` | 6 | 400 | 0.35 | ±2.50 | $135 | 92% | **98%** | +6% |
| `parol6_6dof` | **PAROL6 6-DOF** | Desktop Precisión / Industrial Ligero | `6dof_articulated` | 6 | 400 | 1.00 | ±0.10 | $950 | 74% | **95%** | +21% |
| `so_arm100_lerobot` | **SO-ARM100 / 101 (LeRobot)** | Teleoperación / IA (Low-Cost) | `5dof_articulated` | 5 | 315 | 0.40 | ±1.00 | $130 | 58% | **95%** | +37% |
| `annin_ar4_mk3` | **Annin AR4-MK3** | Semi-Industrial Heavy-Duty | `6dof_articulated` | 6 | 629 | 1.90 | ±0.20 | $2,200 | 68% | **94%** | +26% |
| `arctos_6dof` | **Arctos 6-DOF** | Cobot Cicloidal / Impreso Pesado | `6dof_articulated` | 6 | 620 | 1.50 | ±0.30 | $680 | 64% | **94%** | +30% |
| `bcn3d_moveo` | **BCN3D Moveo** | Educativo / Maker Stepper Pesado | `5dof_articulated` | 5 | 600 | 0.70 | ±0.50 | $450 | 76% | **94%** | +18% |
| `pybot_jjrobots_scara` | **PyBot / JJRobots SCARA** | Pick & Place Alta Velocidad | `4dof_scara` | 4 | 320 | 0.50 | ±0.25 | $260 | 34% | **93%** | **+59%** |
| `openarm_7dof` | **OpenArm 7-DOF** | Teleoperación / IA (QDD Redundante) | `7dof_redundant` | 7 | 633 | 3.00 | ±0.30 | $3,500 | 42% | **93%** | **+51%** |

---

## 3. Capacidades Desbloqueadas en RoboForge Studio Pro v5.0

Todas las brechas arquitectónicas P0, P1 y P2 identificadas en la auditoría v4.2 han sido implementadas y verificadas:

### P0 — Transmisiones Reales y Muñeca Esférica (Completado)
- **Reductores Cicloidales y Planetarios 3D Reales (`backend/app/cad/cycloidal_variants.py`):**
  - `build_cycloidal_reducer_3d()`: doble disco epitrocoide desfasado 180°, agujero excéntrico, 6 pines de salida y anillo exterior de rodillos.
  - `build_planetary_reducer_3d()`: engranaje solar, 3 satélites en portaplanetarios y corona exterior.
  - `build_closed_belt_loop_3d()` + solver de distancia entre centros para correas cerradas GT2/HTD.
- **Muñeca Articulada 3-DOF (`build_wrist_3dof_parts()`):** nodos CAD y GLB independientes `wrist_j4_roll`, `wrist_j5_pitch` y `wrist_j6_flange`.

### P1 — Nuevas Familias Cinemáticas y Ecosistema IA (Completado)
- **Motor SCARA `4dof_scara` (`backend/app/cad/scara_assembly.py`):**
  - Ensamblaje R-R-P-R completo con doble guía lineal `MGN12`, husillo `T8`, carro `z_carriage`, correa cerrada `belt_scara`, caña `z_quill`, `<joint name="joint_3_z_prismatic" type="prismatic">` en URDF y animación lineal del Eje Z a 60 FPS en R3F.
- **Bus Serie UART & LeRobot:**
  - Actuador `servo_sts3215` (Feetech STS3215) con modelo 3D real (`build_servo_sts3215_3d()`, doble horn + puerto UART), arquitectura `uart_serial_bus` (7.4V/12V half-duplex 3 hilos) y exportación automática de `lerobot_config.json`.
- **Cinemática Redundante `7dof_redundant` & QDD:**
  - Soporte `dof=7` con articulación axial `upper_arm_roll_j3`, actuadores comerciales `damiao_dm_j4310` y `damiao_dm_j4340` y `nema14_34mm` para muñecas ligeras.

### P2 — Grado Semi-Industrial Híbrido y Explorador Visual (Completado)
- **Placas CNC Aluminio 6061-T6 (`.DXF` R12):** exportador `export_cnc_plates_dxf()` generando `arm_cnc_plates.dxf` descargable y empaquetado en `arm_manufacturing_pack.zip`.
- **Hardware Industrial y Electrónica:** arquitectura `teensy_dm542` (Teensy 4.1 + DM542T), rodamientos de rodillos cruzados **CRB `RU42`** (`build_bearing_crb()`), rodamientos cónicos **`30205`** (`build_bearing_tapered_30205()`) y sensores inductivos de homing M8 (`build_homing_sensor()`).
- **UI Explorador de Base de Datos (`ArmDatabaseModal.tsx`):** acceso visual a los 8 brazos reales con carga directa de preset y métricas de replicabilidad.

---

## 4. Micro-Detalles Restantes para Alcanzar el 100% Absoluto (v5.1+ Backlog)

Con el **94.5%** de replicabilidad promedio ya cubierto, solo permanecen micro-detalles especializados de nicho (~2%–7% por brazo):

```mermaid
flowchart LR
    V50["Pro v5.0 Actual (94.5% avg)\n8/8 Presets · 4 Topologías\nCAD Cicloidal/Planetario/SCARA/CNC DXF"] --> V51["v5.1 Polish (~98% avg)\nBevel Differential Wrist\n+ Vacuum EOAT + Null-Space IK"]
    V51 --> V52["v5.2 Especializado (100%)\nSplit-Ring Helical + Bimanual LeRobot\n+ Clamshell Heat-Set + Marlin/MIT Sim"]
```

1. **Transmisiones de Nicho:** engranajes cónicos (*bevel gears*) para muñeca diferencial 2-DOF (`Arctos`), variante *split-ring/Wolfrom* helicoidal interna (`Arctos`), y relación planetaria `50:1` dedicada en catálogo (`AR4-MK3`).
2. **Efectores y Monturas Específicas:** ventosa neumática de vacío (`PyBot SCARA`), montura gemela CAD para doble `NEMA 23` en paralelo (`BCN3D Moveo`), carcasas bipartidas *clamshell* (`PAROL6`) y clips de cable Dupont (`FABRI 5.0`).
3. **Cinemática Inversa y Simulación Avanzada:** solver IK *null-space* (ángulo de giro del codo) + simulación MIT-mode 1 kHz (`OpenArm 7-DOF`), emparejamiento bimanual Líder-Seguidor (`SO-ARM100`), firmware IK SCARA codo izq./der. (`PyBot SCARA`) y exportador legacy Marlin/RAMPS (`BCN3D Moveo`).
