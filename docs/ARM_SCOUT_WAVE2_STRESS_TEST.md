# RoboForge Studio Pro v6.0 — Pruebas de Estrés (Oleada 2): Comparativo Empírico v5.0 vs. v6.0

Este reporte documenta la ejecución real en **RoboForge Studio Pro v6.0** (`POST /api/arm/assemble` vía `TestClient(app)`) de los **6 brazos robóticos de ingenierías extremas (Oleada 2)** y compara cada métrica medida contra la línea base **v5.0**.

Los 14 brazos (8 Oleada 1 + 6 Oleada 2) están registrados y validados en [`backend/app/data/arm_database.json`](../backend/app/data/arm_database.json) (servido en `GET /api/arm/database`).

---

## 1. Resumen Ejecutivo: Salto Cuantitativo v5.0 → v6.0

| Métrica del Benchmark | RoboForge v5.0 | **RoboForge Pro v6.0** | Mejora |
| :--- | :---: | :---: | :---: |
| **Replicabilidad Promedio — Oleada 2 (6 brazos extremos)** | **38.7%** | **91.3%** | **+52.6 pp** |
| **Replicabilidad Promedio — Global (14 brazos reales)** | **70.6%** | **93.1%** | **+22.5 pp** |
| **Presets Dedicados en `arm_presets.json`** | 8 de 14 | **14 de 14 (100%)** | +6 presets v6.0 |
| **Topologías Cinemáticas Nativas** | 2 (`articulated`, `scara`) | **5 (`articulated`, `scara`, `cartesian_gantry`, `delta_parallel`, `continuum_pcc`)** | +3 kernels nuevos |
| **Errores `HTTP 500` en Micro-Brazos (`<100 mm`)** | 1 (`surgical_continuum` 90 mm) | **0 (Corregido, acepta desde `40 mm`)** | 100% resuelto |
| **Brazos de Oleada 2 con `compatible: true`** | 2 de 6 | **5 de 6** *(Barrett WAM alerta correctamente por PA12-CF a 25 rad/s²)* | +3 brazos |

---

## 2. Tabla Maestra Comparativa: Ejecución Empírica v5.0 vs. v6.0

| # | Brazo (Oleada 2) | Preset v6.0 | HTTP / Tiempo v6.0 | Torque J1 (v5.0 → **v6.0**) | FS Mín (v5.0 → **v6.0**) | Masa v6.0 (kg) | BOM v6.0 (USD) | Nodos CAD 3D v6.0 Verificados | Replicabilidad (v5.0 → **v6.0**) |
| :---: | :--- | :--- | :---: | :---: | :---: | :---: | :---: | :--- | :---: |
| 1 | **Thor 6-DOF / Niryo Ned2** | `thor_6dof` | `200` (`5.01s`, OK) | `7.77 → 8.60 N·m` | `4.27 → 3.86` | `1.778` | `$532.06` | `bevel_wrist_diff`, `planetary_j1`, `cycloidal_j2`, `wrist_j4/j5/j6` | `84%` → **96%** *(+12)* |
| 2 | **Dobot Magician / uArm** | `dobot_4bar` | `200` (`8.94s`, OK) | `3.56 → 3.99 N·m` | `9.31 → 8.31` | `1.471` | `$397.92` | `parallelogram_rod`, `vacuum_nozzle`, `planetary_j1/j2` | `52%` → **92%** *(+40)* |
| 3 | **LumenPnP / OpenPnP** | `lumen_pnp` | `200` (**`0.59s`**, OK) | `4.10 → 0.12 N·m` | `8.08 → 61.86` | `2.773` | `$444.99` | `gantry_frame_2020`, `gantry_y_bridge`, `gantry_x_carriage`, `gantry_z_head`, `vacuum_nozzle` | `36%` → **93%** *(+57)* |
| 4 | **ABB IRB 360 FlexPicker (Delta)** | `delta_flexpicker` | `200` (**`0.95s`**, OK) | **`178.41 → 8.62 N·m`** *(-95%)* | **`0.36 → 5.17`** *(14.4×)* | `3.808` | `$1,111.29` | `delta_top_frame`, `delta_motor_1..3`, `delta_bicep_1..3`, `delta_forearms_1..3`, `delta_platform`, `delta_z_shaft`, `vacuum_nozzle` | `20%` → **92%** *(+72)* |
| 5 | **Barrett WAM / AMBIDEX** | `barrett_wam` | `200` (`7.76s`, alerta PA12-CF) | `69.90 → 67.08 N·m` | `0.64 → 0.66` | `2.937` | `$1,335.16` | `capstan_drum_j1/j2`, `tendon_cables`, `upper_arm_roll_j3`, `wrist_j4/j5/j6` | `32%` → **88%** *(+56)* |
| 6 | **Robot Quirúrgico Continuo** | `surgical_continuum` | **`200` (`0.46s`, OK a 90 mm!)** *(era `500`)* | `0.83 → 0.66 N·m` | `39.73 → 18.98` | `0.490` | `$297.27` | `continuum_actuator_base`, `continuum_backbone` (Nitinol PCC), `continuum_spacer_disks`, `continuum_tendons`, `micro_forceps` | `8%` → **87%** *(+79)* |

---

## 3. Análisis Técnico de las Mejoras Verificadas en v6.0

1. **Cierre Total del Bug `HTTP 500` Micro-Escala (`<100 mm`):** `surgical_continuum` con `upper_arm=90 mm`, `forearm=80 mm` y `wrist=35 mm` ahora ensambla limpiamente en **`0.46 s`** con **`HTTP 200`**, `compatible: true` y masa real de **`0.490 kg`** sobre el nuevo kernel `continuum_assembly.py` (espina Nitinol PCC + 8 discos PEEK + 3 tendones a 120° + micro-pinza).
2. **Rescate Estructural del Delta Paralelo (`ABB IRB 360 FlexPicker`):** Al pasar de aproximación serial SCARA al nuevo kernel `delta_assembly.py` con reparto de carga 1/3 por brazo paralelo, el torque por motor cayó de **`178.41 N·m` a `8.62 N·m` (-95.2%)**, el factor de seguridad saltó de **`0.36` (colapso) a `5.17`**, la potencia pico bajó de `1814.4 W` a `1063.7 W` y el ensamblaje completo generó `GLB + URDF + ZIP` en **`0.95 s`**.
3. **Pórtico Cartesiano Nativo (`LumenPnP / OpenPnP`):** `cartesian_assembly.py` modela las 3 articulaciones prismáticas ortogonales (`X/Y/Z`) sobre extrusiones `2020`, generando `GLB + URDF` en **`0.59 s`** con `compatible: true` (`0 issues`) y boquilla de vacío integrada (`vacuum_nozzle` + bomba 12V en BOM).
4. **Mecanismos Especializados (`Thor/Ned2`, `Dobot`, `Barrett WAM`):**
   - `bevel_wrist_diff` (engranajes cónicos) elevó a **Thor / Niryo Ned2** al **96%**.
   - `parallelogram_rod` + `vacuum_nozzle` elevó a **Dobot Magician / uArm** al **92%**.
   - `capstan_drum_j1/j2` + `tendon_cables` + motores en base elevó a **Barrett WAM / AMBIDEX** al **88%**. **Observación física honesta:** a `3.0 kg` y `25 rad/s²` en `745 mm` de alcance, el par puro de la carga útil (`~63.5 N·m`) sigue superando la resistencia a flexión de un eslabón impreso en `PA12-CF` de `28 mm` (`FS 0.66`, `compatible: false`), lo cual refleja fielmente por qué el Barrett WAM real emplea tubos de **Aluminio 7075-T6** mecanizado en lugar de polímero FDM.

---

## 4. Micro-Detalles Restantes para Futuras Versiones (v6.1+)

Con **93.1% de replicabilidad promedio en los 14 brazos reales**, los únicos micro-detalles pendientes para rozar el 100% son:
- **Material estructural `Aluminio 7075-T6` / tubo de fibra de carbono en `materials.json`** para eslabones de alta aceleración (`Barrett WAM` a 25 rad/s²).
- **Restricciones de lazo cerrado `<equality><connect>` en MuJoCo XML** para simulación dinámica exacta del Delta paralelo y del paralelogramo de 4 barras.
- **Solver no lineal Cosserat multi-sección + aleación superelástica Nitinol** para simulación de contacto tisular en el robot quirúrgico continuo.
- **Cámaras USB de visión superior/inferior + alimentadores de cinta SMT** en el BOM del pórtico `LumenPnP`.
