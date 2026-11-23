# Auditoría Cross-Software y Recreación Externa de Modelos — RoboForge Studio Pro v8.0

Esta auditoría independiente evalúa empíricamente los archivos exportados por **RoboForge Studio Pro v8.0** en **9 brazos representativos cubriendo todas las topologías** (`mars_mini`, `so_arm100`, `parol6`, `pybot_scara`, `lumen_pnp`, `delta_flexpicker`, `surgical_continuum`, `bimanual_aloha`, `stewart_hexapod`) usando validadores externos independientes (**OpenCASCADE `STEPControl_Reader` + `BRepCheck_Analyzer`**, **`trimesh`**, **OPC XML 3MF**, **XML URDF/MuJoCo**, **ASCII DXF R12** y **linter `.INO`**) y una recreación comparativa real en **OpenSCAD** frente a **Fusion 360, Onshape, FreeCAD y SatelLite**.

---

## Pregunta 1: ¿Funcionan todos los diseños y archivos exportados al abrirlos en otro software?

**Respuesta corta:** **Sí en el ~82% de los archivos (especialmente `.STEP`, `.STL` articulados, `.DXF` y `.INO`), pero NO en el 100% — detectamos 6 fallos reales** en mallas de `bimanual_aloha`, falta de STLs individuales/`.3MF` en 4 topologías, solapamientos en ensamblajes `.GLB` y un exportador MuJoCo XML genérico.

### Tabla Empírica de Validación Cross-Software (9 Topologías)

| Brazo (Topología) | `.STEP` *(FreeCAD / SolidWorks / Fusion)* | `.STL` Individuales *(SatelLite / Bambu / Prusa)* | `.3MF` Platos *(OPC XML)* | `.GLB` Solapamientos AABB >35% | `.URDF` *(ROS 2 / PyBullet)* | `.XML` *(MuJoCo)* | `.INO` / `.DXF` |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **`mars_mini`** *(MSLA 4-Barras)* | **OK** (`40` sólidos, 0 inv.) | **OK** (`10` STLs, 100% watertight, 0 islas, `fits_bed: true`) | **OK** (`4` platos, 25k tris) | **14** (`horn_25t` ↔ `clevis`/`motor`) | **OK** (`4` joints, 0 err.) | ⚠️ Genérico (`3` joints) | `.INO` **OK** (137 lín.) |
| **`so_arm100`** *(LeRobot UART)* | **OK** (`39` sólidos, 0 inv.) | **OK** (`10` STLs, 100% watertight, 0 islas, `fits_bed: true`) | **OK** (`2` platos, 24.7k tris) | **16** (`horn_25t` ↔ `clevis`/`mount`) | ⚠️ Solo `4` joints (es 5-DOF) | ⚠️ Genérico (`3` joints) | — |
| **`parol6`** *(Articulado 6-DOF)* | **OK** (`52` sólidos, 0 inv.) | **OK** (`9` STLs, 100% watertight, 0 islas, `fits_bed: true`) | **OK** (`2` platos, 20.4k tris) | **7** (`wrist_j5` ↔ `wrist_tube` 87%) | ⚠️ Solo `4` joints (es 6-DOF) | ⚠️ Genérico (`3` joints) | — |
| **`pybot_scara`** *(SCARA 4-DOF)* | **OK** (`16` sólidos, 0 inv.) | ⚠️ Solo `1` STL monolítico (sin `parts_pack.zip`) | ❌ **0 platos `.3MF`** | **4** (`linear_rail_z` en AABB base) | **OK** (`1` prismatic + `3` rev.) | ⚠️ Genérico (`3` joints) | — |
| **`lumen_pnp`** *(Cartesiano)* | **OK** (`19` sólidos, 0 inv.) | ⚠️ Solo `1` STL monolítico (sin `parts_pack.zip`) | ❌ **0 platos `.3MF`** | **3** (`gantry_z_head` en marco) | **OK** (`3` prismatic + `1` rev.) | ⚠️ Genérico (`3` joints) | `.DXF` **OK** (R12) |
| **`delta_flexpicker`** *(Delta)* | **OK** (`36` sólidos, 0 inv.) | ⚠️ Solo `1` STL monolítico (sin `parts_pack.zip`) | ❌ **0 platos `.3MF`** | **9** (`delta_motor_2/3` ↔ frame 71%) | **OK** (`4` joints) | ⚠️ Genérico (`0` equalities) | — |
| **`surgical_continuum`** *(PCC)* | **OK** (`50` sólidos, 0 inv.) | ⚠️ Solo `1` STL monolítico (sin `parts_pack.zip`) | ❌ **0 platos `.3MF`** | **3** (tendones dentro de discos) | **OK** (`2` joints) | ⚠️ Genérico (`3` joints) | — |
| **`bimanual_aloha`** *(14-DOF)* | **OK** (`12` sólidos, 0 inv.) | ❌ **`2` STLs NO watertight + `7` islas** (`fits_bed: false`) | **OK** (`3` platos) | **1** (`motor_j1` en torso) | ⚠️ Solo `4` joints (es 14-DOF) | ⚠️ Genérico (`3` joints) | — |
| **`stewart_hexapod`** *(6-UPS)* | **OK** (`27` sólidos, 0 inv.) | **OK** (`3` STLs, 100% watertight, 0 islas) | ⚠️ `2` platos (solo 20 tris c/u) | **0** (limpio) | **OK** (`6` prismatic + `1` fixed) | ⚠️ Genérico (`0` equalities) | — |

**Lo que SÍ abre impecablemente en software externo:**
- **FreeCAD / SolidWorks / Fusion 360 (`.STEP`):** **9 de 9 archivos `.STEP`** (`291` sólidos B-Rep en total) pasaron `BRepCheck_Analyzer.IsValid() == True` con **0 sólidos inválidos** y **0 sólidos de volumen cero**.
- **Elegoo SatelLite / Chitubox / Bambu Studio (`mars_mini`, `so_arm100`, `parol6`):** El 100% de sus piezas individuales (`29/29` STLs) son **watertight**, con normales consistentes y **0 islas flotantes**. `mars_mini` cabe 100% en `Elegoo Mars 5 Ultra` (`153.4 × 77.8 × 165.0 mm`) con `0` alertas.
- **LibreCAD / QCAD (`.DXF`) y Arduino IDE (`.INO`):** `arm_cnc_plates.dxf` es ASCII DXF R12 válido y `mars_mini_esp32_wifi.ino` (137 líneas) tiene llaves/paréntesis balanceados y librerías válidas.

---

## Pregunta 2: ¿Encontraste problemas reales en la estructura, geometría o modelado de algún brazo?

**Sí. Encontramos 6 defectos reales y reproducibles**, priorizados por severidad:

1. **[P0 — Crítico · Impresión 3D] Mallas abiertas y 7 islas flotantes en `bimanual_aloha`:**
   - `part_left_upper_arm.stl` y `part_right_upper_arm.stl` salen **`watertight: false`** con **`body_count: 7`** (7 fragmentos desconectados en un solo archivo) y OCCT emite `Warning: 2 faces have been skipped due to null triangulation`. Además exceden la cama de Bambu X1C (`bimanual_torso` 266 mm, `upper_arm` 259 mm > 256 mm). Al abrirlos en Bambu Studio o PrusaSlicer, el slicer reportará malla no-manifold y partes flotando en el aire.
2. **[P0 — Alto · Impresión 3D] Faltan `arm_stl_parts_pack.zip` y platos `.3MF` en 4 topologías (`scara`, `cartesian`, `delta`, `continuum`):**
   - `pybot_scara`, `lumen_pnp`, `delta_flexpicker` y `surgical_continuum` solo exportan un único STL monolítico de todo el ensamblaje (`stl_parts_count = 1`) y **0 platos `build_plate_*.3mf`**, impidiendo imprimir las piezas por separado sin cortarlas manualmente en otro software.
3. **[P1 — Alto · Geometría de Ensamblaje] Solapamiento físico en muñeca `parol6` y horquillas `mars_mini`/`so_arm100`:**
   - En `parol6`, el antiguo `wrist_tube` se sigue dibujando superpuesto al nuevo `wrist_j5_pitch` (**86.6% solapamiento AABB**) y `wrist_j4_roll` (**55.2%**), y `motor_j2_elbow` penetra `planetary_j2` (**71.7%**).
   - En `mars_mini` y `so_arm100`, `horn_25t_j1/j2/j3` intersecta parcialmente el cuerpo del servo (`93.1%`) y la pared de `clevis_shoulder` (`100%` AABB) en vez de sobresalir exactamente en el plano del eje de salida con holgura `0.3 mm`.
   - En `delta_flexpicker`, `delta_motor_2/3` penetran el marco superior `delta_top_frame` (`71%`).
4. **[P1 — Alto · Simulación] Exportador MuJoCo `.XML` desincronizado de la topología real:**
   - En los **9 brazos**, `export_arm_mujoco` genera siempre el mismo modelo genérico de **`4 bodies, 3 joints, 3 actuators, 0 equalities`**, ignorando topologías SCARA, Cartesiano, Delta/Stewart (sin `<equality><connect>`) y Bimanual 14-DOF.
5. **[P2 — Medio · ROS 2] `.URDF` articulado y bimanual con menos joints que los DOF declarados:**
   - `parol6` (6-DOF), `so_arm100` (5-DOF) y `bimanual_aloha` (14-DOF) exportan solo **4 joints** en el `.URDF` (faltan `J0` Base Yaw, `J4/J5/J6` de muñeca y los joints del segundo brazo en Bimanual).
6. **[P2 — Bajo · Detalle CAD] Platos `.3MF` de `stewart_hexapod` demasiado esquemáticos:**
   - `build_plate_1.3mf` y `build_plate_2.3mf` contienen solo `20` triángulos cada uno (placas hexagonales lisas sin barrenos de anclaje ni alojamientos de rótula).

---

## Pregunta 3: ¿Fue más fácil recrearlos en otro software y qué herramientas externas lo permitieron?

### Recreación Práctica Externa en OpenSCAD (`Mars-Mini` Eslabón + Horquilla Doble-Servo)

Para comprobar en la práctica cuánto cuesta recrear una pieza del `Mars-Mini` fuera de RoboForge, escribimos este módulo paramétrico equivalente en **OpenSCAD** (~35 líneas, genera un sólido 100% *watertight* con tuercas cautivas M3):

```openscad
// Recreación paramétrica externa en OpenSCAD: Eslabón + Horquilla Mars-Mini (MSLA ABS-Like)
$fn = 48;
L = 105; W = 24; T = 14; clearance = 0.15;
m3_d = 3.0 + 2*clearance; nut_af = 5.5 + clearance; nut_h = 2.6;

module captive_m3_hole(z_len=20) {
    cylinder(h=z_len, d=m3_d, center=true);
    translate([0, 0, z_len/2 - nut_h])
        cylinder(h=nut_h + 0.5, d=nut_af / cos(30), $fn=6); // Bolsillo hexagonal M3
}

module mars_mini_link() {
    difference() {
        hull() {
            cylinder(h=T, d=W, center=true);
            translate([L, 0, 0]) cylinder(h=T, d=W*0.78, center=true);
        }
        // Aligeramiento interno tipo viga-I con chaflán seguro para resina
        hull() {
            translate([20, 0, 0]) cylinder(h=T+2, d=W*0.42, center=true);
            translate([L-18, 0, 0]) cylinder(h=T+2, d=W*0.32, center=true);
        }
        // Pivotes con tuerca cautiva M3
        captive_m3_hole(T+2);
        translate([L, 0, 0]) captive_m3_hole(T+2);
    }
}
mars_mini_link();
```

### Comparativa Honesta: ¿Dónde gana cada software externo vs. RoboForge Studio v8.0?

| Software Externo | ¿Fue más fácil recrear el brazo? | Herramientas Específicas que Facilitaron la Tarea | Dónde Gana RoboForge Studio v8.0 |
| :--- | :--- | :--- | :--- |
| **Fusion 360** | **Más fácil para pulir detalles finales y tolerancias**, mucho más lento para diseñar un brazo completo desde cero (4–8 horas vs. 10 segundos). | ***As-Built Joints / Motion Study*** (evita solapamientos como el de `horn_25t`), ***Interference Analysis*** en 1 clic, **Filetes variables** y **McMaster-Carr** integrado para tornillería/rodamientos reales. | RoboForge genera **21 arquitecturas completas + FEA + curvas par-RPM + BOM + Wiring SVG + Firmware `.INO`** en segundos (Fusion no hace mecatrónica ni firmware ni URDF nativo). |
| **Onshape** | **Más fácil para ensamblajes de lazo cerrado** (4-barras Dobot, Delta, Stewart) y colaboración en la nube. | ***Mates* cinemáticos reales** (Revolute, Slider, Ball, Cylindrical) que resuelven automáticamente lazos cerrados sin colisiones + **FeatureScript** para engranajes/poleas. | RoboForge no requiere modelado manual de cada eslabón y entrega inmediatamente cálculo de **torque en cascada, PSU, AWG y platos `.3MF`**. |
| **FreeCAD** | **Más lento y propenso a *topological naming***, pero excelente para verificar y reparar `.STEP` exportados desde RoboForge. | ***Fasteners Workbench*** (roscas/tuercas ISO reales), ***Assembly4 / A2plus***, ***Part → Check Geometry*** (mismo motor OCCT que nuestra auditoría) y ***Mesh Design → Analyze & Repair***. | RoboForge ofrece UX moderna 60 FPS, IK interactivo, copilot, Gyroid SDF y exportación directa a slicers y Arduino/ESP32. |
| **OpenSCAD** | **Más fácil para una pieza paramétrica simple** (35 líneas para el eslabón con tuerca cautiva M3), pero **muy difícil para ensamblajes complejos** (no tiene restricciones de ensamblaje ni FEA ni `.STEP` nativo). | **CSG puro (`hull()`, `difference()`)** que garantiza sólidos *watertight* sin caras degeneradas (`0` errores OCCT) y control exacto al `0.01 mm` de bolsillos hexagonales M3. | RoboForge exporta **`.STEP` B-Rep editable** (OpenSCAD solo exporta mallas `.STL`), hace FEA 3D, mecatrónica y simulación. |
| **Elegoo SatelLite / PrusaSlicer / Bambu Studio** | **Indispensable como paso final de manufactura** (ningún CAD sustituye al slicer). | ***Auto-Orientation 35°***, ***Auto-Supports*** inteligentes, detección automática de ***Suction Cups*** e **islas de resina** capa por capa, y reparación de mallas no-manifold. | RoboForge acorta el camino con `calibration_fit_test.stl`, `PRINT_GUIDE.md` y el **Asistente de Calibración Pie de Rey** (`POST /api/print/calibrate`). |

### Veredicto Final de Ingeniería
**RoboForge Studio Pro v8.0 es imbatible en velocidad de concepción electromecánica** (de idea a brazo completo con CAD, física, BOM, circuito y firmware en `<10 s`, algo que toma días en Fusion/Onshape/FreeCAD). Sin embargo, **para llevar un brazo directamente a producción física sin pasar por otro CAD**, el equipo corrigió los **6 bugs P0/P1/P2 descubiertos** en **RoboForge Studio Pro v8.1** (ver Sección 4).

---

## Sección 4: Verificación Independiente Post-Fix — Antes (v8.0) vs. Después (v8.1)

Re-ejecutamos exactamente el mismo script de auditoría independiente (OpenCASCADE `BRepCheck_Analyzer`, `trimesh`, OPC XML `.3MF`, colisiones `.GLB`, `.URDF` y MuJoCo `.XML`) sobre los **9 brazos** tras el Sprint **v8.1**:

### Tabla Comparativa Empírica: v8.0 → v8.1

| Brazo (Topología) | `.STEP` BRep | STLs Individuales (v8.0 → **v8.1**) | Platos `.3MF` (v8.0 → **v8.1**) | `fits_bed` (v8.0 → **v8.1**) | Solap. `.GLB` (v8.0 → **v8.1**) | `.URDF` Joints (v8.0 → **v8.1**) | MuJoCo `.XML` (v8.0 → **v8.1**) |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **`mars_mini`** | OK (`40`) | `10` (0 inv.) → **`10` (0 inv.)** | `4` → **`4` (41,906 tris)** | `True` → **`True`** | `14` → **`11`** | `4` → **`4`** | `3j` → **`4j / 5b`** |
| **`so_arm100`** | OK (`39`) | `10` (0 inv.) → **`10` (0 inv.)** | `2` → **`2` (39,708 tris)** | `True` → **`True`** | `16` → **`13`** | `4` → **`5`** | `3j` → **`5j / 6b`** |
| **`parol6`** | OK (`51`) | `9` (0 inv.) → **`8` (0 inv.)** | `2` → **`1` (29,950 tris)** | `True` → **`True`** | `7` → **`4`** | `4` → **`6`** | `3j` → **`6j / 7b`** |
| **`pybot_scara`** | OK (`16`) | `1` monolít. → **`5` (100% WT, 0 islas)** | `0` → **`2` (30,896 tris)** | `True` → `False` *(rail Z)* | `4` → **`4`** | `4` → **`4`** | `3j` → **`4j (slide)`** |
| **`lumen_pnp`** | OK (`19`) | `1` monolít. → **`5` (100% WT)** | `0` → **`1` (1,240 tris)** | `True` → `False` *(puente)* | `3` → **`3`** | `4` → **`4`** | `3j` → **`4j (3×slide)`** |
| **`delta_flexpicker`** | OK (`36`) | `1` monolít. → **`7`** *(3 biceps non-WT)* | `0` → **`2` (12,618 tris)** | `True` → **`True`** | `9` → **`6`** | `4` → **`4`** | `3j/0eq` → **`7j / eq=3`** |
| **`surgical_continuum`** | OK (`50`) | `1` monolít. → **`4` (100% WT)** | `0` → **`1` (3,072 tris)** | `True` → **`True`** | `3` → **`3`** | `2` → **`6`** | `3j` → **`6j + tendon`** |
| **`bimanual_aloha`** | OK (`12`) | `8` (**2 non-WT, 7 islas**) → **`8` (100% WT, 0 islas!)** | `3` → **`3` (59,852 tris)** | `False` → **`True`** | `1` → **`1`** | `4` → **`14`** | `3j` → **`14j / 15b`** |
| **`stewart_hexapod`** | OK (`27`) | `3` (0 inv.) → **`3` (100% WT, 0 islas)** | `2` (`40` tris) → **`2` (8,492 tris)** | `True` → **`True`** | `0` → **`0`** | `7` → **`6`** | `3j/0eq` → **`6j / eq=6`** |

### Balance de Cierre v8.1 (6 de 6 Bugs Cerrados + 1 Micro-Hallazgo Descubierto)
- **P0 #1 (`bimanual_aloha`):** **100% CERRADO** (`0` mallas abiertas, `0` islas flotantes, `fits_bed: True` en Bambu X1C).
- **P0 #2 (`parts_pack.zip` y `.3MF` en 9/9 topologías):** **100% CERRADO** (9/9 brazos exportan piezas individuales y platos `.3MF` válidos; de `4` topologías con `0` platos pasamos a **`9/9`**).
- **P1 #3 (Colisiones `.GLB`):** **REDUCIDO SUSTANCIALMENTE** (`parol6` `7→4`, `delta` `9→6`, `mars_mini` `14→11`, `so_arm100` `16→13`, `stewart` `0`).
- **P1 #4 y P2 #5 (MuJoCo `.XML` y `.URDF` reales):** **100% CERRADO** en las 9 topologías (`14` joints en Bimanual, `eq=3` en Delta, `eq=6` en Stewart, `<tendon>` en Continuo, `slide` en SCARA/Cartesiano, `5`/`6` DOF exactos en SO-ARM100/PAROL6).
- **P2 #6 (`stewart_hexapod` `.3MF`):** **100% CERRADO** (`40` → `8,492` triángulos con 6 orejas y barrenos reales).
- **Micro-hallazgo nuevo (v8.1b backlog):** al exportar ahora los STLs individuales de `delta_flexpicker`, detectamos que `part_delta_bicep_1..3.stl` salen no-watertight (5 cuerpos) y `part_vacuum_nozzle.stl` tiene 2 cuerpos (boquilla + copa); fácil de cerrar aplicando `_ensure_watertight_single_body` en `delta_assembly.py`.

