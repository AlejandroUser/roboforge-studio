# Referencia de Ingeniería: Brazos Robóticos Avanzados (Open-Source e Industriales)

Este documento recopila la arquitectura mecánica, actuadores, rodamientos y decisiones de diseño de los brazos robóticos más avanzados para guiar el desarrollo de **RoboForge Studio** (especialmente hacia el **Hito 5: Cadena Cinemática Multi-Eslabón**).

---

## 1. Referentes Open-Source (Impresión 3D y Aluminio Híbrido)

| Proyecto | Arquitectura / Material | Alcance y Carga | Actuación y Transmisión | Lección clave para RoboForge |
| :--- | :--- | :--- | :--- | :--- |
| **PAROL6** (*Source Robotics*) | 6-DOF escritorio, estructura cerrada impresa en 3D (`PETG`/`ABS`) | ~400 mm / ~1.0 kg (~0.1 mm repetibilidad) | Steppers con reducciones de precisión y electrónica integrada | Demuestra que carcasas huecas impresas en 3D con buena topología alcanzan precisión semiprofesional. |
| **AR4-MK5** (*Annin Robotics*) | 6-DOF semi-industrial, placas de aluminio mecanizado + piezas impresas | ~629 mm / 1.5–2.0 kg | NEMA 17/23 + reductores planetarios + **correas GT2/HTD y poleas** | Combinar placas/monturas rígidas con transmisiones por correa GT2 reduce juego (*backlash*) y peso en la muñeca. |
| **SO-ARM100 / SO-ARM101** (*LeRobot*) | 6-DOF ultraligero 100% impreso en 3D (`PLA+`/`PETG`) | Escritorio / 200–500 g | Servos inteligentes de bus serial (alta retro-conducibilidad) | Geometría rápida de imprimir sin soportes, optimizada para IA y teleoperación bimanual. |
| **Faze4** | 6-DOF 100% imprimible en 3D con estética industrial | Escritorio / ~0.5–1 kg | **Reductores cicloidales impresos en 3D** + cableado interno | Eje hueco y reductores cicloidales permiten brazos limpios sin cables colgando. |

---

## 2. Actuadores de Nueva Generación Impresos en 3D (QDD y Cicloidales)

Los proyectos más avanzados de actuadores abiertos (*Quasi-Direct Drive*, inspirados en el MIT Mini Cheetah) combinan motores BLDC planos (*pancake*) con reductores compactos impresos en `PA6-CF` / `PETG-CF`:

- **`JeongSeoJin/quasi-direct-drive-actuator`**: Motor BLDC *pancake* + reductor **cicloidal de doble disco desfasado $180^\circ$** (relación 10:1) para cancelar excentricidad y vibración + controlador FOC (`moteus-c1`).
- **`CKraft11/Ironless-QDD-Actuator`**: Actuador QDD 100% impreso en `PA6-GF/CF` con engranaje cicloidal planetario (~29.4 N·m de torque de retención).
- **`ImZadeQasemzade/QDD5010-Actuator`**: BLDC MKS 5010 + planetario 6:1 impreso en 3D + encoder magnético `AS5600` + `SimpleFOC`.
- **`OpenQDD` (*Aaed Musa*)**: Planetario **helicoidal** 9:1 impreso en 3D (~16.3 N·m pico) con controlador FOC `ODrive S1`.

---

## 3. Secretos Mecánicos de Cobots Industriales (UR5e y Franka Panda)

1. **Rodamientos de Rodillos Cruzados (*Crossed Roller Bearings — CRB*):**
   - Rodillos cilíndricos dispuestos a $90^\circ$ alternados en una pista en V.
   - Un solo rodamiento compacto soporta simultáneamente **carga radial, empuje axial y momentos de vuelco ($M_k$)** con máxima rigidez a la deflexión.
2. **Arquitectura de Eje Hueco (*Hollow-Shaft*) y Cableado Interno:**
   - El centro del motor, el reductor (armónico/cicloidal) y el rodamiento de salida es completamente hueco.
   - Cables de potencia, bus EtherCAT/CAN y sensores viajan por el eje central sin retorcerse ni engancharse (por eso RoboForge incluye **Carcasa Hueca Cobot + Canal Pasacables Interno**).
3. **Doble Encoder + Sensado de Torque:**
   - Encoder en lado motor (conmutación FOC rápida) + encoder absoluto en lado salida (compensa la elasticidad/juego del reductor).
4. **Distribución de Masas (Regla de Oro en Robótica):**
   - Cada gramo ahorrado en el antebrazo o muñeca multiplica el ahorro de torque requerido en el hombro y la base ($\tau_{\text{hombro}} = \sum (m_i g L_i + I_i \alpha)$). De ahí el alto impacto de las variantes **Viga I Orgánica**, **Topología SIMP** y **Radios Biónicos** de RoboForge.

---

## 4. Cómo Aplicamos Esto en RoboForge Studio (Hitos 4 y 5)
- **Hito 4 (Actual):** Perfiles de impresora cerrada vs abierta (`PA12-CF`/`ABS` requieren cámara cerrada para piezas QDD/cobot) + **`.3MF` inteligente con refuerzo local 100% en zonas críticas FEA** + *Ghost Overlay* de `.STEP`.
- **Hito 5 (Próximo):** Cadena cinemática multi-eslabón (Base → Hombro → Brazo → Antebrazo → Muñeca) con cálculo de torque en cascada, opciones de rodamiento CRB / `6805` / `608`, exportación **URDF / MuJoCo XML** y **BOM completa**.
