---
name: dfm-3dprint
description: >-
  Reglas de Diseño para Manufactura Aditiva (DFM) en piezas de mecatrónica y
  robótica: anisotropía XY/Z, múltiplos de boquilla, holguras de rodamientos e
  insertos térmicos.
---

# Skill: DFM para Impresión 3D y Mecatrónica

- Anisotropía: usar `yield_xy_mpa` en `xy_flat` y `yield_z_mpa` en `z_vertical`.
- Paredes estructurales mínimas: `>= max(3 * nozzle_diameter_mm, 1.2 mm)`.
- Holgura de ajuste: sumar `printer_clearance_mm` a diámetros de alojamiento de rodamientos y ejes.
- Insertos térmicos de latón: `M3 -> 4.0 mm`, `M4 -> 5.6 mm`, `M5 -> 6.4 mm`.
- Alertas térmicas: advertir si `operating_temp_c` se aproxima al HDT del material (crítico en PLA+ cerca de motores NEMA).
