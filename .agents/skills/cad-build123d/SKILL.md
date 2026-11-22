---
name: cad-build123d
description: >-
  Patrones probados para modelar eslabones robóticos orgánicos y paramétricos en
  build123d (OpenCASCADE), asegurando sólidos válidos (watertight) y exportación
  limpia a STEP, STL y GLB.
---

# Skill: Modelado CAD B-Rep con `build123d`

- **Eje X:** A lo largo del eslabón (`x = 0` raíz, `x = length_mm` punta).
- **Eje Y:** Ancho lateral simétrico respecto a `y = 0`.
- **Eje Z:** Altura/espesor de extrusión (plano de impresión FDM en `XY`).
- Envolver siempre `fillet` y cortes booleanos complejos (como contornos SIMP) en fallbacks seguros para garantizar sólidos válidos.
- Exportar `.step`, `.stl` y `.glb` con caché determinista por hash SHA-256.
