# RoboForge Studio — Roadmap & Gestión de Proyecto

## Hito 1 — MVP Funcional (Completado ✅)
- [x] Infraestructura de proyecto + Agent Factory (`AGENTS.md`, `.agents/skills/`).
- [x] Base de datos de 7 materiales reales con anisotropía $XY/Z$ (`PLA+`, `PETG`, `ABS/ASA`, `PETG-CF`, `PA12-CF`, `Al 6061`, `Carbon Plate`).
- [x] 4 arquitecturas de brazo (`Articulado 6-DOF`, `SCARA`, `Biela Delta`, `Pata Robótica`) e interfaces (`NEMA 17/23`, `608/6805`).
- [x] Generador CAD `build123d` con 4 variantes orgánicas y exportación `.STEP`, `.STL`, `.GLB`.
- [x] Estudio web 3D (Three.js) con comparador de métricas y corte de sección en tiempo real.
- [x] Suite de 13 tests automatizados (`pytest`) + verificación E2E en navegador.

## Hito 2 — Referencia `.STEP` + Topología Biónica SIMP + Mapa de Esfuerzos (Completado ✅)
- [x] **Analizador de `.STEP` de referencia (`POST /api/analyze-step`):** Subida drag & drop de `.STEP`/`.STP`, extracción de caja envolvente, volumen y agujeros cilíndricos con cálculo de ahorro `% vs Ref. .STEP`.
- [x] **5ª Variante — Optimización Topológica SIMP (`organic_topopt`):** Solver real 2D Q4 SIMP (`spsolve` + filtro de sensibilidad + OC) que talla vaciados biónicos suaves con splines periódicas en `build123d`.
- [x] **Mapa de Calor de Esfuerzos 3D (Heatmap):** Gradiente azul → cian → verde → amarillo → rojo en tiempo real sobre la malla 3D con leyenda en MPa.
- [x] **Suite ampliada de tests (`pytest`):** 16 tests automatizados cubriendo SIMP, round-trip `.STEP` y las 5 variantes.

## Hito 3 — Validación FEA Real & Nuevas Familias (Próximo 📋)
- [ ] Simulación FEA automática completa (`Gmsh` + `CalculiX`) para validar esfuerzos 3D complejos.
- [ ] Generador de **Monturas Mecatrónicas (NEMA 17/23, Servos, Perfiles 2020)**.
- [ ] Generador de **Engranajes Involuta y Reductores** con compensación de *backlash* para impresión 3D.
- [ ] Cadena cinemática multi-eslabón (brazo completo ensamblado + exportación BOM).
