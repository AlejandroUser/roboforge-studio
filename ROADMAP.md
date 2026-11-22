# RoboForge Studio — Roadmap & Gestión de Proyecto

## Hito 1 — MVP Funcional (Completado ✅)
- [x] Infraestructura de proyecto + Agent Factory (`AGENTS.md`, `.agents/skills/`).
- [x] Base de datos de 7 materiales reales con anisotropía $XY/Z$.
- [x] 4 arquitecturas de brazo (`Articulado 6-DOF`, `SCARA`, `Biela Delta`, `Pata Robótica`) e interfaces (`NEMA 17/23`, `608/6805`).
- [x] Generador CAD `build123d` con 4 variantes orgánicas y exportación `.STEP`, `.STL`, `.GLB`.
- [x] Estudio web 3D (Three.js) con comparador de métricas y corte de sección en tiempo real.

## Hito 2 — Referencia `.STEP` + Topología Biónica SIMP + Mapa de Esfuerzos (Completado ✅)
- [x] **Analizador de `.STEP` de referencia (`POST /api/analyze-step`)** con cálculo de ahorro `% vs Ref. .STEP`.
- [x] **5ª Variante — Optimización Topológica SIMP (`organic_topopt`)** en NumPy/SciPy + `build123d`.
- [x] **Mapa de Calor de Esfuerzos 3D** con leyenda en MPa.

## Hito 3A — Validación FEA 3D Real (Completado ✅)
- [x] **Mallado tetraédrico 3D desde `.STEP` con `Gmsh` (`gmsh 4.15.2`)** thread-safe (`interruptible=False`).
- [x] **Solver FEM 3D Tet4 de Elasticidad Lineal** vectorizado en NumPy + `scipy.sparse.linalg.spsolve` (empotramiento en raíz, carga distribuida en punta, Von Mises 3D P95/Pico, desplazamiento máximo real y Factor de Seguridad FEA).
- [x] **Exportación de Malla FEA coloreada por vértice (`COLOR_0` `.glb`)** y endpoint `POST /api/fea`.
- [x] **UI Interactiva FEA 3D:** Botón **"Ejecutar FEA 3D (Gmsh)"**, tarjeta de resultados reales (nodos, tetraedros, tiempo, Von Mises P95/Pico, FS FEA) y alternador **"Malla FEA 3D" / "Modelo CAD"**.

## Hito 3B / 3C — Nuevas Familias y Ensamblaje Multi-Eslabón (Próximo 📋)
- [ ] **Paso B:** Segunda familia de piezas — **Monturas Mecatrónicas (NEMA 17/23, Servos, Perfiles 2020)**.
- [ ] **Paso C:** Tercera familia — **Engranajes Involuta y Reductores** con compensación de *backlash* para impresión 3D.
- [ ] **Paso D:** Cadena cinemática multi-eslabón (brazo completo ensamblado + cálculo en cascada + BOM).
