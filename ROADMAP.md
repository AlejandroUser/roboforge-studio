# RoboForge Studio — Roadmap & Gestión de Proyecto

> Ver estrategia completa hacia la v2.0 en [`docs/ARCHITECTURE_V2.md`](docs/ARCHITECTURE_V2.md).

## Hito 1 — MVP Funcional (Completado ✅)
- [x] Infraestructura de proyecto + Agent Factory (`AGENTS.md`, `.agents/skills/`).
- [x] Base de datos de 7 materiales reales con anisotropía $XY/Z$.
- [x] 4 arquitecturas de brazo (`Articulado 6-DOF`, `SCARA`, `Biela Delta`, `Pata Robótica`) e interfaces (`NEMA 17/23`, `608/6805`).
- [x] Generador CAD `build123d` con 4 variantes orgánicas y exportación `.STEP`, `.STL`, `.GLB`.
- [x] Estudio web 3D (Three.js) con comparador de métricas y corte de sección en tiempo real.

## Hito 2 — Referencia `.STEP` + Topología Biónica SIMP + Mapa de Esfuerzos (Completado ✅)
- [x] Analizador de `.STEP` de referencia (`POST /api/analyze-step`) con cálculo de ahorro `% vs Ref. .STEP`.
- [x] 5ª Variante — Optimización Topológica SIMP (`organic_topopt`) en NumPy/SciPy + `build123d`.
- [x] Mapa de Calor de Esfuerzos 3D con leyenda en MPa.

## Hito 3A — Validación FEA 3D Real + Pulido FEA (Completado ✅)
- [x] Mallado tetraédrico 3D desde `.STEP` con `Gmsh` (`gmsh 4.15.2`) thread-safe.
- [x] Solver FEM 3D Tet4 de Elasticidad Lineal vectorizado (`scipy.sparse.linalg.spsolve`), Von Mises P95/Pico y Factor de Seguridad FEA.
- [x] Exportación de Malla FEA coloreada por vértice (`COLOR_0` `.glb`), selector `Carga Y / Carga Z` y deformación amplificada `0× / 10× / 25×`.

## Hito 3B — Familia 2: Monturas Mecatrónicas + UI Multi-Familia (Completado ✅)
- [x] Estrategia arquitectónica y plan UI v2.0 ([`docs/ARCHITECTURE_V2.md`](docs/ARCHITECTURE_V2.md)).
- [x] 4 presets reales de actuadores (`NEMA 17 + 2020`, `NEMA 23 Heavy`, `Servo MG996R`, `Escuadra 2020`) y 3 bases (`2020`, `2040`, `Placa Plana`).
- [x] 3 variantes B-Rep en `build123d` + física de escuadra 90° + FEA 3D real (`POST /api/mounts/fea`).

## Hito 3C — Familia 3: Transmisión (Engranajes Involuta y Poleas GT2) (Completado ✅)
- [x] 4 presets reales (`Piñón NEMA 17`, `Engranaje Alto Torque`, `Polea GT2 20T`, `Polea GT2 40T`).
- [x] Perfil matemático real de engranaje involuta con compensación de *backlash* FDM y poleas GT2 con pestañas de retención + cubo con prisionero M3 + D-shaft.
- [x] 3 variantes B-Rep en `build123d`: **Sólida**, **Alma Rebajada Aligerada** y **Radios Biónicos Aligerados**.
- [x] Ecuación de Lewis + factor dinámico $K_v$, alertas de *undercutting* y resolución de boquilla, y **FEA 3D real (`POST /api/gears/fea`)**.
- [x] 3ª pestaña **`Transmisión`** en el Header Multi-Familia.

## Hito 4 / 5 — Próximos Pasos (Ver `docs/ARCHITECTURE_V2.md`)
- [ ] **Hito 4:** **Perfiles guardables de Impresora/Material + Exportador `.3MF` Inteligente** (mallas modificadoras de relleno según FEA) + *Ghost Overlay* de referencia `.STEP`.
- [ ] **Hito 5:** **Cadena Cinemática Multi-Eslabón + Exportación URDF / MuJoCo + BOM** (migración planificada a React + TypeScript + R3F).
