# Estrategia y Arquitectura — RoboForge Studio Pro (v2.0)

Este documento define el mapa arquitectónico, el sistema de interfaz y el flujo de trabajo para evolucionar **RoboForge Studio** desde el prototipo actual (Hito 3A) hasta una plataforma profesional de ingeniería computacional para **mecatrónica, robótica e impresión 3D**.

---

## 1. Principio Arquitectónico: Núcleo Modular Multi-Familia

Para crecer sin deuda técnica, cada nueva familia de piezas (Eslabones, Monturas, Engranajes, Ensamblajes) implementa un contrato estándar dentro del backend:

```text
backend/app/
├── core/                      # Infraestructura común (caché SHA-256, materiales, DFM, exportador)
├── physics/
│   ├── mechanics.py           # Familia 1: Eslabones robóticos (viga ahusada, SIMP)
│   ├── mount_mechanics.py     # Familia 2: Monturas mecatrónicas (escuadra 90°, cartelas, térmica)
│   ├── gear_mechanics.py      # [Hito 3C] Familia 3: Engranajes involuta (Lewis/AGMA, backlash FDM)
│   ├── topopt.py              # Solver 2D/3D SIMP
│   └── fea_solver.py          # Solver universal FEA 3D (Gmsh Tet4 + Elasticidad Lineal/Ortotrópica)
├── cad/
│   ├── base_link.py / variants.py   # Familia 1: Eslabones (5 variantes)
│   ├── mount_variants.py            # Familia 2: Monturas (Estándar, Cartelas, Aligerada/Térmica)
│   ├── gear_variants.py             # [Hito 3C] Familia 3: Engranajes y poleas GT2
│   ├── step_analyzer.py             # Ingeniería inversa de referencias .STEP
│   └── exporter.py                  # Exportación unificada .STEP / .STL / .GLB / .3MF
└── main.py                    # API REST versionada por familia (/api/generate, /api/mounts/generate, ...)
```

---

## 2. Estrategia de Interfaz Impecable y Profesional (UI/UX v2.0)

### Fase Actual (v0.3 — Studio Modular Multi-Familia)
- **Navegación por Workspace en el Header:** Pestañas de familia (`Eslabones Robóticos` | `Monturas Mecatrónicas` | próx. `Transmisión & Engranajes` | `Ensamblaje`) que cambian dinámicamente el inspector izquierdo sin perder contexto del visor 3D.
- **Visor 3D de Grado Industrial:** Iluminación 3 puntos, grilla en milímetros, vistas rápidas (`Iso/Sup/Front/Lat`), `Wireframe`, `Corte X`, `Mapa de Esfuerzos` analítico y `FEA 3D` real con deformación amplificada (`0×/10×/25×`).
- **Comparador Basado en Decisiones:** Tarjetas con semáforo de ingeniería (`✓ Cumple`, `★ Recomendada`, `✓ Aprobado FEA`), ahorro de masa y descargas directas `.STEP`/`.STL`.

### Evolución UI hacia Pro v2.0 (cuando integremos Ensamblaje + Pareto + Perfiles)
1. **Migración ordenada a React 19 + TypeScript + Vite + React Three Fiber (`@react-three/fiber` + `@react-three/drei`)** aprovechando Node.js LTS ya instalado:
   - Permite estado complejo compartido (árbol de ensamblaje multi-eslabón, undo/redo, gráfico interactivo de Frontera de Pareto y editor de perfiles de impresora).
2. **Sistema de Diseño CAD Profesional:**
   - **Inspector derecho acoplable** con 3 pestañas limpias: *Variantes*, *Simulación FEA* y *Manufactura / BOM*.
   - **Visor Split-Screen opcional:** Comparar dos variantes lado a lado o superponer la pieza `.STEP` de referencia en modo fantasma translúcido (*ghost overlay*) sobre la versión optimizada.
   - **Barra de Comandos / Copiloto IA integrado (`Ctrl+K`):** Ejecutar rediseños por lenguaje natural con bucle automático *Generar → FEA 3D → Ajustar*.

---

## 3. Plan Maestro de Hitos (Orden Óptimo de Desarrollo)

| Hito | Alcance Clave | Valor para el Usuario |
| :--- | :--- | :--- |
| **Hito 3B (Actual)** | **Familia 2: Monturas Mecatrónicas** (NEMA 17/23, Servos MG996R, Perfil 2020) + Selector Multi-Familia en UI + FEA 3D para monturas. | Cubre el 80% de las piezas estructurales de robots reales (brazos + soportes de motores). |
| **Hito 3C** | **Familia 3: Engranajes Involuta y Poleas GT2** con cálculo por torque/RPM y compensación de *backlash* FDM. | Transmisiones completas imprimibles sin prueba-y-error. |
| **Hito 4** | **Perfiles de Impresora/Material del Usuario + Exportador `.3MF` Inteligente** (mallas modificadoras de relleno según FEA) + *Ghost Overlay* del `.STEP` original. | Del navegador directo a OrcaSlicer / Bambu Studio / PrusaSlicer con refuerzo inteligente. |
| **Hito 5** | **Cadena Cinemática Multi-Eslabón + Exportación URDF / MuJoCo + BOM** (migración a React+TS+R3F). | Diseñar y simular un brazo completo ensamblado con cargas en cascada. |
| **Hito 6** | **FEA Ortotrópico 3D + Modal/Térmico + Copiloto IA en bucle cerrado.** | Nivel de ingeniería computacional comparable a herramientas enterprise, 100% abierto y local. |

---

## 4. Flujo de Trabajo con la Agent Factory (Swarm)

Cada hito sigue un ciclo disciplinado de 4 etapas:
1. **Contrato primero:** El Coordinador actualiza `backend/app/schemas.py` y `swarm/plan.json`.
2. **Ejecución paralela sin colisiones:** `@physics-math` (física/FEA), `@cad-kernel` (`build123d`/API) y `@frontend-3d` (UI/Three.js) trabajan en archivos disjuntos y resuelven bugs de integración por el tablero del Swarm.
3. **Verificación doble:** `pytest` completo en backend + subagente `browser` verificando renderizado 3D, consola limpia y flujos interactivos en `http://127.0.0.1:8000`.
4. **Commit atómico y actualización de `ROADMAP.md`.**
