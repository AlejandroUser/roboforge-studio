# RoboForge Studio — Agent Factory Rules (`AGENTS.md`)

Eres parte de la **Agent Factory de RoboForge Studio**, una aplicación de ingeniería computacional para diseñar, optimizar y validar eslabones y brazos robóticos fabricables en impresión 3D (FDM/SLS) y CNC.

## Reglas Obligatorias de Ingeniería

1. **Sistema de Unidades Estricto (SI / Mecánica):**
   - Longitudes y dimensiones CAD: **milímetros (`mm`)**.
   - Fuerzas: **Newtons (`N`)**.
   - Torques: **Newton-metro (`N·m`)**.
   - Esfuerzos y módulos elásticos: **Megapascales (`MPa` = `N/mm²`)**.
   - Masa: **gramos (`g`)** en reportes de usuario, **kilogramos (`kg`)** en fórmulas dinámicas.
   - Densidad: **`g/cm³`** en `materials.json`.

2. **Integridad Geométrica (`build123d` / OpenCASCADE):**
   - Toda pieza generada debe ser un sólido único válido (`is_valid` y `volume > 0`).
   - Proteger siempre operaciones `fillet` y booleanos con fallback seguro.
   - Nunca romper las caras ni los agujeros de montaje de los extremos A y B al aplicar aligeramientos (perfil I, celosía, carcasa hueca o topología SIMP).

3. **Física Real de Impresión 3D (DFM):**
   - Usar siempre el límite elástico ajustado por orientación de impresión (`yield_xy_mpa` vs `yield_z_mpa`).
   - Emitir alertas claras para temperatura de motores, paredes mínimas respecto a boquilla y factor de seguridad/deflexión.

4. **Verificación Continua:**
   - Todo cambio en `backend/app/cad/` o `backend/app/physics/` debe pasar `pytest` desde la raíz del repositorio.
