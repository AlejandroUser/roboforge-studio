# RoboForge Studio

Aplicación web de **ingeniería computacional y CAD paramétrico** orientada a **mecatrónica, robótica e impresión 3D**. Genera, evalúa físicamente y compara en paralelo **4 arquitecturas estructurales orgánicas** de eslabones/brazos robóticos con exportación real en **`.STEP`**, **`.STL`** y **`.GLB`**.

## Características (MVP Hito 1)
- **4 arquitecturas de brazo preconfiguradas:** Articulado 6-DOF, SCARA, Biela Delta, Pata Robótica / Dron.
- **4 variantes CAD B-Rep generadas con `build123d` (OpenCASCADE):**
  1. **Referencia Ahusada** (*Organic Tapered Solid*)
  2. **Viga Biónica en I** (*Organic Tapered I-Beam*)
  3. **Celosía Orgánica** (*Rounded Truss*)
  4. **Carcasa Hueca Cobot** (*Hollow Tapered Shell*)
- **Física real de impresión 3D y mecatrónica:**
  - Anisotropía de capas ($XY$ vs $Z$), integración numérica de deflexión en viga ahusada, inercia rotacional, torque dinámico, costo de material y alertas DFM/térmicas (ej. PLA+ cerca de motores calientes).
  - Interfaces estándar (`NEMA 17`, `NEMA 23`, rodamientos `608`/`6805`), holgura de impresora e insertos térmicos de latón (`M3`/`M4`/`M5`).
- **Visor 3D interactivo (Three.js):** Órbita, vistas de cámara, modo *wireframe*, plano de corte en tiempo real y descarga directa de `.STEP` y `.STL`.

## Ejecución rápida (Windows)

```powershell
cd C:\Users\user\.gemini\antigravity\scratch\roboforge\backend
..\.venv\Scripts\uvicorn.exe app.main:app --host 127.0.0.1 --port 8000 --reload
```

Luego abre `http://127.0.0.1:8000` en tu navegador.

## Pruebas automatizadas

```powershell
cd C:\Users\user\.gemini\antigravity\scratch\roboforge\backend
..\.venv\Scripts\python.exe -m pytest -v
```
