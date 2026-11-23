# RoboForge Studio Pro v8.0

[![Python 3.12+](https://img.shields.io/badge/Python-3.12%2B-3776AB?logo=python&logoColor=white)](https://www.python.org/)
[![React 19 + R3F](https://img.shields.io/badge/React_19-R3F_%2F_Three.js-61DAFB?logo=react&logoColor=000)](https://react.dev/)
[![build123d / OCCT](https://img.shields.io/badge/CAD-build123d_%2F_OpenCASCADE-0EA5E9)](https://github.com/gumyr/build123d)
[![Docker / HF Spaces](https://img.shields.io/badge/Docker-HF_Spaces_7860-2496ED?logo=docker&logoColor=white)](Dockerfile)
[![License: MIT](https://img.shields.io/badge/License-MIT-10B981.svg)](LICENSE)
[![Benchmark](https://img.shields.io/badge/21_Robotic_Systems-93.0%25_avg-8B5CF6)](docs/)

**RoboForge Studio Pro v8.0** es una plataforma integral de **ingeniería computacional, CAD paramétrico B-Rep, simulación FEA 3D ortotrópica, cinemática inversa (IK) en vivo, control de hardware USB/Wi-Fi y manufactura aditiva FDM + Resina MSLA** para diseñar, validar, imprimir y operar brazos robóticos reales de principio a fin.

Genera geometría manufacturable en **`.STEP`**, **`.STL`**, **`.GLB`**, **`.DXF`**, **`.URDF` / MuJoCo** y platos listos para laminar en **`.3MF`** con pruebas de tolerancia, diagramas de cableado WireViz y firmware **Arduino / ESP32 Wi-Fi `.INO`** autogenerado.

---

## Galería Visual

| Bimanual 14-DOF + Motor de Impresión 3D | Brazo 5-DOF + Circuito Mecatrónico WireViz | Continuum PCC (Tendones + Fuelles TPU) |
| :---: | :---: | :---: |
| ![Bimanual 14-DOF & 3D Print Prep](docs/images/roboforge_v7_bimanual_print.png) | ![Brazo 5-DOF & WireViz Circuit](docs/images/roboforge_v7_brazo5_circuit.png) | ![Continuum PCC](docs/images/roboforge_v7_continuum_pcc.png) |

---

## Novedades Destacadas de v8.0

- **Proyecto Demostrador `RoboForge Mars-Mini` (~$40 USD · Elegoo Mars 5 Ultra MSLA 9K):**
  - Brazo robótico 4-DOF paralelogramo + pinza 100% imprimible en **Resina ABS-Like** (`153.36 × 77.76 × 165 mm`, **0 piezas fuera de cama**), con tuercas cautivas M3, servos económicos (`3× MG996R` + `1× MG90S`), bus `ESP32 + PCA9685 (5V 5A)`, receta dedicada para **Elegoo SatelLite / Chitubox** y servidor Web Wi-Fi integrado (`mars_mini_esp32_wifi.ino`). Consulta la guía paso a paso en [docs/MARS_MINI_BUILD_GUIDE.md](docs/MARS_MINI_BUILD_GUIDE.md).
- **Cinemática Inversa (IK) 3D Interactiva a 60 FPS:**
  - Toggle **`🎯 Modo IK (Objetivo X·Y·Z)`** en el visor 3D con mira objetivo cian en tiempo real, resolución analítica de `J0/J1/J2/J3` (articulado y SCARA) con *clamping* seguro de alcance y presets rápidos (*Mesa*, *Centro*, *Alto*).
- **Control de Hardware USB en Vivo (`LiveHardwarePanel`):**
  - Envía poses `P:<j0>,<j1>,<j2>,<j3>,<g>` en tiempo real desde el visor 3D hacia tu Arduino/ESP32 mediante **WebSerial API** directo del navegador o puente backend (`pyserial` + puerto **`VIRTUAL_SIM`** para pruebas sin placa física).
- **Asistente de Calibración por Pie de Rey (Cupón 15 min):**
  - Mide el agujero M3 y alojamiento del cupón `calibration_fit_test.stl` con pie de rey y obtén al instante la compensación exacta de *XY Hole / Tolerance* para **Elegoo SatelLite** u **OrcaSlicer**.
- **Despliegue en 1 Comando con Docker & HuggingFace Spaces:**
  - `Dockerfile` multi-stage (`node:22-slim` + `python:3.12-slim`) listo para ejecutar en local o subir directamente a **HuggingFace Spaces** en el puerto `7860`.

---

## Capacidades de Ingeniería (v7.0 – v8.0)

- **8+ Topologías Cinemáticas Paramétricas (21 Sistemas · 93.0% promedio de replicabilidad):**
  - Articulado Serial (`6-DOF` / `7-DOF` QDD), `Mars-Mini` (4-DOF paralelogramo MSLA), `SCARA 4-DOF` (Z prismático), `Delta Paralelo`, `Pórtico Cartesiano (Gantry)`, `Continuum PCC` (tendones + fuelles TPU), `Bimanual 14-DOF`, `Plataforma Stewart 6-UPS` y mecanismos `4-Bar / Capstan / Bevel`.
- **FEA 3D Ortotrópico y Física Extrema:**
  - Anisotropía FDM ($XY$ vs $Z$) e isotropía MSLA (`resin_abs_like`), deflexión, torque dinámico, entornos **0-G (espacial)** y **ROV submarino**, polímeros técnicos (`PLA+`, `PETG`, `ABS/ASA`, `PA-CF`, `TPU 95A`, `Resina ABS-Like`) y metales (`Al 6061`, `Ti-6Al-4V`, `Acero 304`).
- **Estructuras Orgánicas B-Rep y Relleno Biónico Gyroid SDF:**
  - `build123d` (OpenCASCADE) + generación volumétrica **Gyroid SDF** (`scikit-image` + `trimesh`) + reductores cicloidales y planetarios 3D.
- **Mecatrónica, Circuitos WireViz y Firmware Dual `.INO`:**
  - Esquemáticos SVG + YAML WireViz, BOM completo con costos reales y generación de firmware **Arduino UNO/Nano** + **ESP32 Wi-Fi Web Server**.
- **Motor Real de Impresión 3D (FDM + MSLA Resina):**
  - Empaquetado multi-cama **`build_plate_N.3mf`**, **`calibration_fit_test.stl`**, y manual **`PRINT_GUIDE.md`** adaptado automáticamente a FDM (*Bambu/OrcaSlicer*) o MSLA (*Elegoo SatelLite/Chitubox*).

---

## Arranque en 1 Comando con Docker (Local o HuggingFace Spaces)

```powershell
docker build -t roboforge .
docker run -p 7860:7860 roboforge
```

Abre `http://127.0.0.1:7860` en tu navegador. El mismo `Dockerfile` funciona directamente al crear un **Docker Space** en **HuggingFace Spaces** (puerto `7860`).

---

## Instalación Local (Desarrollo y Escritorio Windows)

### Requisitos Previos
- **Python 3.12+**
- **Node.js 22+**
- **Windows 10/11**, Linux o macOS

### 1. Instalar dependencias

```powershell
git clone https://github.com/<tu-usuario>/roboforge-studio.git
cd roboforge-studio

# Backend Python
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r backend/requirements.txt

# Frontend React + Three.js (R3F)
cd frontend-react
npm ci
npm run build
cd ..
```

### 2. Ejecutar RoboForge Studio

- **Modo Escritorio Nativo (Windows):** Ejecuta `RoboForge_Studio.bat` para abrir la app en ventana nativa `pywebview`.
- **Modo Servidor Web (`uvicorn`):**
  ```powershell
  .\.venv\Scripts\uvicorn.exe app.main:app --app-dir backend --host 127.0.0.1 --port 8000 --reload
  ```
  Luego abre `http://127.0.0.1:8000`.
- **Modo Desarrollo Frontend (Hot Reload):**
  ```powershell
  cd frontend-react
  npm run dev
  ```

### 3. Empaquetar Ejecutable Standalone Windows (`.exe`)

```powershell
.\.venv\Scripts\pip.exe install pyinstaller pillow
.\.venv\Scripts\python.exe desktop/build_exe.py
```

El ejecutable autocontenido se genera en `dist/RoboForgeStudio/RoboForgeStudio.exe`.

---

## Pruebas Automatizadas

```powershell
# Suite rápida CI (física, mecatrónica, Mars-Mini MSLA, IK, hardware USB y regresiones v4–v8)
.\.venv\Scripts\python.exe -m pytest -v \
  backend/tests/test_physics.py \
  backend/tests/test_mechatronics_v4.py \
  backend/tests/test_fabri_arm_5.py \
  backend/tests/test_universal_arms_v5.py \
  backend/tests/test_wave2_fixes_v6.py \
  backend/tests/test_wave3_v7.py \
  backend/tests/test_firmware_and_db.py \
  backend/tests/test_mars_mini_v71.py \
  backend/tests/test_v8_ik_and_hardware.py

# Verificación de tipos y build de producción del frontend
cd frontend-react
npx tsc --noEmit && npm run build
```

---

## Guías y Roadmap

- [docs/MARS_MINI_BUILD_GUIDE.md](docs/MARS_MINI_BUILD_GUIDE.md) — Guía paso a paso para imprimir y armar el demostrador **Mars-Mini (~$40 USD)** en Elegoo Mars 5 Ultra.
- [docs/GITHUB_RELEASE_ROADMAP.md](docs/GITHUB_RELEASE_ROADMAP.md) — Ruta de publicación en GitHub, CI/CD, releases `.exe` y estrategia de comunidad.

## Licencia

Distribuido bajo licencia **MIT**. Consulta [LICENSE](LICENSE) para más detalles.
