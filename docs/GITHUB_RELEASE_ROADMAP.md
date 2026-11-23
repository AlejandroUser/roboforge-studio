# Ruta de Acción y Lanzamiento en GitHub — RoboForge Studio Pro v7.0

Guía operativa paso a paso para publicar **RoboForge Studio Pro v7.0** en GitHub, proteger el flujo de desarrollo con CI/CD, distribuir el instalador de Windows y hacer crecer el proyecto en la comunidad de robótica e impresión 3D.

---

## 1. Conectar el Repositorio Local a tu Cuenta de GitHub

Antes de subir el código, verifica que los últimos cambios (incluyendo `docs/images/`, `README.md`, `LICENSE` y `.github/`) estén commiteados en local:

```powershell
cd C:\Users\user\.gemini\antigravity\scratch\roboforge
git status
# Si hay cambios pendientes, haz commit primero:
# git add .
# git commit -m "chore(release): add GitHub CI, MIT license, templates, v7.0 README & release roadmap"
```

> **Recomendación de rama principal:** Tu repositorio local está actualmente en `master`. Si prefieres usar `main` como estándar moderno en GitHub, renómbrala antes del primer push:
> ```powershell
> git branch -M main
> ```

### Opción A — Vía GitHub CLI (`gh`) *(Más rápida y automatizada)*

1. Instala GitHub CLI con `winget`:
   ```powershell
   winget install --id GitHub.cli -e
   ```
   *(Cierra y vuelve a abrir PowerShell para que reconozca `gh`.)*
2. Autentica tu cuenta en el navegador:
   ```powershell
   gh auth login
   ```
   Selecciona: **GitHub.com → HTTPS (o SSH) → Login with a web browser**.
3. Crea el repositorio remoto y haz el primer push en un solo paso:
   ```powershell
   gh repo create roboforge-studio --public --source=. --remote=origin --push \
     --description "Parametric CAD, 3D orthotropic FEA, mechatronics & 3D print prep studio for robotic arms (build123d + React/R3F)"
   ```
   *(Cambia `--public` por `--private` si deseas revisarlo en privado antes de abrirlo al público.)*

### Opción B — Vía GitHub.com + Git Remoto (HTTPS con GCM o SSH)

1. Entra a [https://github.com/new](https://github.com/new) y crea un repositorio vacío llamado `roboforge-studio` (**sin** inicializar README, `.gitignore` ni LICENSE porque ya existen en local).
2. Conecta el remoto usando **HTTPS** (aprovechando *Git Credential Manager*, ya configurado en tu equipo) **o SSH** (ya tienes una clave pública `~/.ssh/id_ed25519.pub` que puedes pegar en *GitHub → Settings → SSH and GPG keys*):

   **Por HTTPS (Git Credential Manager):**
   ```powershell
   git remote add origin https://github.com/<tu-usuario>/roboforge-studio.git
   git push -u origin main
   ```

   **Por SSH (si agregas `~/.ssh/id_ed25519.pub` a GitHub):**
   ```powershell
   git remote add origin git@github.com:<tu-usuario>/roboforge-studio.git
   git push -u origin main
   ```

3. Verifica que el workflow de **GitHub Actions (`.github/workflows/ci.yml`)** se ejecute en verde en la pestaña **Actions** del repositorio (`backend-tests` + `frontend-build`).

---

## 2. Estrategia de Ramas, Tags Semánticos y GitHub Release (`.exe`)

### Estrategia de Ramas
- **`main` (protegida):** Siempre compilable y con CI en verde.
  - En *Settings → Branches → Branch protection rules* para `main`:
    - Activar **Require a pull request before merging**.
    - Activar **Require status checks to pass before merging** seleccionando `Backend Physics, API & Schema Tests (Python 3.12)` y `Frontend Typecheck & Production Build (Node 22)`.
- **Ramas de trabajo:**
  - `feat/<nombre>` para nuevas topologías, materiales, módulos CAD o paneles UI.
  - `fix/<nombre>` para correcciones de física, mallas o API.
  - `docs/<nombre>` para documentación, guías y capturas.

### Versionado Semántico (`SemVer`) y Tag `v7.0.0`
```powershell
git tag -a v7.0.0 -m "RoboForge Studio Pro v7.0.0 — 8 kinematic topologies, 0-G/ROV FEA, Gyroid SDF & Real 3D Print Prep"
git push origin v7.0.0
```

### Publicar GitHub Release con el Ejecutable de Windows (`.exe`)
1. Compila el frontend y el paquete standalone de Windows:
   ```powershell
   cd frontend-react
   npm ci && npm run build
   cd ..
   .\.venv\Scripts\python.exe desktop/build_exe.py
   ```
2. Comprime la carpeta portable generada en un `.zip` listo para usuarios finales:
   ```powershell
   Compress-Archive -Path dist\RoboForgeStudio\* -DestinationPath dist\RoboForgeStudio-v7.0.0-win64.zip -Force
   ```
3. Publica la Release en GitHub (vía `gh` o desde *Releases → Draft a new release*):
   ```powershell
   gh release create v7.0.0 dist\RoboForgeStudio-v7.0.0-win64.zip \
     --title "RoboForge Studio Pro v7.0.0" \
     --notes "Primera release pública de RoboForge Studio Pro v7.0: 8 topologías cinemáticas (20 sistemas · 92.7% avg), FEA 3D ortotrópico + 0-G/ROV, relleno biónico Gyroid SDF, circuitos WireViz + firmware .INO y motor de impresión 3D (.3MF + calibration_fit_test.stl + PRINT_GUIDE.md)."
   ```
   *(Opcional futuro: crear un instalador `Setup.exe` con Inno Setup o automatizar el build de Windows en `.github/workflows/release.yml` cuando se suba un tag `v*`.)*

---

## 3. Plan de Crecimiento y Visibilidad del Repositorio

### SEO y Descubribilidad en GitHub
- **Topics recomendados** (en *About → Settings* del repo):
  `robotics`, `cad`, `build123d`, `opencascade`, `3d-printing`, `fea`, `mechatronics`, `robot-arm`, `react-three-fiber`, `fastapi`, `parametric-cad`, `lerobot`, `gyroid`, `3mf`
- **Social Preview Image:** Sube una imagen `1280x640` compuesta con `docs/images/roboforge_v7_bimanual_print.png` y el título *RoboForge Studio Pro v7.0* en *Settings → General → Social preview*.
- **Demo rápida (GIF/Video de 30s):** Graba un clip corto cambiando topologías, inspeccionando el visor 3D y descargando `build_plate_1.3mf` para fijarlo en la parte superior del README y redes.

### Difusión en Comunidades de Robótica e Impresión 3D
- **Robótica Open-Source:** Compartir un post técnico con capturas reales y el benchmark de 20 sistemas en `r/robotics`, `r/ROS`, Discord de **LeRobot (Hugging Face)** y **OpenArm / SO-100**.
- **Impresión 3D y CAD:** Mostrar la generación automática de `build_plate_N.3mf` + `calibration_fit_test.stl` + Gyroid SDF en `r/3Dprinting`, `r/functionalprint` y la comunidad de **build123d / CadQuery**.
- **Comunidad Dev / Makers:** Lanzamiento *Show HN* en Hacker News y publicación en LinkedIn/X destacando la arquitectura Python + OpenCASCADE + React Three Fiber.

### Roadmap Público v8.0 (Próximas Metas)
- [ ] **Exportación URDF / MuJoCo / Isaac Sim** directa desde el ensamblador modular para entrenamiento de políticas RL/VLA.
- [ ] **Simulación Cinemática Inversa (IK) Interactiva** en tiempo real dentro del visor 3D (arrastrar efector final con detección de singularidades y colisiones).
- [ ] **Generador Automático de BOM con Precios Reales** (DigiKey/LCSC/AliExpress) y perfiles de impresora comunitarios (Bambu Lab, Prusa, Voron, Creality).
- [ ] **Workflow `release.yml` Multiplataforma** (compilación automática de binarios Windows/Linux/macOS en cada tag `v*`).
- [ ] **Galería Comunitaria de Presets** vía Pull Requests validados automáticamente por CI.
