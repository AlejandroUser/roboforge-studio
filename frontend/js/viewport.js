/**
 * viewport.js — Motor de visualización 3D CAD en Three.js para RoboForge Studio
 */

import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { GLTFLoader } from "three/addons/loaders/GLTFLoader.js";

export class Viewport3D {
  constructor(container) {
    this.container = container;
    this.scene = new THREE.Scene();
    this.scene.background = null; // Permite ver el degradado radial CSS del estudio

    const width = container.clientWidth || 800;
    const height = container.clientHeight || 600;

    this.camera = new THREE.PerspectiveCamera(42, width / height, 0.5, 5000);
    this.camera.position.set(170, 125, 190);

    this.renderer = new THREE.WebGLRenderer({
      antialias: true,
      alpha: true,
      powerPreference: "high-performance",
    });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    this.renderer.setSize(width, height);
    this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
    this.renderer.toneMappingExposure = 1.15;
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    this.renderer.localClippingEnabled = true;

    container.appendChild(this.renderer.domElement);

    // OrbitControls
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.07;
    this.controls.rotateSpeed = 0.85;
    this.controls.panSpeed = 0.85;
    this.controls.zoomSpeed = 1.05;
    this.controls.minDistance = 20;
    this.controls.maxDistance = 1500;
    this.controls.autoRotate = false;
    this.controls.autoRotateSpeed = 1.6;

    // Estado del visor
    this.loader = new GLTFLoader();
    this.modelCache = new Map();
    this.activeGroup = null;
    this.modelBounds = new THREE.Box3(
      new THREE.Vector3(-90, 0, -25),
      new THREE.Vector3(90, 25, 25)
    );
    this.modelRadius = 110;
    this.modelCenter = new THREE.Vector3(0, 12, 0);

    this.wireframeEnabled = false;
    this.sectionCutEnabled = false;
    this.sectionCutRatio = 0.5;
    this.heatmapEnabled = false;
    this.isShowingFeaMesh = false;
    this.stressProfile = null;
    this.ghostGroup = null;
    this.ghostUrl = null;
    this.ghostVisible = false;
    this.clipPlaneX = new THREE.Plane(new THREE.Vector3(-1, 0, 0), 0);
    this.loadToken = 0;

    // Animación suave de cámara
    this.camAnim = null;

    this._setupLighting();
    this._setupStudioGrid();
    this._setupResizeObserver();

    // Detener animación de cámara si el usuario interactúa
    this.controls.addEventListener("start", () => {
      this.camAnim = null;
    });

    this._animate = this._animate.bind(this);
    this._rafId = requestAnimationFrame(this._animate);
  }

  _setupLighting() {
    // Iluminación de estudio 3 puntos + Hemisferio
    const hemiLight = new THREE.HemisphereLight(0xdbeafe, 0x1e293b, 0.85);
    hemiLight.position.set(0, 300, 0);
    this.scene.add(hemiLight);

    // Key light (luz principal cálida-neutra)
    const keyLight = new THREE.DirectionalLight(0xffffff, 1.85);
    keyLight.position.set(180, 260, 200);
    this.scene.add(keyLight);

    // Fill light (relleno frío cian suave)
    const fillLight = new THREE.DirectionalLight(0x93c5fd, 0.85);
    fillLight.position.set(-220, 110, 140);
    this.scene.add(fillLight);

    // Rim light (contraluz para perfilar chaflanes y nervios)
    const rimLight = new THREE.DirectionalLight(0x38bdf8, 1.0);
    rimLight.position.set(-80, 180, -260);
    this.scene.add(rimLight);

    // Luz inferior muy sutil para que las cavidades inferiores no queden negras
    const bounceLight = new THREE.DirectionalLight(0x475569, 0.35);
    bounceLight.position.set(0, -180, 0);
    this.scene.add(bounceLight);
  }

  _setupStudioGrid() {
    // Grilla principal en milímetros (500mm x 500mm, divisiones de 10mm)
    const gridFine = new THREE.GridHelper(500, 50, 0x26354d, 0x162030);
    gridFine.position.y = -0.15;
    if (Array.isArray(gridFine.material)) {
      gridFine.material.forEach((m) => {
        m.transparent = true;
        m.opacity = 0.55;
      });
    } else {
      gridFine.material.transparent = true;
      gridFine.material.opacity = 0.55;
    }
    this.scene.add(gridFine);

    // Ejes discretos en el origen
    const axes = new THREE.AxesHelper(32);
    axes.position.set(0, 0.1, 0);
    axes.material.transparent = true;
    axes.material.opacity = 0.55;
    this.scene.add(axes);
  }

  _setupResizeObserver() {
    this.resizeObserver = new ResizeObserver(() => {
      const w = this.container.clientWidth;
      const h = this.container.clientHeight;
      if (!w || !h) return;
      this.camera.aspect = w / h;
      this.camera.updateProjectionMatrix();
      this.renderer.setSize(w, h);
    });
    this.resizeObserver.observe(this.container);
  }

  /**
   * Carga un archivo GLB por URL (usando caché en memoria si ya fue cargado).
   */
  async loadModel(glbUrl, { autoFit = true, isFeaMesh = false, bypassCache = false } = {}) {
    const token = ++this.loadToken;

    let cached = bypassCache ? null : this.modelCache.get(glbUrl);
    if (!cached) {
      const gltf = await new Promise((resolve, reject) => {
        this.loader.load(
          glbUrl,
          (data) => resolve(data),
          undefined,
          (err) => reject(err)
        );
      });

      if (token !== this.loadToken) return null;

      const wrapper = this._prepareLoadedScene(gltf.scene, { isFeaMesh });
      this.modelCache.set(glbUrl, wrapper);
      cached = wrapper;
    }

    if (token !== this.loadToken) return null;

    if (this.activeGroup) {
      this.scene.remove(this.activeGroup);
    }

    this.activeGroup = cached;
    this.isShowingFeaMesh = Boolean(isFeaMesh);
    this.scene.add(this.activeGroup);

    // Recalcular bounds del modelo activo
    this.modelBounds = new THREE.Box3().setFromObject(this.activeGroup);
    const sphere = new THREE.Sphere();
    this.modelBounds.getBoundingSphere(sphere);
    this.modelCenter.copy(sphere.center);
    this.modelRadius = Math.max(sphere.radius, 20);

    // Reaplicar wireframe y corte de sección al modelo activo
    this._applyMaterialStates();

    if (autoFit) {
      this.setCameraView("iso", true);
    }

    const size = new THREE.Vector3();
    this.modelBounds.getSize(size);
    return { size };
  }

  /**
   * Centra el modelo sobre la grilla (Y >= 0, X/Z centrados) y mejora materiales PBR.
   */
  _prepareLoadedScene(root, { isFeaMesh = false } = {}) {
    const group = new THREE.Group();

    // Asegurar normales suaves/limpias y propiedades PBR industriales (o vertexColors reales para FEA)
    root.traverse((obj) => {
      if (!obj.isMesh) return;
      if (obj.geometry && !obj.geometry.attributes.normal) {
        obj.geometry.computeVertexNormals();
      }
      const hasVertexColors = Boolean(obj.geometry?.attributes?.color);
      const materials = Array.isArray(obj.material) ? obj.material : [obj.material];
      const enhanced = materials.map((mat) => {
        if (isFeaMesh || hasVertexColors) {
          return new THREE.MeshStandardMaterial({
            color: 0xffffff,
            vertexColors: true,
            metalness: 0.12,
            roughness: 0.48,
          });
        }
        if (!mat) {
          return new THREE.MeshStandardMaterial({
            color: 0x64748b,
            metalness: 0.35,
            roughness: 0.42,
          });
        }
        const clone = mat.clone();
        if (clone.isMeshStandardMaterial) {
          clone.metalness = Math.min(Math.max(clone.metalness ?? 0.3, 0.2), 0.7);
          clone.roughness = Math.min(Math.max(clone.roughness ?? 0.45, 0.25), 0.65);
        }
        return clone;
      });
      obj.material = Array.isArray(obj.material) ? enhanced : enhanced[0];
      obj.userData.originalMaterial = obj.material;
    });

    // Todos los sólidos CAD de build123d/Gmsh vienen en convención Z-up (base en plano XY, altura en Z).
    // Rotar -90° en X convierte Z-up a Y-up de Three.js para que tanto eslabones como monturas en L
    // queden apoyados correctamente sobre la grilla horizontal del estudio.
    root.rotation.x = -Math.PI / 2;
    root.updateMatrixWorld(true);

    const box = new THREE.Box3().setFromObject(root);
    const center = new THREE.Vector3();
    box.getCenter(center);

    // Centrar en X y Z, apoyar base en Y = 0
    root.position.x -= center.x;
    root.position.z -= center.z;
    root.position.y -= box.min.y;

    group.add(root);
    group.updateMatrixWorld(true);
    return group;
  }

  _stressToRgb(t) {
    const stops = [
      [0.0, [0.145, 0.388, 0.922]],  // #2563eb Azul
      [0.25, [0.024, 0.714, 0.831]], // #06b6d4 Cian
      [0.5, [0.133, 0.773, 0.369]],  // #22c55e Verde
      [0.75, [0.918, 0.702, 0.031]], // #eab308 Amarillo
      [1.0, [0.937, 0.267, 0.267]],  // #ef4444 Rojo
    ];
    const clamped = Math.min(Math.max(t, 0), 1);
    for (let i = 0; i < stops.length - 1; i++) {
      const [t0, c0] = stops[i];
      const [t1, c1] = stops[i + 1];
      if (clamped >= t0 && clamped <= t1) {
        const f = (clamped - t0) / (t1 - t0);
        return [
          c0[0] + (c1[0] - c0[0]) * f,
          c0[1] + (c1[1] - c0[1]) * f,
          c0[2] + (c1[2] - c0[2]) * f,
        ];
      }
    }
    return stops[stops.length - 1][1];
  }

  _sampleStressProfile(u) {
    const prof = this.stressProfile;
    if (!Array.isArray(prof) || prof.length === 0) {
      return Math.min(Math.max(1 - u, 0), 1);
    }
    if (prof.length === 1) return Math.min(Math.max(Number(prof[0]) || 0, 0), 1);
    const idx = Math.min(Math.max(u, 0), 1) * (prof.length - 1);
    const i0 = Math.floor(idx);
    const i1 = Math.min(i0 + 1, prof.length - 1);
    const frac = idx - i0;
    const v0 = Number(prof[i0]) || 0;
    const v1 = Number(prof[i1]) || 0;
    return Math.min(Math.max(v0 + (v1 - v0) * frac, 0), 1);
  }

  _applyHeatmapColorsToMesh(mesh, minX, spanX) {
    const geom = mesh.geometry;
    const pos = geom?.attributes?.position;
    if (!pos) return;

    const count = pos.count;
    let colorAttr = geom.attributes.color;
    if (!colorAttr || colorAttr.count !== count) {
      colorAttr = new THREE.BufferAttribute(new Float32Array(count * 3), 3);
      geom.setAttribute("color", colorAttr);
    }

    const v = new THREE.Vector3();
    for (let i = 0; i < count; i++) {
      v.fromBufferAttribute(pos, i).applyMatrix4(mesh.matrixWorld);
      const u = Math.min(Math.max((v.x - minX) / spanX, 0), 1);
      const stress = this._sampleStressProfile(u);
      const [r, g, b] = this._stressToRgb(stress);
      colorAttr.setXYZ(i, r, g, b);
    }
    colorAttr.needsUpdate = true;

    if (!mesh.userData.heatmapMaterial) {
      mesh.userData.heatmapMaterial = new THREE.MeshStandardMaterial({
        vertexColors: true,
        roughness: 0.45,
        metalness: 0.12,
      });
    }
    mesh.material = mesh.userData.heatmapMaterial;
  }

  _applyMaterialStates() {
    if (!this.activeGroup) return;

    this.activeGroup.updateMatrixWorld(true);
    const minX = this.modelBounds.min.x;
    const maxX = this.modelBounds.max.x;
    const spanX = Math.max(maxX - minX, 1e-4);
    const pad = 2.0;
    const cutX = minX - pad + (maxX - minX + pad * 2) * this.sectionCutRatio;
    this.clipPlaneX.constant = cutX;

    const activePlanes = this.sectionCutEnabled ? [this.clipPlaneX] : [];

    this.activeGroup.traverse((obj) => {
      if (!obj.isMesh) return;

      if (this.isShowingFeaMesh) {
        if (obj.userData.originalMaterial) {
          obj.material = obj.userData.originalMaterial;
        }
      } else if (this.heatmapEnabled) {
        this._applyHeatmapColorsToMesh(obj, minX, spanX);
      } else if (obj.userData.originalMaterial) {
        obj.material = obj.userData.originalMaterial;
      }

      const mats = Array.isArray(obj.material) ? obj.material : [obj.material];
      for (const m of mats) {
        if (!m) continue;
        m.wireframe = this.wireframeEnabled;
        m.clippingPlanes = activePlanes;
        m.clipShadows = this.sectionCutEnabled;
        m.side = this.sectionCutEnabled ? THREE.DoubleSide : THREE.FrontSide;
        m.needsUpdate = true;
      }
    });
  }

  setStressHeatmap(enabled, stressProfile = this.stressProfile) {
    this.heatmapEnabled = Boolean(enabled);
    if (stressProfile !== undefined) {
      this.stressProfile = stressProfile;
    }
    this._applyMaterialStates();
  }

  setWireframe(enabled) {
    this.wireframeEnabled = Boolean(enabled);
    this._applyMaterialStates();
  }

  setAutoRotate(enabled) {
    this.controls.autoRotate = Boolean(enabled);
  }

  setSectionCut(enabled, ratio = this.sectionCutRatio) {
    this.sectionCutEnabled = Boolean(enabled);
    this.sectionCutRatio = Math.min(Math.max(ratio, 0), 1);
    this._applyMaterialStates();
  }

  /**
   * Superpone el sólido .STEP original como malla holográfica cian translúcida (Fantasma .STEP).
   */
  async setGhostOverlay(glbUrl, visible) {
    this.ghostVisible = Boolean(visible);
    if (!glbUrl) {
      this.clearGhostOverlay();
      return;
    }

    if (this.ghostGroup && this.ghostUrl === glbUrl) {
      this.ghostGroup.visible = this.ghostVisible;
      return;
    }

    this.clearGhostOverlay();
    if (!this.ghostVisible) return;

    const gltf = await new Promise((resolve, reject) => {
      this.loader.load(glbUrl, (d) => resolve(d), undefined, (e) => reject(e));
    });

    const root = gltf.scene;
    const ghostMat = new THREE.MeshStandardMaterial({
      color: 0x38bdf8,
      emissive: 0x0ea5e9,
      emissiveIntensity: 0.35,
      roughness: 0.35,
      metalness: 0.1,
      transparent: true,
      opacity: 0.28,
      depthWrite: false,
      side: THREE.DoubleSide,
    });

    root.traverse((obj) => {
      if (!obj.isMesh) return;
      obj.material = ghostMat;
      obj.renderOrder = 2;
    });

    root.rotation.x = -Math.PI / 2;
    root.updateMatrixWorld(true);

    const box = new THREE.Box3().setFromObject(root);
    const center = new THREE.Vector3();
    box.getCenter(center);
    root.position.x -= center.x;
    root.position.z -= center.z;
    root.position.y -= box.min.y;

    const wrapper = new THREE.Group();
    wrapper.add(root);
    wrapper.visible = this.ghostVisible;

    this.ghostGroup = wrapper;
    this.ghostUrl = glbUrl;
    this.scene.add(wrapper);
  }

  clearGhostOverlay() {
    if (this.ghostGroup) {
      this.scene.remove(this.ghostGroup);
      this.ghostGroup.traverse((obj) => {
        if (!obj.isMesh) return;
        obj.geometry?.dispose?.();
        const mats = Array.isArray(obj.material) ? obj.material : [obj.material];
        mats.forEach((m) => m?.dispose?.());
      });
      this.ghostGroup = null;
    }
    this.ghostUrl = null;
    this.ghostVisible = false;
  }

  setCameraView(viewName = "iso", animate = true) {
    const r = this.modelRadius;
    const fovRad = THREE.MathUtils.degToRad(this.camera.fov);
    const fitDist = (r / Math.sin(fovRad / 2)) * 1.18;
    const target = this.modelCenter.clone();

    let dir = new THREE.Vector3(0.72, 0.52, 0.78);
    if (viewName === "top") {
      dir = new THREE.Vector3(0.001, 1, 0.001);
    } else if (viewName === "front") {
      dir = new THREE.Vector3(0, 0.08, 1);
    } else if (viewName === "side") {
      dir = new THREE.Vector3(1, 0.12, 0);
    }
    dir.normalize();

    const destPos = target.clone().addScaledVector(dir, fitDist);

    if (!animate) {
      this.camera.position.copy(destPos);
      this.controls.target.copy(target);
      this.controls.update();
      return;
    }

    this.camAnim = {
      startPos: this.camera.position.clone(),
      endPos: destPos,
      startTarget: this.controls.target.clone(),
      endTarget: target,
      t: 0,
    };
  }

  clearModelCache() {
    if (this.activeGroup) {
      this.scene.remove(this.activeGroup);
      this.activeGroup = null;
    }
    for (const group of this.modelCache.values()) {
      group.traverse((obj) => {
        if (!obj.isMesh) return;
        obj.geometry?.dispose?.();
        const origMats = Array.isArray(obj.userData.originalMaterial)
          ? obj.userData.originalMaterial
          : [obj.userData.originalMaterial];
        origMats.forEach((m) => m?.dispose?.());
        obj.userData.heatmapMaterial?.dispose?.();
      });
    }
    this.modelCache.clear();
  }

  _animate() {
    this._rafId = requestAnimationFrame(this._animate);

    if (this.camAnim) {
      this.camAnim.t = Math.min(this.camAnim.t + 0.08, 1);
      const ease = 1 - Math.pow(1 - this.camAnim.t, 3);
      this.camera.position.lerpVectors(this.camAnim.startPos, this.camAnim.endPos, ease);
      this.controls.target.lerpVectors(this.camAnim.startTarget, this.camAnim.endTarget, ease);
      if (this.camAnim.t >= 1) {
        this.camAnim = null;
      }
    }

    this.controls.update();
    this.renderer.render(this.scene, this.camera);
  }

  dispose() {
    cancelAnimationFrame(this._rafId);
    this.resizeObserver?.disconnect();
    this.clearGhostOverlay();
    this.clearModelCache();
    this.controls.dispose();
    this.renderer.dispose();
  }
}
