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
  async loadModel(glbUrl, { autoFit = true } = {}) {
    const token = ++this.loadToken;

    let cached = this.modelCache.get(glbUrl);
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

      const wrapper = this._prepareLoadedScene(gltf.scene);
      this.modelCache.set(glbUrl, wrapper);
      cached = wrapper;
    }

    if (token !== this.loadToken) return null;

    if (this.activeGroup) {
      this.scene.remove(this.activeGroup);
    }

    this.activeGroup = cached;
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
  _prepareLoadedScene(root) {
    const group = new THREE.Group();

    // Asegurar normales suaves/limpias y propiedades PBR industriales
    root.traverse((obj) => {
      if (!obj.isMesh) return;
      if (obj.geometry && !obj.geometry.attributes.normal) {
        obj.geometry.computeVertexNormals();
      }
      const materials = Array.isArray(obj.material) ? obj.material : [obj.material];
      const enhanced = materials.map((mat) => {
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
    });

    // Calcular caja original y reorientar si el eslabón viene acostado en XY (build123d extruye en Z)
    const rawBox = new THREE.Box3().setFromObject(root);
    const rawSize = new THREE.Vector3();
    rawBox.getSize(rawSize);

    // En build123d la longitud suele estar en X, ancho en Y, espesor en Z.
    // Rotar -90° en X coloca el plano XY de impresión sobre el plano horizontal XZ de Three.js (Y hacia arriba).
    if (rawSize.z > 0 && rawSize.y >= rawSize.z * 0.85) {
      root.rotation.x = -Math.PI / 2;
      root.updateMatrixWorld(true);
    }

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

  _applyMaterialStates() {
    if (!this.activeGroup) return;

    const minX = this.modelBounds.min.x;
    const maxX = this.modelBounds.max.x;
    const pad = 2.0;
    const cutX = minX - pad + (maxX - minX + pad * 2) * this.sectionCutRatio;
    this.clipPlaneX.constant = cutX;

    const activePlanes = this.sectionCutEnabled ? [this.clipPlaneX] : [];

    this.activeGroup.traverse((obj) => {
      if (!obj.isMesh) return;
      const mats = Array.isArray(obj.material) ? obj.material : [obj.material];
      for (const m of mats) {
        m.wireframe = this.wireframeEnabled;
        m.clippingPlanes = activePlanes;
        m.clipShadows = this.sectionCutEnabled;
        m.side = this.sectionCutEnabled ? THREE.DoubleSide : THREE.FrontSide;
        m.needsUpdate = true;
      }
    });
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
        const mats = Array.isArray(obj.material) ? obj.material : [obj.material];
        mats.forEach((m) => m?.dispose?.());
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
    this.clearModelCache();
    this.controls.dispose();
    this.renderer.dispose();
  }
}
