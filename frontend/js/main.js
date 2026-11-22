/**
 * main.js — Orquestador principal de RoboForge Studio
 */

import {
  analyzeStepFile,
  checkHealth,
  fetchMaterials,
  fetchPresets,
  generateDesign,
  runFEA,
} from "/js/api.js";
import { Viewport3D } from "/js/viewport.js";
import { ControlsController } from "/js/controls.js";
import { ComparatorController, VARIANT_COLORS } from "/js/comparator.js";

class RoboForgeApp {
  constructor() {
    this.viewport = null;
    this.controls = null;
    this.comparator = null;
    this.currentResponse = null;
    this.activeVariant = null;
    this.isGenerating = false;
    this.isRunningFea = false;
    this.viewingFeaMesh = false;
    this.feaResultsByVariant = new Map();
    this.abortController = null;
    this.toastTimer = null;

    // UI elements
    this.btnGenerate = document.getElementById("btn-generate");
    this.btnRunFea = document.getElementById("btn-run-fea");
    this.btnToggleFeaMesh = document.getElementById("btn-toggle-fea-mesh");
    this.toggleFeaMeshLabel = document.getElementById("toggle-fea-mesh-label");
    this.statusIndicator = document.getElementById("status-indicator");
    this.statusText = document.getElementById("status-text");
    this.loaderEl = document.getElementById("viewport-loader");
    this.loaderTitle = document.getElementById("loader-title");
    this.loaderSubtitle = document.getElementById("loader-subtitle");
    this.hudDot = document.getElementById("hud-variant-dot");
    this.hudName = document.getElementById("hud-variant-name");
    this.hudMeta = document.getElementById("hud-variant-meta");
    this.toastEl = document.getElementById("app-toast");
    this.heatmapLegendEl = document.getElementById("heatmap-legend");
    this.heatmapLegendTitle = document.getElementById("heatmap-legend-title");
    this.heatmapMidLabel = document.getElementById("heatmap-mid-label");
    this.heatmapMaxLabel = document.getElementById("heatmap-max-label");
  }

  async init() {
    const canvasContainer = document.getElementById("viewport-canvas-container");
    this.viewport = new Viewport3D(canvasContainer);

    this.controls = new ControlsController({
      onArchitectureChange: () => {
        // El usuario puede ajustar parámetros adicionales o pulsar Generar
      },
      onStepFileSelected: (file, validationErr) => this.handleStepUpload(file, validationErr),
      onClearStepReference: () => {
        this.showToast("Referencia .STEP eliminada.", false);
      },
    });

    this.comparator = new ComparatorController({
      onSelectVariant: (variant) => this.selectVariant(variant, { autoFit: false }),
    });

    this._bindHeaderActions();
    this._bindViewportToolbar();

    await this._bootstrapBackendData();
  }

  _bindHeaderActions() {
    this.btnGenerate?.addEventListener("click", () => this.runGeneration());

    window.addEventListener("keydown", (e) => {
      if ((e.ctrlKey || e.metaKey) && e.key === "Enter") {
        e.preventDefault();
        this.runGeneration();
      }
    });
  }

  _bindViewportToolbar() {
    // Camera views
    const camButtons = document.querySelectorAll("[data-cam-view]");
    camButtons.forEach((btn) => {
      btn.addEventListener("click", () => {
        const view = btn.dataset.camView;
        camButtons.forEach((b) => b.classList.toggle("active", b === btn));
        this.viewport.setCameraView(view, true);
      });
    });

    // Fit camera
    document.getElementById("btn-fit-camera")?.addEventListener("click", () => {
      this.viewport.setCameraView("iso", true);
      camButtons.forEach((b) => b.classList.toggle("active", b.dataset.camView === "iso"));
    });

    // Real 3D FEA (Gmsh + Tet4) trigger
    this.btnRunFea?.addEventListener("click", () => this.runFeaSimulation());

    // Toggle between Real 3D FEA mesh and CAD model
    this.btnToggleFeaMesh?.addEventListener("click", () => this.toggleFeaMeshView());

    // Stress Heatmap toggle
    const btnHeatmap = document.getElementById("btn-heatmap");
    btnHeatmap?.addEventListener("click", async () => {
      if (this.viewingFeaMesh && this.activeVariant) {
        await this._switchToCadModel();
      }
      const next = !this.viewport.heatmapEnabled;
      const profile = this.activeVariant?.metrics?.stress_profile || null;
      this.viewport.setStressHeatmap(next, profile);
      btnHeatmap.classList.toggle("active", next);
      this._updateHeatmapLegend();
    });

    // Wireframe toggle
    const btnWire = document.getElementById("btn-wireframe");
    btnWire?.addEventListener("click", () => {
      const next = !this.viewport.wireframeEnabled;
      this.viewport.setWireframe(next);
      btnWire.classList.toggle("active", next);
    });

    // Auto-rotate toggle
    const btnRotate = document.getElementById("btn-autorotate");
    btnRotate?.addEventListener("click", () => {
      const next = !this.viewport.controls.autoRotate;
      this.viewport.setAutoRotate(next);
      btnRotate.classList.toggle("active", next);
    });

    // Section Cut X toggle + slider
    const btnSection = document.getElementById("btn-section-cut");
    const sliderSection = document.getElementById("section-cut-slider");
    const valSection = document.getElementById("val-section-cut");

    btnSection?.addEventListener("click", () => {
      const next = !this.viewport.sectionCutEnabled;
      btnSection.classList.toggle("active", next);
      if (sliderSection) sliderSection.disabled = !next;
      const ratio = Number(sliderSection?.value ?? 50) / 100;
      this.viewport.setSectionCut(next, ratio);
    });

    sliderSection?.addEventListener("input", () => {
      const pct = Number(sliderSection.value);
      if (valSection) valSection.textContent = `${pct}%`;
      this.viewport.setSectionCut(this.viewport.sectionCutEnabled, pct / 100);
    });
  }

  async _bootstrapBackendData() {
    this.setStatus("busy", "Conectando...");
    try {
      const [health, materials, presets] = await Promise.all([
        checkHealth(),
        fetchMaterials(),
        fetchPresets(),
      ]);

      if (materials?.list?.length) {
        this.controls.setMaterials(materials);
      }
      if (presets) {
        this.controls.setPresets(presets);
      }

      // Aplicar preset inicial para sincronizar UI
      this.controls.applyArchitecturePreset("articulated_6dof");

      const okLabel = health?.status ? "Listo · CAD Kernel" : "Listo";
      this.setStatus("ready", okLabel);

      // Generación automática inicial para mostrar el estudio con un modelo real
      await this.runGeneration();
    } catch (err) {
      console.error("Bootstrap error:", err);
      this.setStatus("error", "Sin conexión API");
      this.showToast(`No se pudo conectar al backend: ${err.message}`, true);
    }
  }

  async handleStepUpload(file, validationErr) {
    if (validationErr) {
      this.showToast(validationErr, true);
      return;
    }
    if (!file || this.isGenerating) return;

    this.controls.setStepUploadBusy(true);
    this.setStatus("busy", "Analizando .STEP...");
    this.showLoader(
      "Analizando pieza .STEP de referencia...",
      `Extrayendo volumen, caja envolvente y agujeros de ${file.name}`
    );

    try {
      const analysis = await analyzeStepFile(file);
      this.controls.setStepReference(analysis);
      this.showToast(
        `Pieza ${analysis.filename} analizada (${Number(analysis.volume_cm3).toFixed(1)} cm³). Generando 5 variantes...`,
        false
      );
      this.hideLoader();
      this.controls.setStepUploadBusy(false);
      await this.runGeneration();
    } catch (err) {
      console.error("STEP analyze error:", err);
      this.setStatus("error", "Error .STEP");
      this.showToast(`Error analizando .STEP: ${err.message}`, true);
      this.hideLoader();
      this.controls.setStepUploadBusy(false);
    }
  }

  _updateHeatmapLegend() {
    if (!this.heatmapLegendEl) return;

    const feaResult = this.activeVariant
      ? this.feaResultsByVariant.get(this.activeVariant.variant_id)
      : null;

    if (this.viewingFeaMesh && feaResult) {
      if (this.heatmapLegendTitle) {
        this.heatmapLegendTitle.textContent = "Von Mises FEA 3D (MPa)";
      }
      if (this.heatmapMidLabel) {
        this.heatmapMidLabel.textContent = `P95: ${Number(feaResult.p95_von_mises_mpa ?? 0).toFixed(1)}`;
      }
      if (this.heatmapMaxLabel) {
        this.heatmapMaxLabel.textContent = `Pico: ${Number(feaResult.max_von_mises_mpa ?? 0).toFixed(1)} MPa`;
      }
      this.heatmapLegendEl.classList.remove("hidden");
      return;
    }

    if (this.viewport.heatmapEnabled && this.activeVariant) {
      const maxStress = Number(this.activeVariant?.metrics?.max_stress_mpa ?? 0);
      if (this.heatmapLegendTitle) {
        this.heatmapLegendTitle.textContent = "Esfuerzo Analítico (MPa)";
      }
      if (this.heatmapMidLabel) {
        this.heatmapMidLabel.textContent = `${(maxStress / 2).toFixed(1)}`;
      }
      if (this.heatmapMaxLabel) {
        this.heatmapMaxLabel.textContent = `${maxStress.toFixed(1)} MPa`;
      }
      this.heatmapLegendEl.classList.remove("hidden");
      return;
    }

    this.heatmapLegendEl.classList.add("hidden");
  }

  _syncFeaUiForActiveVariant() {
    const feaResult = this.activeVariant
      ? this.feaResultsByVariant.get(this.activeVariant.variant_id)
      : null;

    if (this.btnToggleFeaMesh) {
      this.btnToggleFeaMesh.classList.toggle("hidden", !feaResult);
      this.btnToggleFeaMesh.classList.toggle("active", Boolean(feaResult && this.viewingFeaMesh));
    }
    if (this.toggleFeaMeshLabel) {
      this.toggleFeaMeshLabel.textContent = this.viewingFeaMesh ? "Malla FEA" : "Modelo CAD";
    }

    this.comparator.renderFeaResult(
      feaResult || null,
      this.activeVariant?.name,
      this.viewingFeaMesh,
      {
        onToggleMesh: () => this.toggleFeaMeshView(),
        loadDirection: this.feaLoadDirection || "y_inplane",
        deformScale: this.feaDeformScale || 0,
        onChangeOptions: ({ loadDirection, deformScale }) => {
          this.feaLoadDirection = loadDirection;
          this.feaDeformScale = deformScale;
          this.runFeaSimulation();
        },
      }
    );

    this._updateHeatmapLegend();
  }

  async runFeaSimulation() {
    if (this.isRunningFea || this.isGenerating) return;
    if (!this.currentResponse || !this.activeVariant) {
      this.showToast("Genera un diseño antes de ejecutar FEA 3D.", true);
      return;
    }

    this.isRunningFea = true;
    if (this.btnRunFea) this.btnRunFea.disabled = true;
    this.setStatus("busy", "Resolviendo FEA 3D...");
    this.showLoader(
      "Ejecutando Solver FEA 3D (Gmsh + Tet4)...",
      `Mallando sólido .STEP de ${this.activeVariant.name} y resolviendo elasticidad lineal 3D`
    );

    try {
      const payload = {
        design_hash: this.currentResponse.design_hash,
        variant_id: this.activeVariant.variant_id,
        request: this.controls.getPayload(),
        mesh_size_mm: 7.0,
        load_direction: this.feaLoadDirection || "y_inplane",
        deform_scale: this.feaDeformScale || 0,
      };

      const feaResult = await runFEA(payload);
      this.feaResultsByVariant.set(this.activeVariant.variant_id, feaResult);

      // Cargar malla FEA 3D con vertexColors reales
      await this.viewport.loadModel(feaResult.fea_glb_url, {
        autoFit: false,
        isFeaMesh: true,
        bypassCache: true,
      });
      this.viewingFeaMesh = true;

      // Desactivar botón de mapa analítico si estaba activo para evitar confusión
      const btnHeatmap = document.getElementById("btn-heatmap");
      btnHeatmap?.classList.remove("active");
      this.viewport.heatmapEnabled = false;

      if (this.hudMeta) {
        this.hudMeta.textContent = `FEA 3D: P95 ${Number(feaResult.p95_von_mises_mpa).toFixed(
          1
        )} MPa · Pico ${Number(feaResult.max_von_mises_mpa).toFixed(1)} MPa · FS ${Number(
          feaResult.fea_safety_factor
        ).toFixed(2)}×`;
      }

      this._syncFeaUiForActiveVariant();
      this.setStatus("ready", `FEA 3D listo (${Number(feaResult.solve_time_s).toFixed(2)}s)`);
      this.showToast(
        `FEA 3D completado (${feaResult.element_count.toLocaleString("es")} tetraedros, FS ${Number(
          feaResult.fea_safety_factor
        ).toFixed(2)}×).`,
        false
      );
    } catch (err) {
      console.error("FEA 3D error:", err);
      this.setStatus("error", "Error FEA 3D");
      this.showToast(`Error en FEA 3D: ${err.message}`, true);
    } finally {
      this.hideLoader();
      if (this.btnRunFea) this.btnRunFea.disabled = false;
      this.isRunningFea = false;
    }
  }

  async _switchToCadModel() {
    if (!this.activeVariant) return;
    this.viewingFeaMesh = false;
    await this.viewport.loadModel(this.activeVariant.glb_url, {
      autoFit: false,
      isFeaMesh: false,
    });
    const m = this.activeVariant.metrics || {};
    if (this.hudMeta) {
      this.hudMeta.textContent = `${Number(m.mass_g ?? 0).toFixed(1)} g · FS ${Number(
        m.safety_factor ?? 0
      ).toFixed(2)}× · Deflex. ${Number(m.tip_deflection_mm ?? 0).toFixed(2)} mm`;
    }
    this._syncFeaUiForActiveVariant();
  }

  async toggleFeaMeshView() {
    if (!this.activeVariant) return;
    const feaResult = this.feaResultsByVariant.get(this.activeVariant.variant_id);
    if (!feaResult) return;

    if (this.viewingFeaMesh) {
      await this._switchToCadModel();
    } else {
      this.viewingFeaMesh = true;
      await this.viewport.loadModel(feaResult.fea_glb_url, {
        autoFit: false,
        isFeaMesh: true,
      });
      if (this.hudMeta) {
        this.hudMeta.textContent = `FEA 3D: P95 ${Number(feaResult.p95_von_mises_mpa).toFixed(
          1
        )} MPa · Pico ${Number(feaResult.max_von_mises_mpa).toFixed(1)} MPa · FS ${Number(
          feaResult.fea_safety_factor
        ).toFixed(2)}×`;
      }
      this._syncFeaUiForActiveVariant();
    }
  }

  async runGeneration() {
    if (this.isGenerating) return;
    this.isGenerating = true;

    if (this.abortController) {
      this.abortController.abort();
    }
    this.abortController = new AbortController();

    const payload = this.controls.getPayload();

    this.btnGenerate.disabled = true;
    this.setStatus("busy", "Optimizando CAD + SIMP...");
    this.showLoader(
      "Sintetizando Geometría B-Rep + TopOpt SIMP...",
      "Generando 5 variantes topológicas, mallas GLB/STL/STEP y perfil de esfuerzos"
    );

    try {
      const response = await generateDesign(payload, this.abortController.signal);
      this.currentResponse = response;

      // Limpiar caché de modelos y FEA de generaciones anteriores
      this.viewport.clearModelCache();
      this.feaResultsByVariant.clear();
      this.viewingFeaMesh = false;
      this.comparator.renderFeaResult(null);
      this.btnToggleFeaMesh?.classList.add("hidden");

      const recommended =
        response.variants?.find((v) => v.variant_id === response.recommended_variant) ||
        response.variants?.[0];

      this.comparator.render(
        response,
        recommended?.variant_id,
        payload.target_safety_factor,
        payload.max_deflection_mm
      );

      if (recommended) {
        await this.selectVariant(recommended, { autoFit: true });
      }

      const count = response.variants?.length || 5;
      this.setStatus("ready", `${count} variantes listas`);
      this.showToast(`Diseño #${response.design_hash.slice(0, 8)} generado (${count} variantes).`, false);
    } catch (err) {
      if (err.name === "AbortError") return;
      console.error("Generation error:", err);
      this.setStatus("error", "Error de cálculo");
      this.showToast(`Error al generar: ${err.message}`, true);
    } finally {
      this.hideLoader();
      this.btnGenerate.disabled = false;
      this.isGenerating = false;
    }
  }

  async selectVariant(variant, { autoFit = false } = {}) {
    if (!variant?.glb_url) return;
    this.activeVariant = variant;
    this.viewingFeaMesh = false;

    const accent = VARIANT_COLORS[variant.variant_id] || "#38bdf8";
    if (this.hudDot) {
      this.hudDot.style.background = accent;
      this.hudDot.style.boxShadow = `0 0 8px ${accent}`;
    }
    if (this.hudName) {
      this.hudName.textContent = variant.name || variant.variant_id;
    }

    const m = variant.metrics || {};
    if (this.hudMeta) {
      this.hudMeta.textContent = `${Number(m.mass_g ?? 0).toFixed(1)} g · FS ${Number(
        m.safety_factor ?? 0
      ).toFixed(2)}× · Deflex. ${Number(m.tip_deflection_mm ?? 0).toFixed(2)} mm`;
    }

    // Mostrar loader solo si no está ya en caché
    const isCached = this.viewport.modelCache.has(variant.glb_url);
    if (!isCached && !this.isGenerating) {
      this.showLoader("Cargando malla 3D...", variant.name || variant.variant_id);
    }

    try {
      this.viewport.stressProfile = variant.metrics?.stress_profile || null;
      await this.viewport.loadModel(variant.glb_url, { autoFit, isFeaMesh: false });
      this._syncFeaUiForActiveVariant();
    } catch (err) {
      console.error("GLB load error:", err);
      this.showToast(`Error cargando modelo 3D: ${err.message}`, true);
    } finally {
      if (!this.isGenerating) {
        this.hideLoader();
      }
    }
  }

  setStatus(state, text) {
    if (this.statusIndicator) this.statusIndicator.dataset.state = state;
    if (this.statusText) this.statusText.textContent = text;
  }

  showLoader(title, subtitle) {
    if (this.loaderTitle) this.loaderTitle.textContent = title;
    if (this.loaderSubtitle) this.loaderSubtitle.textContent = subtitle;
    this.loaderEl?.classList.remove("hidden");
  }

  hideLoader() {
    this.loaderEl?.classList.add("hidden");
  }

  showToast(message, isError = false) {
    if (!this.toastEl) return;
    this.toastEl.textContent = message;
    this.toastEl.classList.toggle("error", isError);
    this.toastEl.classList.remove("hidden");
    clearTimeout(this.toastTimer);
    this.toastTimer = setTimeout(() => {
      this.toastEl.classList.add("hidden");
    }, 4200);
  }
}

document.addEventListener("DOMContentLoaded", () => {
  const app = new RoboForgeApp();
  app.init();
});
