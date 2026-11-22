/**
 * main.js — Orquestador principal de RoboForge Studio
 */

import {
  analyzeStepFile,
  checkHealth,
  fetchMaterials,
  fetchPresets,
  generateDesign,
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
    this.abortController = null;
    this.toastTimer = null;

    // UI elements
    this.btnGenerate = document.getElementById("btn-generate");
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

    // Stress Heatmap toggle
    const btnHeatmap = document.getElementById("btn-heatmap");
    btnHeatmap?.addEventListener("click", () => {
      const next = !this.viewport.heatmapEnabled;
      const profile = this.activeVariant?.metrics?.stress_profile || null;
      this.viewport.setStressHeatmap(next, profile);
      btnHeatmap.classList.toggle("active", next);
      this._updateHeatmapLegend(next, this.activeVariant);
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

  _updateHeatmapLegend(enabled, variant) {
    if (!this.heatmapLegendEl) return;
    if (!enabled) {
      this.heatmapLegendEl.classList.add("hidden");
      return;
    }
    const maxStress = Number(variant?.metrics?.max_stress_mpa ?? 0);
    if (this.heatmapMidLabel) {
      this.heatmapMidLabel.textContent = `${(maxStress / 2).toFixed(1)}`;
    }
    if (this.heatmapMaxLabel) {
      this.heatmapMaxLabel.textContent = `${maxStress.toFixed(1)} MPa`;
    }
    this.heatmapLegendEl.classList.remove("hidden");
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

      // Limpiar caché de modelos de generaciones anteriores si el hash cambió
      this.viewport.clearModelCache();

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
      await this.viewport.loadModel(variant.glb_url, { autoFit });
      this._updateHeatmapLegend(this.viewport.heatmapEnabled, variant);
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
