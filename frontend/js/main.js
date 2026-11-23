/**
 * main.js — Orquestador principal de RoboForge Studio
 */

import {
  analyzeStepFile,
  checkHealth,
  createCustomMaterial,
  createCustomPrinter,
  deleteCustomMaterial,
  deleteCustomPrinter,
  fetchArmPresets,
  fetchGearPresets,
  fetchMaterials,
  fetchMountPresets,
  fetchPresets,
  fetchPrinters,
  generateArmAssembly,
  generateDesign,
  generateGears,
  generateGyroidSDF,
  generateMounts,
  runCopilotOptimize,
  runFEA,
  runGearFEA,
  runMountFEA,
} from "/js/api.js";
import { Viewport3D } from "/js/viewport.js";
import { ControlsController } from "/js/controls.js";
import { ComparatorController, VARIANT_COLORS } from "/js/comparator.js";

class RoboForgeApp {
  constructor() {
    this.viewport = null;
    this.controls = null;
    this.comparator = null;
    this.activeFamily = "links";
    this.currentResponse = null;
    this.activeVariant = null;
    this.isGenerating = false;
    this.isRunningFea = false;
    this.viewingFeaMesh = false;
    this.referenceGlbUrl = null;
    this.feaResultsByVariant = new Map();
    this.abortController = null;
    this.toastTimer = null;

    // UI elements
    this.btnGenerate = document.getElementById("btn-generate");
    this.btnGenerateLabel = document.getElementById("btn-generate-label");
    this.btnRunFea = document.getElementById("btn-run-fea");
    this.btnGhostStep = document.getElementById("btn-ghost-step");
    this.feaLoadDirEl = document.getElementById("fea-load-dir");
    this.feaDeformScaleEl = document.getElementById("fea-deform-scale");
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
        this.scheduleAutoGenerate(120);
      },
      onStepFileSelected: (file, validationErr) => this.handleStepUpload(file, validationErr),
      onClearStepReference: () => {
        this.referenceGlbUrl = null;
        this.viewport.clearGhostOverlay();
        this.btnGhostStep?.classList.add("hidden");
        this.btnGhostStep?.classList.remove("active");
        this.showToast("Referencia .STEP y fantasma eliminados.", false);
      },
    });

    this.autoGenTimer = null;
    this.showLoads3D = true;
    this._bindLiveAutoRegenerate();
    this._bindRealtimeArmFK();

    this.comparator = new ComparatorController({
      onSelectVariant: (variant) => this.selectVariant(variant, { autoFit: false }),
    });

    this._bindHeaderActions();
    this._bindProjectPersistence();
    this._bindViewportToolbar();
    this._bindProfileModal();
    this._bindCopilot();

    await this._bootstrapBackendData();
  }

  _bindLiveAutoRegenerate() {
    const sidebar = document.querySelector(".sidebar-left");
    if (!sidebar) return;

    const fkIds = new Set([
      "arm-shoulder_joint_deg",
      "arm-elbow_joint_deg",
      "arm-wrist_joint_deg",
    ]);

    sidebar.addEventListener("change", (e) => {
      const el = e.target;
      if (!el || !el.matches("input, select")) return;
      if (el.type === "file") return;
      if (el.id === "copilot-prompt-input") return;
      if (el.closest("#profile-modal")) return;
      // FK angle sliders animate at 60 FPS on the GPU without slow backend roundtrip
      if (fkIds.has(el.id) && this.activeFamily === "arm") return;
      this.scheduleAutoGenerate(420);
    });
  }

  _bindRealtimeArmFK() {
    const ids = ["arm-shoulder_joint_deg", "arm-elbow_joint_deg", "arm-wrist_joint_deg"];
    const onPoseInput = () => {
      if (this.activeFamily !== "arm") return;
      const sDeg = Number(document.getElementById("arm-shoulder_joint_deg")?.value || 0);
      const eDeg = Number(document.getElementById("arm-elbow_joint_deg")?.value || 0);
      const wDeg = Number(document.getElementById("arm-wrist_joint_deg")?.value || 0);
      this.viewport.setArmJointAngles(sDeg, eDeg, wDeg);

      // Update live TCP (FK) readout in right panel instantaneously
      const lBase = Number(document.getElementById("arm-base_height_mm")?.value || 95);
      const l1 = Number(document.getElementById("arm-upper_arm_mm")?.value || 200);
      const l2 = Number(document.getElementById("arm-forearm_mm")?.value || 170);
      const l3 = Number(document.getElementById("arm-wrist_mm")?.value || 75);
      const a1 = (sDeg * Math.PI) / 180;
      const a2 = ((sDeg + eDeg) * Math.PI) / 180;
      const a3 = ((sDeg + eDeg + wDeg) * Math.PI) / 180;
      const tcpX = l1 * Math.cos(a1) + l2 * Math.cos(a2) + l3 * Math.cos(a3);
      const tcpZ = lBase + l1 * Math.sin(a1) + l2 * Math.sin(a2) + l3 * Math.sin(a3);
      const tcpEl = document.querySelector(".arm-tcp-banner strong") || document.querySelector(".arm-tcp-coords");
      if (tcpEl) {
        tcpEl.textContent = `X ${tcpX.toFixed(1)} · Y 0.0 · Z ${tcpZ.toFixed(1)} mm`;
      }
      if (this.showLoads3D) {
        this._refreshEngineeringOverlays();
      }
    };
    ids.forEach((id) => {
      document.getElementById(id)?.addEventListener("input", onPoseInput);
    });
  }

  _refreshEngineeringOverlays() {
    if (!this.viewport) return;
    const armPayload = this.activeFamily === "arm" ? this.controls.getArmPayload() : null;
    const maxReach = armPayload
      ? armPayload.upper_arm_mm + armPayload.forearm_mm + armPayload.wrist_mm
      : 445;
    this.viewport.setEngineeringOverlays({
      family: this.activeFamily,
      visible: this.showLoads3D,
      loadDirection: this.feaLoadDirection || "y_inplane",
      maxReachMm: maxReach,
      baseHeightMm: armPayload?.base_height_mm || 95,
    });
  }

  scheduleAutoGenerate(delayMs = 420) {
    if (this.autoGenTimer) clearTimeout(this.autoGenTimer);
    this.autoGenTimer = setTimeout(() => {
      this.autoGenTimer = null;
      if (!this.isRunningFea) {
        this.runGeneration();
      }
    }, delayMs);
  }

  _bindHeaderActions() {
    this.btnGenerate?.addEventListener("click", () => this.runGeneration());

    document.querySelectorAll(".family-tab").forEach((tab) => {
      tab.addEventListener("click", () => {
        const nextFamily = tab.dataset.family;
        if (nextFamily && nextFamily !== this.activeFamily) {
          this.switchFamily(nextFamily);
        }
      });
    });

    window.addEventListener("keydown", (e) => {
      if ((e.ctrlKey || e.metaKey) && e.key === "Enter") {
        e.preventDefault();
        this.runGeneration();
      }
    });
  }

  _bindProjectPersistence() {
    const btnSave = document.getElementById("btn-save-project");
    const btnOpen = document.getElementById("btn-open-project");
    const fileInput = document.getElementById("project-file-input");

    btnSave?.addEventListener("click", () => {
      const project = {
        format: "roboforge-project",
        version: "2.0",
        saved_at: new Date().toISOString(),
        active_family: this.activeFamily,
        links: this.controls.getPayload(),
        mounts: this.controls.getMountPayload(),
        gears: this.controls.getGearPayload(),
        arm: this.controls.getArmPayload(),
      };
      const blob = new Blob([JSON.stringify(project, null, 2)], { type: "application/json" });
      const url = URL.createObjectURL(blob);
      const a = document.createElement("a");
      a.href = url;
      a.download = `roboforge_${this.activeFamily}_${Date.now()}.rforge`;
      a.click();
      URL.revokeObjectURL(url);
      this.showToast("Proyecto guardado en formato .rforge", false);
    });

    btnOpen?.addEventListener("click", () => fileInput?.click());
    fileInput?.addEventListener("change", async (e) => {
      const file = e.target.files?.[0];
      fileInput.value = "";
      if (!file) return;
      try {
        const text = await file.text();
        const data = JSON.parse(text);
        if (data.format !== "roboforge-project") {
          throw new Error("Archivo .rforge inválido");
        }
        if (data.links) this.controls.applyDesignParams(data.links);
        if (data.mounts) this.controls.applyMountPreset(data.mounts.actuator_type || "nema17");
        if (data.gears) this.controls.applyGearPreset(data.gears.preset || "spur_nema17");
        if (data.arm) this.controls.applyArmPreset(data.arm.preset || "parol6_desktop");
        const targetFam = data.active_family || "links";
        await this.switchFamily(targetFam);
        this.showToast(`Proyecto ${file.name} cargado correctamente.`, false);
      } catch (err) {
        this.showToast(`No se pudo abrir el proyecto: ${err.message}`, true);
      }
    });
  }

  async switchFamily(family) {
    if (this.isGenerating || this.isRunningFea) return;
    this.activeFamily = ["links", "mounts", "gears", "arm"].includes(family) ? family : "links";

    document.querySelectorAll(".family-tab").forEach((tab) => {
      const isActive = tab.dataset.family === this.activeFamily;
      tab.classList.toggle("active", isActive);
      tab.setAttribute("aria-selected", String(isActive));
    });

    this.controls.setFamily(this.activeFamily);

    const isArm = this.activeFamily === "arm";
    this.btnRunFea?.classList.toggle("hidden", isArm);
    document.getElementById("btn-heatmap")?.classList.toggle("hidden", isArm);
    if (isArm) {
      this.btnToggleFeaMesh?.classList.add("hidden");
      this.heatmapLegendEl?.classList.add("hidden");
    }

    if (this.btnGenerateLabel) {
      if (this.activeFamily === "mounts") {
        this.btnGenerateLabel.textContent = "Generar Monturas (3 Variantes)";
      } else if (this.activeFamily === "gears") {
        this.btnGenerateLabel.textContent = "Generar Transmisión (3 Variantes)";
      } else if (this.activeFamily === "arm") {
        this.btnGenerateLabel.textContent = "Generar Brazo Completo";
      } else {
        this.btnGenerateLabel.textContent = "Generar y Optimizar (5 Variantes)";
      }
    }

    await this.runGeneration();
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

    // Real 3D FEA (Gmsh + Tet4) trigger + toolbar selects sync
    this.feaLoadDirEl?.addEventListener("change", () => {
      this.feaLoadDirection = this.feaLoadDirEl.value;
    });
    this.feaDeformScaleEl?.addEventListener("change", () => {
      this.feaDeformScale = Number(this.feaDeformScaleEl.value || 0);
    });
    this.btnRunFea?.addEventListener("click", () => this.runFeaSimulation());

    // Toggle between Real 3D FEA mesh and CAD model
    this.btnToggleFeaMesh?.addEventListener("click", () => this.toggleFeaMeshView());

    // Field-Driven Implicit SDF + TPMS Gyroid (PicoGK / nTop style)
    const btnGyroid = document.getElementById("btn-gyroid-sdf");
    btnGyroid?.addEventListener("click", () => this.runGyroidSdfGeneration());

    // 3D Engineering Overlays (Load arrows + Clamp markers + Workspace Envelope)
    const btnLoads3d = document.getElementById("btn-loads-3d");
    btnLoads3d?.addEventListener("click", () => {
      this.showLoads3D = !this.showLoads3D;
      btnLoads3d.classList.toggle("active", this.showLoads3D);
      this._refreshEngineeringOverlays();
    });

    // Ghost Overlay .STEP toggle
    this.btnGhostStep?.addEventListener("click", async () => {
      if (!this.referenceGlbUrl) return;
      const next = !this.viewport.ghostVisible;
      try {
        await this.viewport.setGhostOverlay(this.referenceGlbUrl, next);
        this.btnGhostStep.classList.toggle("active", next);
      } catch (err) {
        console.error("Ghost overlay error:", err);
        this.showToast(`Error cargando fantasma .STEP: ${err.message}`, true);
      }
    });

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

  _bindProfileModal() {
    const modal = document.getElementById("profile-modal");
    const openBtn = document.getElementById("btn-open-profile-modal");
    const closeBtn = document.getElementById("btn-close-profile-modal");
    const formPrinter = document.getElementById("form-custom-printer");
    const formMaterial = document.getElementById("form-custom-material");
    const delPrinterBtn = document.getElementById("btn-del-printer");
    const delMaterialBtn = document.getElementById("btn-del-material");

    const closeModal = () => modal?.classList.add("hidden");
    openBtn?.addEventListener("click", () => modal?.classList.remove("hidden"));
    closeBtn?.addEventListener("click", closeModal);
    modal?.querySelectorAll("[data-close-modal]").forEach((b) => b.addEventListener("click", closeModal));
    modal?.addEventListener("click", (e) => {
      if (e.target === modal) closeModal();
    });

    // Tabs
    const tabBtns = modal?.querySelectorAll("[data-profile-tab]") || [];
    const panels = modal?.querySelectorAll("[data-profile-panel]") || [];
    tabBtns.forEach((btn) => {
      btn.addEventListener("click", () => {
        const target = btn.dataset.profileTab;
        tabBtns.forEach((b) => {
          const active = b.dataset.profileTab === target;
          b.classList.toggle("active", active);
          b.setAttribute("aria-selected", String(active));
        });
        panels.forEach((p) => p.classList.toggle("active", p.dataset.profilePanel === target));
      });
    });

    const slugify = (text, prefix) => {
      const base = String(text || "")
        .toLowerCase()
        .normalize("NFD")
        .replace(/[\u0300-\u036f]/g, "")
        .replace(/[^a-z0-9]+/g, "_")
        .replace(/^_+|_+$/g, "")
        .slice(0, 26);
      return base.length >= 2 ? base : `${prefix}_${Date.now().toString().slice(-4)}`;
    };

    formPrinter?.addEventListener("submit", async (e) => {
      e.preventDefault();
      const name = document.getElementById("cp-name")?.value?.trim();
      const payload = {
        id: slugify(name, "printer"),
        name,
        bed_x_mm: Number(document.getElementById("cp-bed-x")?.value || 256),
        bed_y_mm: Number(document.getElementById("cp-bed-y")?.value || 256),
        bed_z_mm: Number(document.getElementById("cp-bed-z")?.value || 256),
        default_nozzle_mm: Number(document.getElementById("cp-nozzle")?.value || 0.4),
        clearance_mm: Number(document.getElementById("cp-clearance")?.value || 0.15),
        max_bed_temp_c: Number(document.getElementById("cp-bed-temp")?.value || 110),
        enclosed: Boolean(document.getElementById("cp-enclosed")?.checked),
        notes: document.getElementById("cp-notes")?.value?.trim() || "",
      };
      try {
        const saved = await createCustomPrinter(payload);
        const printers = await fetchPrinters();
        this.controls.setPrinters(printers, saved?.id || payload.id);
        formPrinter.reset();
        closeModal();
        this.showToast(`Impresora "${name}" guardada.`, false);
      } catch (err) {
        this.showToast(`Error guardando impresora: ${err.message}`, true);
      }
    });

    formMaterial?.addEventListener("submit", async (e) => {
      e.preventDefault();
      const name = document.getElementById("cm-name")?.value?.trim();
      const payload = {
        id: slugify(name, "mat"),
        name,
        category: document.getElementById("cm-category")?.value || "fdm",
        density_g_cm3: Number(document.getElementById("cm-density")?.value || 1.25),
        youngs_xy_mpa: Number(document.getElementById("cm-exy")?.value || 5200),
        youngs_z_mpa: Number(document.getElementById("cm-ez")?.value || 3100),
        yield_xy_mpa: Number(document.getElementById("cm-yxy")?.value || 72),
        yield_z_mpa: Number(document.getElementById("cm-yz")?.value || 40),
        max_temp_c: Number(document.getElementById("cm-temp")?.value || 115),
        cost_per_kg_usd: Number(document.getElementById("cm-cost")?.value || 65),
        color_hex: document.getElementById("cm-color")?.value || "#38bdf8",
        notes: document.getElementById("cm-notes")?.value?.trim() || "",
      };
      try {
        const saved = await createCustomMaterial(payload);
        const materials = await fetchMaterials();
        this.controls.setMaterials(materials, saved?.id || payload.id);
        formMaterial.reset();
        closeModal();
        this.showToast(`Material "${name}" guardado.`, false);
      } catch (err) {
        this.showToast(`Error guardando material: ${err.message}`, true);
      }
    });

    delPrinterBtn?.addEventListener("click", async () => {
      const id = document.getElementById("printer_id")?.value;
      if (!id) return;
      try {
        await deleteCustomPrinter(id);
        const printers = await fetchPrinters();
        this.controls.setPrinters(printers, "bambu_x1c");
        this.showToast(`Impresora personalizada eliminada.`, false);
      } catch (err) {
        this.showToast(`Error eliminando impresora: ${err.message}`, true);
      }
    });

    delMaterialBtn?.addEventListener("click", async () => {
      const id = document.getElementById("material_id")?.value;
      if (!id) return;
      try {
        await deleteCustomMaterial(id);
        const materials = await fetchMaterials();
        this.controls.setMaterials(materials, "petg_cf");
        this.showToast(`Material personalizado eliminado.`, false);
      } catch (err) {
        this.showToast(`Error eliminando material: ${err.message}`, true);
      }
    });
  }

  _bindCopilot() {
    const input = document.getElementById("copilot-prompt");
    const btn = document.getElementById("btn-copilot-run");
    btn?.addEventListener("click", () => this.runCopilot());
    input?.addEventListener("keydown", (e) => {
      if (e.key === "Enter") {
        e.preventDefault();
        this.runCopilot();
      }
    });
  }

  async runCopilot() {
    const input = document.getElementById("copilot-prompt");
    const btn = document.getElementById("btn-copilot-run");
    const resultBox = document.getElementById("copilot-result");
    const badgeEl = document.getElementById("copilot-status-badge");
    const iterEl = document.getElementById("copilot-iter-count");
    const summaryEl = document.getElementById("copilot-summary");
    const chipsEl = document.getElementById("copilot-chips");

    const prompt = input?.value?.trim() || "";
    if (prompt.length < 4) {
      this.showToast("Describe tu objetivo (ej. 'Brazo de 240mm para 40N en PA12-CF').", true);
      return;
    }
    if (this.isGenerating || this.isRunningFea) return;

    if (btn) btn.disabled = true;
    this.setStatus("busy", "Copiloto optimizando...");
    this.showLoader(
      "Copiloto IA en Bucle Cerrado...",
      "Interpretando restricciones e iterando geometría óptima para mínima masa"
    );

    try {
      const res = await runCopilotOptimize({
        prompt,
        current_request: this.controls.getPayload(),
      });

      if (badgeEl) {
        badgeEl.textContent = res.converged ? "✓ Convergió" : "⚠ Mejor compromiso";
        badgeEl.className = `badge ${res.converged ? "badge-recommended" : "badge-supports-req"}`;
      }
      if (iterEl) {
        const n = (res.iterations || []).length;
        iterEl.textContent = `${n} iter.`;
      }
      if (summaryEl) {
        summaryEl.textContent = res.summary || "";
      }
      if (chipsEl) {
        chipsEl.innerHTML = "";
        for (const c of res.parsed_constraints || []) {
          const span = document.createElement("span");
          span.className = "copilot-chip";
          span.textContent = c;
          chipsEl.appendChild(span);
        }
      }
      resultBox?.classList.remove("hidden");

      if (res.optimized_request) {
        this.controls.applyDesignParams(res.optimized_request);
      }

      this.hideLoader();
      if (btn) btn.disabled = false;
      this.showToast("Copiloto completó optimización. Generando variantes 3D...", false);
      await this.runGeneration();
    } catch (err) {
      console.error("Copilot error:", err);
      this.setStatus("error", "Error Copiloto");
      this.showToast(`Error en Copiloto IA: ${err.message}`, true);
      this.hideLoader();
      if (btn) btn.disabled = false;
    }
  }

  async _bootstrapBackendData() {
    this.setStatus("busy", "Conectando...");
    try {
      const [health, materials, presets, mountPresets, gearPresets, printers, armPresets] =
        await Promise.all([
          checkHealth(),
          fetchMaterials(),
          fetchPresets(),
          fetchMountPresets().catch(() => null),
          fetchGearPresets().catch(() => null),
          fetchPrinters().catch(() => null),
          fetchArmPresets().catch(() => null),
        ]);

      if (materials?.list?.length) {
        this.controls.setMaterials(materials);
      }
      if (printers?.list?.length) {
        this.controls.setPrinters(printers);
      }
      if (presets) {
        this.controls.setPresets(presets);
      }
      if (mountPresets) {
        this.controls.setMountPresets(mountPresets);
      }
      if (gearPresets) {
        this.controls.setGearPresets(gearPresets);
      }
      if (armPresets) {
        this.controls.setArmPresets(armPresets);
      }

      // Aplicar presets iniciales para sincronizar UI de las 4 familias
      this.controls.applyArmPreset("parol6_desktop");
      this.controls.applyGearPreset("spur_nema17");
      this.controls.applyMountPreset("nema17");
      this.controls.applyArchitecturePreset("articulated_6dof");
      this.controls.setFamily("links");

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
      this.referenceGlbUrl = analysis.reference_glb_url || null;
      this.viewport.clearGhostOverlay();
      if (this.btnGhostStep) {
        this.btnGhostStep.classList.toggle("hidden", !this.referenceGlbUrl);
        this.btnGhostStep.classList.remove("active");
      }
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
          if (this.feaLoadDirEl) this.feaLoadDirEl.value = loadDirection;
          if (this.feaDeformScaleEl) this.feaDeformScaleEl.value = String(deformScale);
          this.runFeaSimulation();
        },
      }
    );

    this._updateHeatmapLegend();
  }

  async runGyroidSdfGeneration() {
    if (this.isGenerating || this.isRunningFea) return;
    if (this.activeFamily !== "links") {
      this.showToast("Gyroid SDF está disponible en la pestaña Eslabones Robóticos.", false);
      return;
    }

    this.isGenerating = true;
    this.setStatus("busy", "Sintetizando SDF Gyroid...");
    this.showLoader(
      "Generando Celosía Implícita SDF + TPMS Gyroid...",
      "Evaluando campo de distancia 3D guiado por esfuerzo + Marching Cubes (estilo PicoGK)"
    );

    try {
      const res = await generateGyroidSDF({
        request: this.controls.getPayload(),
        cell_size_mm: 14.0,
        shell_thickness_mm: 2.4,
        min_density: 0.28,
        max_density: 0.68,
        voxel_size_mm: 1.6,
      });

      await this.viewport.loadModel(res.glb_url, { autoFit: false, isFeaMesh: false, bypassCache: true });
      this.viewingFeaMesh = false;
      this._refreshEngineeringOverlays();

      if (this.hudDot) {
        this.hudDot.style.background = "#22d3a8";
        this.hudDot.style.boxShadow = "0 0 8px #22d3a8";
      }
      if (this.hudName) {
        this.hudName.textContent = "Celosía Implícita SDF Gyroid (TPMS)";
      }
      if (this.hudMeta) {
        this.hudMeta.textContent = `${Number(res.mass_g).toFixed(1)} g · ${res.voxel_grid.join("×")} vóxeles · ${Number(
          res.triangle_count
        ).toLocaleString("es")} tris (${Number(res.solve_time_s).toFixed(2)}s)`;
      }

      this.setStatus("ready", `Gyroid SDF listo (${Number(res.solve_time_s).toFixed(2)}s)`);
      this.showToast(
        `Gyroid SDF listo (${Number(res.mass_g).toFixed(1)} g). Descarga .3MF o .STL desde /api/files/${res.design_hash}/gyroid_sdf.3mf`,
        false
      );
    } catch (err) {
      this.setStatus("error", "Error Gyroid SDF");
      this.showToast(`Error Gyroid SDF: ${err.message}`, true);
    } finally {
      this.hideLoader();
      this.isGenerating = false;
    }
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
      const isMounts = this.activeFamily === "mounts";
      const isGears = this.activeFamily === "gears";
      const loadDirection = this.feaLoadDirection || "y_inplane";
      const deformScale = Number(this.feaDeformScale ?? 0);

      const payload = {
        design_hash: this.currentResponse.design_hash,
        variant_id: this.activeVariant.variant_id,
        request: isGears
          ? this.controls.getGearPayload()
          : isMounts
            ? this.controls.getMountPayload()
            : this.controls.getPayload(),
        mesh_size_mm: isGears ? 4.5 : isMounts ? 5.5 : 7.0,
        load_direction: loadDirection,
        deform_scale: deformScale,
      };

      const feaResult = isGears
        ? await runGearFEA(payload)
        : isMounts
          ? await runMountFEA(payload)
          : await runFEA(payload);
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

    const isMounts = this.activeFamily === "mounts";
    const isGears = this.activeFamily === "gears";
    const isArm = this.activeFamily === "arm";
    const payload = isArm
      ? this.controls.getArmPayload()
      : isGears
        ? this.controls.getGearPayload()
        : isMounts
          ? this.controls.getMountPayload()
          : this.controls.getPayload();

    this.btnGenerate.disabled = true;
    this.setStatus(
      "busy",
      isArm
        ? "Ensamblando Brazo..."
        : isGears
          ? "Generando Transmisión..."
          : isMounts
            ? "Generando Monturas..."
            : "Optimizando CAD + SIMP..."
    );
    this.showLoader(
      isArm
        ? "Sintetizando Ensamblaje 3D + URDF/MuJoCo + BOM..."
        : isGears
          ? "Sintetizando Engranajes / Poleas GT2 (build123d)..."
          : isMounts
            ? "Sintetizando Monturas Mecatrónicas (build123d)..."
            : "Sintetizando Geometría B-Rep + TopOpt SIMP...",
      isArm
        ? "Calculando cinemática directa (FK), torques en cascada, BOM y ensamblaje multi-cuerpo"
        : isGears
          ? "Generando 3 variantes (Sólida, Alma Rebajada, Radios Biónicos), mallas y cálculo de Lewis"
          : isMounts
            ? "Generando 3 variantes (Estándar, Cartelas, Aligerada), mallas GLB/STL/STEP y análisis estructural"
            : "Generando 5 variantes topológicas, mallas GLB/STL/STEP y perfil de esfuerzos"
    );

    try {
      if (isArm) {
        const armRes = await generateArmAssembly(payload, this.abortController.signal);
        this.currentResponse = armRes;
        this.activeVariant = null;
        this.viewport.clearModelCache();
        this.feaResultsByVariant.clear();
        this.viewingFeaMesh = false;
        this.comparator.renderFeaResult(null);
        this.btnToggleFeaMesh?.classList.add("hidden");

        this.comparator.renderArmAssembly(armRes);

        if (armRes.glb_url) {
          await this.viewport.loadModel(armRes.glb_url, { autoFit: true, isFeaMesh: false });
          this._refreshEngineeringOverlays();
        }
        if (this.hudDot) {
          this.hudDot.style.background = "#38bdf8";
          this.hudDot.style.boxShadow = "0 0 8px #38bdf8";
        }
        if (this.hudName) {
          this.hudName.textContent = `${armRes.preset_name || "Brazo Completo"} (${armRes.dof}-DOF)`;
        }
        if (this.hudMeta) {
          this.hudMeta.textContent = `Alcance ${Math.round(armRes.max_reach_mm)} mm · ${Number(
            armRes.total_arm_mass_kg
          ).toFixed(2)} kg · Costo $${Number(armRes.estimated_total_cost_usd).toFixed(0)}`;
        }

        this.setStatus("ready", `Brazo ${armRes.dof}-DOF listo`);
        this.showToast(`Ensamblaje #${armRes.design_hash.slice(0, 8)} generado con URDF, MuJoCo y BOM.`, false);
        return;
      }

      const response = isGears
        ? await generateGears(payload, this.abortController.signal)
        : isMounts
          ? await generateMounts(payload, this.abortController.signal)
          : await generateDesign(payload, this.abortController.signal);
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
        payload.max_deflection_mm ?? 0.25
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
      this._refreshEngineeringOverlays();
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
