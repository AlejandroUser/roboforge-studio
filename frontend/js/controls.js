/**
 * controls.js — Gestión del panel izquierdo de parámetros de RoboForge Studio
 */

const ARCH_META = {
  articulated_6dof: {
    label: "Articulado 6-DOF",
    subtitle: "Brazo industrial multi-eje",
    icon: `<svg viewBox="0 0 24 24" fill="none"><circle cx="6" cy="18" r="3" stroke="currentColor" stroke-width="1.8"/><circle cx="13" cy="10" r="2.5" stroke="currentColor" stroke-width="1.8"/><circle cx="19" cy="6" r="2" stroke="currentColor" stroke-width="1.8"/><path d="M8 15.8L11.3 12M15 8.6L17.2 7.2" stroke="currentColor" stroke-width="1.8" stroke-linecap="round"/></svg>`,
  },
  scara: {
    label: "SCARA",
    subtitle: "Alta velocidad en plano XY",
    icon: `<svg viewBox="0 0 24 24" fill="none"><circle cx="6" cy="12" r="3.2" stroke="currentColor" stroke-width="1.8"/><circle cx="18" cy="12" r="2.4" stroke="currentColor" stroke-width="1.8"/><path d="M9.2 10.5L15.8 10.8M9.2 13.5L15.8 13.2" stroke="currentColor" stroke-width="1.7" stroke-linecap="round"/></svg>`,
  },
  delta_rod: {
    label: "Biela Delta",
    subtitle: "Ultra-ligera, baja inercia",
    icon: `<svg viewBox="0 0 24 24" fill="none"><circle cx="5" cy="12" r="2.2" stroke="currentColor" stroke-width="1.8"/><circle cx="19" cy="12" r="2.2" stroke="currentColor" stroke-width="1.8"/><path d="M7.2 12H16.8" stroke="currentColor" stroke-width="2" stroke-linecap="round"/><path d="M10 9.5L14 14.5M14 9.5L10 14.5" stroke="currentColor" stroke-width="1.2" stroke-opacity="0.6"/></svg>`,
  },
  robot_leg: {
    label: "Pata Robótica",
    subtitle: "Cuadrúpedo / alto impacto",
    icon: `<svg viewBox="0 0 24 24" fill="none"><circle cx="7" cy="6" r="2.6" stroke="currentColor" stroke-width="1.8"/><circle cx="15" cy="13" r="2.2" stroke="currentColor" stroke-width="1.8"/><path d="M8.8 8L13.5 11.6M14.2 15.1L11.5 20.5" stroke="currentColor" stroke-width="1.9" stroke-linecap="round"/></svg>`,
  },
};

const FALLBACK_END_PRESETS = {
  nema17: {
    label: "NEMA 17",
    outer_diameter_mm: 46.0,
    bore_diameter_mm: 22.0,
    bolt_count: 4,
    bolt_pcd_mm: 31.0,
    bolt_size: "M3",
    use_heat_inserts: true,
  },
  nema23: {
    label: "NEMA 23",
    outer_diameter_mm: 62.0,
    bore_diameter_mm: 38.1,
    bolt_count: 4,
    bolt_pcd_mm: 47.14,
    bolt_size: "M5",
    use_heat_inserts: true,
  },
  bearing_608: {
    label: "Rod. 608",
    outer_diameter_mm: 32.0,
    bore_diameter_mm: 22.0,
    bolt_count: 0,
    bolt_pcd_mm: 26.0,
    bolt_size: "M3",
    use_heat_inserts: false,
  },
  bearing_6805: {
    label: "Rod. 6805",
    outer_diameter_mm: 48.0,
    bore_diameter_mm: 37.0,
    bolt_count: 0,
    bolt_pcd_mm: 42.0,
    bolt_size: "M3",
    use_heat_inserts: false,
  },
  custom: {
    label: "Custom",
  },
};

const FALLBACK_ARCH_DEFAULTS = {
  articulated_6dof: {
    architecture: "articulated_6dof",
    length_mm: 180,
    root_width_mm: 38,
    tip_width_mm: 24,
    thickness_mm: 20,
    organic_taper: 0.75,
    cable_channel: true,
    root_end: { ...FALLBACK_END_PRESETS.nema17, preset: "nema17" },
    tip_end: { ...FALLBACK_END_PRESETS.bearing_608, preset: "bearing_608" },
    tip_load_n: 25,
    angular_accel_rad_s2: 15,
    target_safety_factor: 2.0,
    max_deflection_mm: 0.5,
    operating_temp_c: 45,
    material_id: "petg_cf",
    print_orientation: "xy_flat",
    nozzle_diameter_mm: 0.4,
    printer_clearance_mm: 0.15,
  },
  scara: {
    architecture: "scara",
    length_mm: 220,
    root_width_mm: 52,
    tip_width_mm: 34,
    thickness_mm: 24,
    organic_taper: 0.65,
    cable_channel: true,
    root_end: { ...FALLBACK_END_PRESETS.nema23, preset: "nema23" },
    tip_end: { ...FALLBACK_END_PRESETS.bearing_6805, preset: "bearing_6805" },
    tip_load_n: 35,
    angular_accel_rad_s2: 30,
    target_safety_factor: 2.2,
    max_deflection_mm: 0.35,
    operating_temp_c: 50,
    material_id: "pa12_cf",
    print_orientation: "xy_flat",
    nozzle_diameter_mm: 0.4,
    printer_clearance_mm: 0.15,
  },
  delta_rod: {
    architecture: "delta_rod",
    length_mm: 240,
    root_width_mm: 22,
    tip_width_mm: 20,
    thickness_mm: 12,
    organic_taper: 0.85,
    cable_channel: false,
    root_end: { ...FALLBACK_END_PRESETS.bearing_608, preset: "bearing_608" },
    tip_end: { ...FALLBACK_END_PRESETS.bearing_608, preset: "bearing_608" },
    tip_load_n: 12,
    angular_accel_rad_s2: 60,
    target_safety_factor: 2.0,
    max_deflection_mm: 0.4,
    operating_temp_c: 35,
    material_id: "petg_cf",
    print_orientation: "xy_flat",
    nozzle_diameter_mm: 0.4,
    printer_clearance_mm: 0.15,
  },
  robot_leg: {
    architecture: "robot_leg",
    length_mm: 200,
    root_width_mm: 46,
    tip_width_mm: 26,
    thickness_mm: 22,
    organic_taper: 0.8,
    cable_channel: true,
    root_end: { ...FALLBACK_END_PRESETS.nema23, preset: "nema23" },
    tip_end: { ...FALLBACK_END_PRESETS.bearing_608, preset: "bearing_608" },
    tip_load_n: 80,
    angular_accel_rad_s2: 45,
    target_safety_factor: 2.5,
    max_deflection_mm: 0.8,
    operating_temp_c: 55,
    material_id: "pa12_cf",
    print_orientation: "xy_flat",
    nozzle_diameter_mm: 0.6,
    printer_clearance_mm: 0.15,
  },
};

function clamp(val, min, max, fallback) {
  const n = Number(val);
  if (!Number.isFinite(n)) return fallback;
  return Math.min(Math.max(n, min), max);
}

export class ControlsController {
  constructor({ onArchitectureChange, onStepFileSelected, onClearStepReference } = {}) {
    this.onArchitectureChange = onArchitectureChange || (() => {});
    this.onStepFileSelected = onStepFileSelected || (() => {});
    this.onClearStepReference = onClearStepReference || (() => {});
    this.materialsMap = {};
    this.archPresets = structuredClone(FALLBACK_ARCH_DEFAULTS);
    this.endPresets = structuredClone(FALLBACK_END_PRESETS);
    this.currentArchitecture = "articulated_6dof";
    this.currentOrientation = "xy_flat";
    this.referenceVolumeCm3 = null;

    this._bindStepUpload();
    this._bindSliders();
    this._bindEndTabs();
    this._bindEndControls("root");
    this._bindEndControls("tip");
    this._bindOrientationPills();
    this._bindMaterialSelect();
    this._renderArchitectureGrid();
  }

  _bindStepUpload() {
    const dropzone = document.getElementById("step-dropzone");
    const fileInput = document.getElementById("step-file-input");
    const clearBtn = document.getElementById("btn-clear-step");

    const triggerFile = (file) => {
      if (!file) return;
      const lower = file.name.toLowerCase();
      if (!lower.endsWith(".step") && !lower.endsWith(".stp")) {
        this.onStepFileSelected(null, "Solo se admiten archivos .STEP o .STP.");
        return;
      }
      this.onStepFileSelected(file, null);
    };

    dropzone?.addEventListener("click", () => fileInput?.click());
    dropzone?.addEventListener("keydown", (e) => {
      if (e.key === "Enter" || e.key === " ") {
        e.preventDefault();
        fileInput?.click();
      }
    });

    fileInput?.addEventListener("change", () => {
      const file = fileInput.files?.[0];
      if (file) triggerFile(file);
      fileInput.value = "";
    });

    dropzone?.addEventListener("dragover", (e) => {
      e.preventDefault();
      dropzone.classList.add("dragover");
    });
    dropzone?.addEventListener("dragleave", () => {
      dropzone.classList.remove("dragover");
    });
    dropzone?.addEventListener("drop", (e) => {
      e.preventDefault();
      dropzone.classList.remove("dragover");
      const file = e.dataTransfer?.files?.[0];
      if (file) triggerFile(file);
    });

    clearBtn?.addEventListener("click", () => {
      this.clearStepReference();
      this.onClearStepReference();
    });
  }

  setStepUploadBusy(busy) {
    const dropzone = document.getElementById("step-dropzone");
    dropzone?.classList.toggle("busy", Boolean(busy));
  }

  setStepReference(analysis) {
    const card = document.getElementById("step-summary-card");
    const nameEl = document.getElementById("step-filename");
    const bboxEl = document.getElementById("step-bbox");
    const volEl = document.getElementById("step-volume");
    const holesEl = document.getElementById("step-holes");
    const notesEl = document.getElementById("step-notes");

    if (!analysis) {
      this.clearStepReference();
      return;
    }

    const refVol = analysis.suggested_request?.reference_volume_cm3 ?? analysis.volume_cm3;
    this.referenceVolumeCm3 = Number.isFinite(Number(refVol)) && Number(refVol) > 0 ? Number(refVol) : null;

    if (nameEl) nameEl.textContent = analysis.filename || "referencia.step";
    if (bboxEl) {
      bboxEl.textContent = `${Math.round(analysis.bbox_length_mm)}×${Math.round(
        analysis.bbox_width_mm
      )}×${Math.round(analysis.bbox_thickness_mm)} mm`;
    }
    if (volEl) volEl.textContent = `${Number(analysis.volume_cm3 || 0).toFixed(1)} cm³`;
    if (holesEl) {
      const holes = analysis.detected_holes || [];
      if (holes.length) {
        const diams = holes
          .slice(0, 2)
          .map((h) => `Ø${Number(h.diameter_mm).toFixed(1)}`)
          .join(", ");
        holesEl.textContent = `${holes.length} (${diams})`;
      } else {
        holesEl.textContent = "0";
      }
    }
    if (notesEl) {
      notesEl.innerHTML = "";
      for (const note of analysis.notes || []) {
        const li = document.createElement("li");
        li.textContent = note;
        notesEl.appendChild(li);
      }
    }

    card?.classList.remove("hidden");

    if (analysis.suggested_request) {
      this.applyDesignParams(analysis.suggested_request);
    }
  }

  clearStepReference() {
    this.referenceVolumeCm3 = null;
    const card = document.getElementById("step-summary-card");
    card?.classList.add("hidden");
  }

  _bindSliders() {
    const sliderConfigs = [
      { id: "length_mm", suffix: " mm", decimals: 0 },
      { id: "root_width_mm", suffix: " mm", decimals: 0 },
      { id: "tip_width_mm", suffix: " mm", decimals: 0 },
      { id: "thickness_mm", suffix: " mm", decimals: 0 },
      { id: "organic_taper", suffix: "%", decimals: 0 },
      { id: "topopt_volume_fraction", suffix: "%", decimals: 0 },
      { id: "printer_clearance_mm", suffix: " mm", decimals: 2 },
    ];

    for (const cfg of sliderConfigs) {
      const input = document.getElementById(cfg.id);
      const readout = document.getElementById(`val-${cfg.id}`);
      if (!input || !readout) continue;
      const update = () => {
        const num = Number(input.value);
        readout.textContent = `${num.toFixed(cfg.decimals)}${cfg.suffix}`;
      };
      input.addEventListener("input", update);
      update();
    }
  }

  _bindEndTabs() {
    const tabBtns = document.querySelectorAll("[data-end-tab]");
    const panels = document.querySelectorAll("[data-end-panel]");
    tabBtns.forEach((btn) => {
      btn.addEventListener("click", () => {
        const target = btn.dataset.endTab;
        tabBtns.forEach((b) => {
          const active = b.dataset.endTab === target;
          b.classList.toggle("active", active);
          b.setAttribute("aria-selected", String(active));
        });
        panels.forEach((p) => {
          p.classList.toggle("active", p.dataset.endPanel === target);
        });
      });
    });
  }

  _bindEndControls(side) {
    const presetSel = document.getElementById(`${side}-preset`);
    const manualFields = [
      "outer_diameter_mm",
      "bore_diameter_mm",
      "bolt_count",
      "bolt_pcd_mm",
      "bolt_size",
      "use_heat_inserts",
    ];

    presetSel?.addEventListener("change", () => {
      const presetKey = presetSel.value;
      const presetData = this.endPresets[presetKey];
      if (presetKey !== "custom" && presetData) {
        this._setEndFormValues(side, { preset: presetKey, ...presetData });
      }
      this._updateEndTabSummary(side);
    });

    for (const field of manualFields) {
      const el = document.getElementById(`${side}-${field}`);
      if (!el) continue;
      el.addEventListener("input", () => {
        if (presetSel && presetSel.value !== "custom") {
          presetSel.value = "custom";
        }
        this._updateEndTabSummary(side);
      });
      el.addEventListener("change", () => {
        this._updateEndTabSummary(side);
      });
    }
    this._updateEndTabSummary(side);
  }

  _bindOrientationPills() {
    const pills = document.querySelectorAll("[data-orientation]");
    pills.forEach((pill) => {
      pill.addEventListener("click", () => {
        this.currentOrientation = pill.dataset.orientation;
        pills.forEach((p) => p.classList.toggle("active", p === pill));
      });
    });
  }

  _bindMaterialSelect() {
    const sel = document.getElementById("material_id");
    sel?.addEventListener("change", () => {
      this._renderMaterialBadge(sel.value);
    });
  }

  _renderArchitectureGrid() {
    const grid = document.getElementById("arch-preset-grid");
    if (!grid) return;
    grid.innerHTML = "";

    for (const [archId, meta] of Object.entries(ARCH_META)) {
      const backendPreset = this.archPresets[archId];
      const title = backendPreset?.name || meta.label;
      const subtitle = backendPreset?.description || meta.subtitle;

      const btn = document.createElement("button");
      btn.type = "button";
      btn.className = `arch-card${archId === this.currentArchitecture ? " active" : ""}`;
      btn.dataset.archId = archId;
      btn.innerHTML = `
        <div class="arch-card-head">
          ${meta.icon}
        </div>
        <span class="arch-card-title">${title}</span>
        <span class="arch-card-sub">${subtitle}</span>
      `;
      btn.addEventListener("click", () => {
        this.applyArchitecturePreset(archId);
      });
      grid.appendChild(btn);
    }
  }

  /**
   * Carga materiales desde GET /api/materials y actualiza el selector + badge.
   */
  setMaterials({ map, list }) {
    this.materialsMap = map || {};
    const sel = document.getElementById("material_id");
    if (!sel || !list?.length) return;

    const currentVal = sel.value || "petg_cf";
    sel.innerHTML = "";
    for (const mat of list) {
      const opt = document.createElement("option");
      opt.value = mat.id;
      const cat = (mat.category || "fdm").toUpperCase();
      opt.textContent = `${mat.name || mat.id} [${cat}]`;
      sel.appendChild(opt);
    }

    if (this.materialsMap[currentVal]) {
      sel.value = currentVal;
    } else if (this.materialsMap.petg_cf) {
      sel.value = "petg_cf";
    }
    this._renderMaterialBadge(sel.value);
  }

  _renderMaterialBadge(materialId) {
    const mat = this.materialsMap[materialId];
    if (!mat) return;

    const swatch = document.getElementById("mat-swatch");
    const cat = document.getElementById("mat-category");
    const cost = document.getElementById("mat-cost");
    const density = document.getElementById("mat-density");
    const youngs = document.getElementById("mat-youngs");
    const yieldEl = document.getElementById("mat-yield");
    const temp = document.getElementById("mat-temp");
    const notes = document.getElementById("mat-notes");

    if (swatch) swatch.style.background = mat.color_hex || "#38bdf8";
    if (cat) cat.textContent = (mat.category || "FDM").toUpperCase();
    if (cost) cost.textContent = `$${Number(mat.cost_per_kg_usd ?? 0).toFixed(0)}/kg`;
    if (density) density.textContent = `${Number(mat.density_g_cm3 ?? 0).toFixed(2)} g/cm³`;
    if (youngs) {
      youngs.textContent = `${Math.round(mat.youngs_xy_mpa ?? 0)} / ${Math.round(mat.youngs_z_mpa ?? 0)} MPa`;
    }
    if (yieldEl) {
      yieldEl.textContent = `${Math.round(mat.yield_xy_mpa ?? 0)} / ${Math.round(mat.yield_z_mpa ?? 0)} MPa`;
    }
    if (temp) temp.textContent = `${Math.round(mat.max_temp_c ?? 0)} °C`;
    if (notes) notes.textContent = mat.notes || "";
  }

  /**
   * Fusiona presets cargados desde GET /api/presets con los valores de respaldo.
   */
  setPresets({ architectures, endInterfaces }) {
    if (endInterfaces && typeof endInterfaces === "object") {
      for (const [key, val] of Object.entries(endInterfaces)) {
        if (val && typeof val === "object") {
          const flatEnd = val.params || val.defaults || val;
          this.endPresets[key] = {
            ...(this.endPresets[key] || {}),
            ...flatEnd,
          };
        }
      }
    }

    if (architectures && typeof architectures === "object") {
      const entries = Array.isArray(architectures)
        ? architectures.map((a) => [a.id || a.architecture, a])
        : Object.entries(architectures);

      for (const [key, val] of entries) {
        if (!key || !val || typeof val !== "object") continue;
        const flatParams = val.defaults || val.default_request || val.params || val;
        this.archPresets[key] = {
          ...(this.archPresets[key] || {}),
          ...flatParams,
          // Conservamos etiquetas en español para la interfaz
          name: ARCH_META[key]?.label || val.name,
          description: ARCH_META[key]?.subtitle || val.description || val.tagline,
        };
      }
    }

    this._renderArchitectureGrid();
  }

  applyArchitecturePreset(archId) {
    const preset = this.archPresets[archId] || FALLBACK_ARCH_DEFAULTS[archId];
    if (!preset) return;
    this.applyDesignParams({ ...preset, architecture: archId });
    this.onArchitectureChange(archId);
  }

  applyDesignParams(params) {
    if (!params) return;

    const archId = params.architecture || this.currentArchitecture;
    this.currentArchitecture = archId;

    // Actualizar cards activas
    document.querySelectorAll(".arch-card").forEach((card) => {
      card.classList.toggle("active", card.dataset.archId === archId);
    });

    // Actualizar header pill
    const headerLabel = document.getElementById("header-arch-label");
    if (headerLabel) {
      headerLabel.textContent = ARCH_META[archId]?.label || params.name || archId;
    }

    // Geometría
    this._setInputValue("length_mm", params.length_mm);
    this._setInputValue("root_width_mm", params.root_width_mm);
    this._setInputValue("tip_width_mm", params.tip_width_mm);
    this._setInputValue("thickness_mm", params.thickness_mm);
    if (params.organic_taper !== undefined) {
      const pct = params.organic_taper <= 1 ? Math.round(params.organic_taper * 100) : Math.round(params.organic_taper);
      this._setInputValue("organic_taper", pct);
    }
    if (params.topopt_volume_fraction !== undefined) {
      const vfPct =
        params.topopt_volume_fraction <= 1
          ? Math.round(params.topopt_volume_fraction * 100)
          : Math.round(params.topopt_volume_fraction);
      this._setInputValue("topopt_volume_fraction", vfPct);
    }
    this._setCheckbox("cable_channel", params.cable_channel);

    // Extremos
    if (params.root_end) this._setEndFormValues("root", params.root_end);
    if (params.tip_end) this._setEndFormValues("tip", params.tip_end);

    // Cargas
    this._setInputValue("tip_load_n", params.tip_load_n);
    this._setInputValue("angular_accel_rad_s2", params.angular_accel_rad_s2);
    this._setInputValue("max_deflection_mm", params.max_deflection_mm);
    this._setInputValue("target_safety_factor", params.target_safety_factor);
    this._setInputValue("operating_temp_c", params.operating_temp_c);

    // Material y manufactura
    if (params.material_id && this.materialsMap[params.material_id]) {
      this._setInputValue("material_id", params.material_id);
      this._renderMaterialBadge(params.material_id);
    }
    if (params.print_orientation) {
      this.currentOrientation = params.print_orientation;
      document.querySelectorAll("[data-orientation]").forEach((p) => {
        p.classList.toggle("active", p.dataset.orientation === this.currentOrientation);
      });
    }
    if (params.nozzle_diameter_mm !== undefined) {
      this._setInputValue("nozzle_diameter_mm", String(params.nozzle_diameter_mm));
    }
    if (params.printer_clearance_mm !== undefined) {
      this._setInputValue("printer_clearance_mm", params.printer_clearance_mm);
    }
  }

  _setInputValue(id, value) {
    if (value === undefined || value === null) return;
    const el = document.getElementById(id);
    if (!el) return;
    el.value = String(value);
    el.dispatchEvent(new Event("input", { bubbles: true }));
  }

  _setCheckbox(id, value) {
    if (value === undefined || value === null) return;
    const el = document.getElementById(id);
    if (!el) return;
    el.checked = Boolean(value);
  }

  _setEndFormValues(side, data) {
    if (!data) return;
    const presetEl = document.getElementById(`${side}-preset`);
    if (presetEl && data.preset) presetEl.value = data.preset;

    const mapFields = [
      "outer_diameter_mm",
      "bore_diameter_mm",
      "bolt_count",
      "bolt_pcd_mm",
      "bolt_size",
    ];
    for (const f of mapFields) {
      if (data[f] !== undefined && data[f] !== null) {
        const el = document.getElementById(`${side}-${f}`);
        if (el) el.value = String(data[f]);
      }
    }
    if (data.use_heat_inserts !== undefined) {
      this._setCheckbox(`${side}-use_heat_inserts`, data.use_heat_inserts);
    }
    this._updateEndTabSummary(side);
  }

  _updateEndTabSummary(side) {
    const summaryEl = document.getElementById(`tab-summary-${side}`);
    const presetVal = document.getElementById(`${side}-preset`)?.value || "custom";
    const od = Number(document.getElementById(`${side}-outer_diameter_mm`)?.value || 0);
    if (!summaryEl) return;
    const shortLabel = FALLBACK_END_PRESETS[presetVal]?.label || presetVal;
    summaryEl.textContent = `${shortLabel} · Ø${Math.round(od)}`;
  }

  _readEndInterface(side) {
    const preset = document.getElementById(`${side}-preset`)?.value || "custom";
    const outer = clamp(document.getElementById(`${side}-outer_diameter_mm`)?.value, 14, 120, 36);
    const bore = clamp(document.getElementById(`${side}-bore_diameter_mm`)?.value, 3, Math.min(90, outer - 2), 22);
    const boltCount = Math.round(clamp(document.getElementById(`${side}-bolt_count`)?.value, 0, 8, 4));
    const boltPcd = clamp(document.getElementById(`${side}-bolt_pcd_mm`)?.value, 10, 105, 29);
    const boltSize = document.getElementById(`${side}-bolt_size`)?.value || "M3";
    const useHeatInserts = Boolean(document.getElementById(`${side}-use_heat_inserts`)?.checked);

    return {
      preset,
      outer_diameter_mm: outer,
      bore_diameter_mm: bore,
      bolt_count: boltCount,
      bolt_pcd_mm: boltPcd,
      bolt_size: boltSize,
      use_heat_inserts: useHeatInserts,
    };
  }

  /**
   * Construye el payload JSON exacto que espera LinkDesignRequest en backend/app/schemas.py.
   */
  getPayload() {
    return {
      architecture: this.currentArchitecture,
      length_mm: clamp(document.getElementById("length_mm")?.value, 60, 450, 180),
      root_width_mm: clamp(document.getElementById("root_width_mm")?.value, 16, 110, 38),
      tip_width_mm: clamp(document.getElementById("tip_width_mm")?.value, 12, 90, 24),
      thickness_mm: clamp(document.getElementById("thickness_mm")?.value, 8, 60, 20),
      organic_taper: clamp(Number(document.getElementById("organic_taper")?.value || 75) / 100, 0, 1, 0.75),
      cable_channel: Boolean(document.getElementById("cable_channel")?.checked),

      root_end: this._readEndInterface("root"),
      tip_end: this._readEndInterface("tip"),

      tip_load_n: clamp(document.getElementById("tip_load_n")?.value, 1, 500, 25),
      angular_accel_rad_s2: clamp(document.getElementById("angular_accel_rad_s2")?.value, 0, 200, 15),
      target_safety_factor: clamp(document.getElementById("target_safety_factor")?.value, 1.1, 10, 2.0),
      max_deflection_mm: clamp(document.getElementById("max_deflection_mm")?.value, 0.05, 10, 0.5),
      operating_temp_c: clamp(document.getElementById("operating_temp_c")?.value, -10, 150, 45),

      material_id: document.getElementById("material_id")?.value || "petg_cf",
      print_orientation: this.currentOrientation,
      nozzle_diameter_mm: clamp(document.getElementById("nozzle_diameter_mm")?.value, 0.2, 1.0, 0.4),
      printer_clearance_mm: clamp(document.getElementById("printer_clearance_mm")?.value, 0, 0.5, 0.15),

      reference_volume_cm3: this.referenceVolumeCm3,
      topopt_volume_fraction: clamp(
        Number(document.getElementById("topopt_volume_fraction")?.value || 52) / 100,
        0.3,
        0.75,
        0.52
      ),
    };
  }
}
