/**
 * comparator.js — Panel derecho de Comparador de Variantes y Alertas DFM/Térmicas
 */

const VARIANT_COLORS = {
  tapered_solid: "#94a3b8",
  organic_ibeam: "#38bdf8",
  rounded_truss: "#10b981",
  hollow_cobot: "#a78bfa",
  organic_topopt: "#f59e0b",
};

const ALERT_ICONS = {
  info: "ℹ",
  warning: "⚠",
  critical: "⛔",
};

function escapeHtml(str) {
  return String(str ?? "")
    .replaceAll("&", "&amp;")
    .replaceAll("<", "&lt;")
    .replaceAll(">", "&gt;")
    .replaceAll('"', "&quot;");
}

function fmt(val, decimals = 1) {
  const n = Number(val);
  if (!Number.isFinite(n)) return "—";
  return n.toFixed(decimals);
}

export class ComparatorController {
  constructor({ onSelectVariant } = {}) {
    this.onSelectVariant = onSelectVariant || (() => {});
    this.alertsEl = document.getElementById("global-alerts");
    this.feaCardEl = document.getElementById("fea-results-card");
    this.listEl = document.getElementById("variants-list");
    this.hashPillEl = document.getElementById("design-hash-pill");
    this.subtitleEl = document.getElementById("comparator-subtitle");
    this.activeVariantId = null;
    this.lastResponse = null;
  }

  render(response, activeVariantId, targetSafetyFactor = 2.0, maxDeflectionMm = 0.5) {
    this.lastResponse = response;
    this.activeVariantId = activeVariantId || response?.recommended_variant || response?.variants?.[0]?.variant_id;

    // Header hash & material
    if (this.hashPillEl && response?.design_hash) {
      this.hashPillEl.textContent = `#${response.design_hash.slice(0, 8)}`;
      this.hashPillEl.classList.remove("hidden");
    }
    if (this.subtitleEl && response?.material_name) {
      this.subtitleEl.textContent = `Material: ${response.material_name} · Clic en tarjeta para ver 3D`;
    }

    this._renderGlobalAlerts(response?.global_alerts || []);
    this._renderVariantCards(response, targetSafetyFactor, maxDeflectionMm);
  }

  setActiveVariant(variantId) {
    this.activeVariantId = variantId;
    if (!this.listEl) return;
    this.listEl.querySelectorAll(".variant-card").forEach((card) => {
      card.classList.toggle("active", card.dataset.variantId === variantId);
    });
  }

  renderFeaResult(feaResult, variantName, isShowingFeaMesh, { onToggleMesh } = {}) {
    if (!this.feaCardEl) return;
    if (!feaResult) {
      this.feaCardEl.classList.add("hidden");
      this.feaCardEl.innerHTML = "";
      return;
    }

    this.feaCardEl.classList.remove("hidden");
    this.feaCardEl.classList.toggle("fea-fail", !feaResult.passes_fea);

    const statusBadge = feaResult.passes_fea
      ? `<span class="badge badge-recommended">✓ Aprobado FEA</span>`
      : `<span class="badge badge-fail">✕ Revisar</span>`;

    const fsClass = feaResult.passes_fea ? "good" : feaResult.fea_safety_factor >= 1.2 ? "warn" : "bad";
    const toggleLabel = isShowingFeaMesh ? "Ver Modelo CAD" : "Ver Malla FEA 3D";
    const notesHtml = (feaResult.notes || [])
      .map((n) => `<li>${escapeHtml(n)}</li>`)
      .join("");

    this.feaCardEl.innerHTML = `
      <div class="fea-card-head">
        <div class="fea-card-title">
          <strong>Resultados FEA 3D Real</strong>
          <small>${escapeHtml(variantName || feaResult.variant_id)} · ${escapeHtml(feaResult.solver_name || "Gmsh + Tet4")}</small>
        </div>
        ${statusBadge}
      </div>

      <div class="metrics-grid">
        <div class="metric-cell">
          <span class="metric-label">Nodos</span>
          <span class="metric-val">${Number(feaResult.node_count || 0).toLocaleString("es")}</span>
          <span class="metric-sub">3D Tet4</span>
        </div>
        <div class="metric-cell">
          <span class="metric-label">Tetraedros</span>
          <span class="metric-val">${Number(feaResult.element_count || 0).toLocaleString("es")}</span>
          <span class="metric-sub">${fmt(feaResult.solve_time_s, 2)} s</span>
        </div>
        <div class="metric-cell">
          <span class="metric-label">FS FEA</span>
          <span class="metric-val ${fsClass}">${fmt(feaResult.fea_safety_factor, 2)}×</span>
          <span class="metric-sub">Sy ${fmt(feaResult.effective_yield_mpa, 0)} MPa</span>
        </div>
        <div class="metric-cell">
          <span class="metric-label">Von Mises P95</span>
          <span class="metric-val">${fmt(feaResult.p95_von_mises_mpa, 1)}</span>
          <span class="metric-sub">MPa (efect.)</span>
        </div>
        <div class="metric-cell">
          <span class="metric-label">Von Mises Pico</span>
          <span class="metric-val">${fmt(feaResult.max_von_mises_mpa, 1)}</span>
          <span class="metric-sub">MPa máx.</span>
        </div>
        <div class="metric-cell">
          <span class="metric-label">Desplazamiento</span>
          <span class="metric-val">${fmt(feaResult.max_displacement_mm, 2)}</span>
          <span class="metric-sub">mm máx.</span>
        </div>
      </div>

      <div class="fea-card-actions">
        <button type="button" class="btn-fea-switch" id="btn-fea-card-toggle">${toggleLabel}</button>
      </div>

      <ul class="fea-notes-list">${notesHtml}</ul>
    `;

    const toggleBtn = this.feaCardEl.querySelector("#btn-fea-card-toggle");
    toggleBtn?.addEventListener("click", () => {
      onToggleMesh?.();
    });
  }

  _renderGlobalAlerts(alerts) {
    if (!this.alertsEl) return;
    if (!alerts.length) {
      this.alertsEl.innerHTML = "";
      return;
    }

    this.alertsEl.innerHTML = alerts
      .map((a) => {
        const level = ["info", "warning", "critical"].includes(a.level) ? a.level : "info";
        const icon = ALERT_ICONS[level];
        return `
          <div class="alert-banner ${level}">
            <span class="alert-icon" aria-hidden="true">${icon}</span>
            <div class="alert-body">
              <span class="alert-code">${escapeHtml(a.code || level)}</span>
              <span>${escapeHtml(a.message)}</span>
            </div>
          </div>
        `;
      })
      .join("");
  }

  _renderVariantCards(response, targetFS, maxDefl) {
    if (!this.listEl) return;
    const variants = response?.variants || [];
    if (!variants.length) {
      this.listEl.innerHTML = `<div class="empty-state"><p>No se generaron variantes válidas.</p></div>`;
      return;
    }

    const recommendedId = response.recommended_variant;
    this.listEl.innerHTML = "";

    for (const v of variants) {
      const m = v.metrics || {};
      const isRec = v.variant_id === recommendedId;
      const isActive = v.variant_id === this.activeVariantId;
      const accentColor = VARIANT_COLORS[v.variant_id] || "#38bdf8";

      const savingsNum = Number(m.mass_savings_pct ?? 0);
      const savingsText =
        Math.abs(savingsNum) < 0.05
          ? "Base (0%)"
          : savingsNum > 0
            ? `−${fmt(savingsNum, 1)}% peso`
            : `+${fmt(Math.abs(savingsNum), 1)}% peso`;

      const fsVal = Number(m.safety_factor ?? 0);
      const fsClass = fsVal >= targetFS ? "good" : fsVal >= 1.2 ? "warn" : "bad";

      const deflVal = Number(m.tip_deflection_mm ?? 0);
      const deflClass = deflVal <= maxDefl ? "good" : deflVal <= maxDefl * 1.35 ? "warn" : "bad";

      const variantAlertsHtml = (v.alerts || []).length
        ? `<div class="variant-alerts">
            ${v.alerts
              .map((a) => {
                const lvl = ["info", "warning", "critical"].includes(a.level) ? a.level : "info";
                return `<div class="variant-alert-item ${lvl}">${escapeHtml(a.message)}</div>`;
              })
              .join("")}
          </div>`
        : "";

      const card = document.createElement("article");
      card.className = `variant-card${isActive ? " active" : ""}`;
      card.dataset.variantId = v.variant_id;
      card.tabIndex = 0;

      const isBionic = v.variant_id === "organic_topopt";
      const hasRefSavings =
        m.reference_savings_pct !== null &&
        m.reference_savings_pct !== undefined &&
        Number.isFinite(Number(m.reference_savings_pct));
      const refSavNum = hasRefSavings ? Number(m.reference_savings_pct) : 0;
      const refSavBadge = hasRefSavings
        ? `<span class="badge badge-ref-savings">${
            refSavNum >= 0 ? `−${fmt(refSavNum, 1)}%` : `+${fmt(Math.abs(refSavNum), 1)}%`
          } vs Ref. .STEP</span>`
        : "";

      card.innerHTML = `
        <div class="variant-card-head">
          <div class="variant-title-row">
            <span class="variant-name">
              <span class="variant-color-dot" style="background:${accentColor}; box-shadow:0 0 8px ${accentColor}88;"></span>
              ${escapeHtml(v.name || v.variant_id)}
            </span>
          </div>
          <div class="variant-tagline">${escapeHtml(v.tagline || "")}</div>
          <div class="badge-row">
            ${isBionic ? `<span class="badge badge-bionic">✦ Biónica SIMP</span>` : ""}
            ${isRec ? `<span class="badge badge-recommended">★ Recomendada</span>` : ""}
            ${
              m.passes_requirements
                ? `<span class="badge badge-pass">✓ Cumple</span>`
                : `<span class="badge badge-fail">✕ No cumple</span>`
            }
            ${
              m.print_without_supports
                ? `<span class="badge badge-support-free">Sin Soportes</span>`
                : `<span class="badge badge-supports-req">Requiere Soportes</span>`
            }
            ${refSavBadge}
          </div>
        </div>

        <div class="metrics-grid">
          <div class="metric-cell">
            <span class="metric-label">Masa</span>
            <span class="metric-val">${fmt(m.mass_g, 1)} g</span>
            <span class="metric-sub${savingsNum > 0.5 ? " savings" : ""}">${savingsText}</span>
          </div>
          <div class="metric-cell">
            <span class="metric-label">Factor Seg.</span>
            <span class="metric-val ${fsClass}">${fmt(fsVal, 2)}×</span>
            <span class="metric-sub">${fmt(m.max_stress_mpa, 1)} MPa</span>
          </div>
          <div class="metric-cell">
            <span class="metric-label">Deflexión</span>
            <span class="metric-val ${deflClass}">${fmt(deflVal, 2)} mm</span>
            <span class="metric-sub">Máx ${fmt(maxDefl, 2)}</span>
          </div>
          <div class="metric-cell">
            <span class="metric-label">Inercia Rot.</span>
            <span class="metric-val">${fmt(m.rotational_inertia_kg_cm2, 2)}</span>
            <span class="metric-sub">kg·cm²</span>
          </div>
          <div class="metric-cell">
            <span class="metric-label">Torque Din.</span>
            <span class="metric-val">${fmt(m.dynamic_torque_nm, 2)}</span>
            <span class="metric-sub">N·m</span>
          </div>
          <div class="metric-cell">
            <span class="metric-label">Costo Est.</span>
            <span class="metric-val">$${fmt(m.estimated_cost_usd, 2)}</span>
            <span class="metric-sub">Score ${fmt(m.stiffness_to_weight_score, 0)}</span>
          </div>
        </div>

        ${variantAlertsHtml}

        <div class="variant-actions">
          <a class="btn-download" href="${escapeHtml(v.step_url)}" download title="Descargar sólido B-Rep (.STEP)">
            <svg viewBox="0 0 16 16" fill="none" aria-hidden="true">
              <path d="M8 2.5V10.5M8 10.5L5 7.5M8 10.5L11 7.5M2.5 12.5H13.5" stroke="currentColor" stroke-width="1.5" stroke-linecap="round" stroke-linejoin="round"/>
            </svg>
            .STEP
          </a>
          <a class="btn-download" href="${escapeHtml(v.stl_url)}" download title="Descargar malla para impresión 3D (.STL)">
            <svg viewBox="0 0 16 16" fill="none" aria-hidden="true">
              <path d="M8 2.5V10.5M8 10.5L5 7.5M8 10.5L11 7.5M2.5 12.5H13.5" stroke="currentColor" stroke-width="1.5" stroke-linecap="round" stroke-linejoin="round"/>
            </svg>
            .STL
          </a>
        </div>
      `;

      const selectHandler = (e) => {
        if (e.target.closest("a.btn-download")) return;
        this.setActiveVariant(v.variant_id);
        this.onSelectVariant(v);
      };

      card.addEventListener("click", selectHandler);
      card.addEventListener("keydown", (e) => {
        if (e.key === "Enter" || e.key === " ") {
          e.preventDefault();
          selectHandler(e);
        }
      });

      this.listEl.appendChild(card);
    }
  }
}

export { VARIANT_COLORS };
