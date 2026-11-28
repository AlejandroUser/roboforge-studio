import React, { useState } from "react";
import { api } from "../api";
import type { DFMAuditReport, OrganicDomainId, OrganicStudioResponse, PropellerWinner } from "../types";
import { DFMAuditCard } from "./families/DFMAuditCard";

const ORGANIC_DOMAINS: Array<{
  id: OrganicDomainId;
  icon: string;
  title: string;
  subtitle: string;
}> = [
  { id: "link", icon: "🦴", title: "Eslabón", subtitle: "SIMP 3D + Langelaar 45°" },
  { id: "mount", icon: "🔩", title: "Montura", subtitle: "NEMA17 / 23 / MG996R" },
  { id: "transmission", icon: "⚙️", title: "Transmisión", subtitle: "Engranaje Biónico" },
  { id: "full_arm", icon: "🦾", title: "Brazo Completo", subtitle: "Dual SIMP 3D + RNEA" },
  { id: "propeller", icon: "🌀", title: "Hélice", subtitle: "Evolución NSGA-II" },
  { id: "fpv_drone", icon: "🚁", title: "Dron Completo", subtitle: "Chasis + 4× NSGA-II" },
];

const METRIC_LABELS: Record<string, string> = {
  mass_g: "Masa (g)",
  volume_mm3: "Volumen (mm³)",
  mass_reduction_pct: "Ahorro Masa (%)",
  final_volume_fraction: "Fracción Vol.",
  simp_elapsed_s: "Tiempo SIMP (s)",
  gen_time_s: "Tiempo Total (s)",
  payload_kg: "Carga (kg)",
  torque_nm: "Torque (N·m)",
  actuator: "Actuador",
  teeth: "Dientes (Z)",
  module_mm: "Módulo (mm)",
  pitch_diameter_mm: "Ø Primitivo (mm)",
  tooth_safety_factor: "FS Diente",
  reach_mm: "Alcance (mm)",
  links_mass_g: "Masa Eslabones (g)",
  shoulder_torque_nm: "Torque Hombro (N·m)",
  thrust_g: "Empuje (g)",
  efficiency_g_per_w: "Eficiencia (g/W)",
  estimated_noise_dba: "Ruido (dBA)",
  root_safety_factor: "FS Raíz",
  baseline_g_per_w: "Base (g/W)",
  twr: "Relación TWR",
  total_thrust_g: "Empuje Total (g)",
  auw_mass_g: "Peso AUW (g)",
  frame_mass_g: "Masa Chasis (g)",
  prop_efficiency_g_per_w: "Hélice (g/W)",
  prop_noise_dba: "Ruido Hélice (dBA)",
  prop_root_sf: "FS Raíz Hélice",
};

interface OrganicSidebarProps {
  materials: Record<string, { id?: string; name?: string }>;
  printers: Record<string, { id?: string; name?: string }>;
  materialId: string;
  printerId: string;
  onChangeMaterial: (id: string) => void;
  onChangePrinter: (id: string) => void;
  onGenerated: (res: OrganicStudioResponse) => void;
  onStatus?: (msg: string) => void;
}

export const OrganicStudioSidebar: React.FC<OrganicSidebarProps> = ({
  materials,
  printers,
  materialId,
  printerId,
  onChangeMaterial,
  onChangePrinter,
  onGenerated,
  onStatus,
}) => {
  const [domain, setDomain] = useState<OrganicDomainId>("link");
  const [busy, setBusy] = useState(false);

  // Domain-specific parameters
  const [lengthMm, setLengthMm] = useState(180);
  const [payloadKg, setPayloadKg] = useState(2.0);
  const [accel, setAccel] = useState(8);
  const [volFrac, setVolFrac] = useState(0.38);
  const [overhang45, setOverhang45] = useState(true);

  const [actuator, setActuator] = useState("nema17");
  const [torqueNm, setTorqueNm] = useState(2.5);

  const [teeth, setTeeth] = useState(32);
  const [moduleMm, setModuleMm] = useState(2.0);
  const [faceW, setFaceW] = useState(12);
  const [spokes, setSpokes] = useState(6);

  const [upperL, setUpperL] = useState(180);
  const [foreL, setForeL] = useState(150);

  const [targetThrustG, setTargetThrustG] = useState(600);
  const [propInch, setPropInch] = useState(5.0);
  const [rpm, setRpm] = useState(18000);
  const [blades, setBlades] = useState(3);
  const [priority, setPriority] = useState("max_efficiency");

  const [wheelbaseMm, setWheelbaseMm] = useState(215);
  const [plateT, setPlateT] = useState(4.0);

  const runSculpt = async (overridePriority?: string) => {
    setBusy(true);
    onStatus?.(`Esculpiendo ${ORGANIC_DOMAINS.find((d) => d.id === domain)?.title || domain} por física...`);
    try {
      let params: Record<string, unknown> = {};
      if (domain === "link") {
        params = {
          length_mm: lengthMm,
          payload_kg: payloadKg,
          angular_accel_rad_s2: accel,
          volume_fraction: volFrac,
          overhang_filter_45: overhang45,
        };
      } else if (domain === "mount") {
        params = {
          actuator_type: actuator,
          torque_nm: torqueNm,
          volume_fraction: volFrac,
          overhang_filter_45: overhang45,
        };
      } else if (domain === "transmission") {
        params = {
          teeth,
          module_mm: moduleMm,
          face_width_mm: faceW,
          torque_nm: torqueNm,
          spokes,
        };
      } else if (domain === "full_arm") {
        params = {
          upper_length_mm: upperL,
          forearm_length_mm: foreL,
          payload_kg: payloadKg,
          angular_accel_rad_s2: accel,
          volume_fraction: volFrac,
          overhang_filter_45: overhang45,
        };
      } else if (domain === "propeller") {
        params = {
          target_thrust_g: targetThrustG,
          diameter_inch: propInch,
          rpm,
          blade_count: blades,
          priority: overridePriority ?? priority,
        };
      } else if (domain === "fpv_drone") {
        params = {
          prop_inch: propInch,
          wheelbase_mm: wheelbaseMm,
          plate_thickness_mm: plateT,
          rpm,
          priority: overridePriority ?? priority,
        };
      }

      const res = await api.generateOrganicStudio({
        domain,
        material_id: materialId,
        printer_id: printerId,
        params,
      });
      onGenerated(res);
      onStatus?.(`${res.name} listo (DFM ${res.dfm_audit.status.toUpperCase()}).`);
    } catch (err) {
      onStatus?.(`Error en Esculpido Orgánico: ${err instanceof Error ? err.message : String(err)}`);
    } finally {
      setBusy(false);
    }
  };

  return (
    <>
      <div className="section-card">
        <div className="section-title">
          <span>🧬 Estudio de Esculpido Orgánico</span>
          <span className="badge-tag">v14.0</span>
        </div>
        <div
          style={{
            display: "grid",
            gridTemplateColumns: "1fr 1fr",
            gap: 6,
            marginBottom: 10,
          }}
        >
          {ORGANIC_DOMAINS.map((d) => (
            <button
              key={d.id}
              type="button"
              className={`btn-tool ${domain === d.id ? "active" : ""}`}
              style={{
                padding: "8px 6px",
                textAlign: "left",
                whiteSpace: "normal",
                lineHeight: 1.2,
                minWidth: 0,
                display: "flex",
                flexDirection: "column",
                gap: 2,
              }}
              onClick={() => setDomain(d.id)}
            >
              <strong style={{ fontSize: 11.5 }}>
                {d.icon} {d.title}
              </strong>
              <small style={{ fontSize: 9.5, opacity: 0.72 }}>{d.subtitle}</small>
            </button>
          ))}
        </div>

        {/* Domain-specific controls */}
        {domain === "link" && (
          <>
            <div className="field-row">
              <div className="field-head">
                <span>Longitud Eslabón</span>
                <span>{lengthMm} mm</span>
              </div>
              <input type="range" min={100} max={280} step={10} value={lengthMm} onChange={(e) => setLengthMm(Number(e.target.value))} />
            </div>
            <div className="field-row">
              <div className="field-head">
                <span>Carga Útil</span>
                <span>{payloadKg.toFixed(1)} kg</span>
              </div>
              <input type="range" min={0.5} max={10} step={0.5} value={payloadKg} onChange={(e) => setPayloadKg(Number(e.target.value))} />
            </div>
            <div className="field-row">
              <div className="field-head">
                <span>Aceleración Dinámica</span>
                <span>{accel} rad/s²</span>
              </div>
              <input type="range" min={0} max={30} step={2} value={accel} onChange={(e) => setAccel(Number(e.target.value))} />
            </div>
            <div className="field-row">
              <div className="field-head">
                <span>Fracción Volumen</span>
                <span>{Math.round(volFrac * 100)}%</span>
              </div>
              <input type="range" min={0.25} max={0.55} step={0.02} value={volFrac} onChange={(e) => setVolFrac(Number(e.target.value))} />
            </div>
            <label style={{ display: "flex", alignItems: "center", gap: 6, fontSize: 11, marginBottom: 8 }}>
              <input type="checkbox" checked={overhang45} onChange={(e) => setOverhang45(e.target.checked)} />
              Filtro Langelaar 45° (Sin Soportes)
            </label>
          </>
        )}

        {domain === "mount" && (
          <>
            <div className="field-row">
              <div className="field-head"><span>Actuador / Motor</span></div>
              <select value={actuator} onChange={(e) => setActuator(e.target.value)}>
                <option value="nema17">NEMA 17 (42×42 mm · PCD 31)</option>
                <option value="nema23">NEMA 23 (57×57 mm · PCD 47.1)</option>
                <option value="mg996r">Servo MG996R / RDS3225</option>
              </select>
            </div>
            <div className="field-row">
              <div className="field-head">
                <span>Torque de Reacción</span>
                <span>{torqueNm.toFixed(1)} N·m</span>
              </div>
              <input type="range" min={0.5} max={15} step={0.5} value={torqueNm} onChange={(e) => setTorqueNm(Number(e.target.value))} />
            </div>
            <div className="field-row">
              <div className="field-head">
                <span>Fracción Volumen</span>
                <span>{Math.round(volFrac * 100)}%</span>
              </div>
              <input type="range" min={0.30} max={0.56} step={0.02} value={volFrac} onChange={(e) => setVolFrac(Number(e.target.value))} />
            </div>
          </>
        )}

        {domain === "transmission" && (
          <>
            <div className="two-col">
              <div className="field-row">
                <div className="field-head"><span>Dientes (Z)</span><span>{teeth}</span></div>
                <input type="range" min={16} max={56} step={2} value={teeth} onChange={(e) => setTeeth(Number(e.target.value))} />
              </div>
              <div className="field-row">
                <div className="field-head"><span>Módulo</span><span>{moduleMm.toFixed(1)}</span></div>
                <input type="range" min={1.0} max={3.5} step={0.5} value={moduleMm} onChange={(e) => setModuleMm(Number(e.target.value))} />
              </div>
            </div>
            <div className="two-col">
              <div className="field-row">
                <div className="field-head"><span>Ancho Cara</span><span>{faceW} mm</span></div>
                <input type="range" min={6} max={24} step={2} value={faceW} onChange={(e) => setFaceW(Number(e.target.value))} />
              </div>
              <div className="field-row">
                <div className="field-head"><span>Radios</span><span>{spokes}</span></div>
                <input type="range" min={5} max={9} step={1} value={spokes} onChange={(e) => setSpokes(Number(e.target.value))} />
              </div>
            </div>
            <div className="field-row">
              <div className="field-head"><span>Torque Transmitido</span><span>{torqueNm.toFixed(1)} N·m</span></div>
              <input type="range" min={0.5} max={25} step={0.5} value={torqueNm} onChange={(e) => setTorqueNm(Number(e.target.value))} />
            </div>
          </>
        )}

        {domain === "full_arm" && (
          <>
            <div className="two-col">
              <div className="field-row">
                <div className="field-head"><span>Húmero</span><span>{upperL} mm</span></div>
                <input type="range" min={130} max={240} step={10} value={upperL} onChange={(e) => setUpperL(Number(e.target.value))} />
              </div>
              <div className="field-row">
                <div className="field-head"><span>Antebrazo</span><span>{foreL} mm</span></div>
                <input type="range" min={110} max={210} step={10} value={foreL} onChange={(e) => setForeL(Number(e.target.value))} />
              </div>
            </div>
            <div className="field-row">
              <div className="field-head"><span>Carga Útil</span><span>{payloadKg.toFixed(1)} kg</span></div>
              <input type="range" min={0.5} max={6} step={0.5} value={payloadKg} onChange={(e) => setPayloadKg(Number(e.target.value))} />
            </div>
            <div className="field-row">
              <div className="field-head"><span>Fracción Volumen</span><span>{Math.round(volFrac * 100)}%</span></div>
              <input type="range" min={0.30} max={0.52} step={0.02} value={volFrac} onChange={(e) => setVolFrac(Number(e.target.value))} />
            </div>
          </>
        )}

        {(domain === "propeller" || domain === "fpv_drone") && (
          <>
            <div className="field-row">
              <div className="field-head"><span>Diámetro Hélice</span><span>{propInch.toFixed(1)}&quot;</span></div>
              <input type="range" min={3.0} max={7.0} step={0.5} value={propInch} onChange={(e) => setPropInch(Number(e.target.value))} />
            </div>
            {domain === "propeller" && (
              <div className="field-row">
                <div className="field-head"><span>Empuje Objetivo</span><span>{targetThrustG} g</span></div>
                <input type="range" min={200} max={1800} step={50} value={targetThrustG} onChange={(e) => setTargetThrustG(Number(e.target.value))} />
              </div>
            )}
            {domain === "fpv_drone" && (
              <div className="two-col">
                <div className="field-row">
                  <div className="field-head"><span>Wheelbase</span><span>{wheelbaseMm} mm</span></div>
                  <input type="range" min={145} max={245} step={5} value={wheelbaseMm} onChange={(e) => setWheelbaseMm(Number(e.target.value))} />
                </div>
                <div className="field-row">
                  <div className="field-head"><span>Placa</span><span>{plateT.toFixed(1)} mm</span></div>
                  <input type="range" min={2.5} max={5.5} step={0.5} value={plateT} onChange={(e) => setPlateT(Number(e.target.value))} />
                </div>
              </div>
            )}
            <div className="two-col">
              <div className="field-row">
                <div className="field-head"><span>RPM</span><span>{rpm}</span></div>
                <input type="range" min={10000} max={28000} step={1000} value={rpm} onChange={(e) => setRpm(Number(e.target.value))} />
              </div>
              <div className="field-row">
                <div className="field-head"><span>Prioridad NSGA-II</span></div>
                <select value={priority} onChange={(e) => setPriority(e.target.value)}>
                  <option value="max_efficiency">Máx. Eficiencia (g/W)</option>
                  <option value="ultra_quiet">Ultra-Silenciosa (dBA)</option>
                  <option value="max_thrust">Máx. Empuje</option>
                </select>
              </div>
            </div>
            {domain === "propeller" && (
              <div className="field-row">
                <div className="field-head"><span>Número de Palas</span><span>{blades}p</span></div>
                <input type="range" min={2} max={4} step={1} value={blades} onChange={(e) => setBlades(Number(e.target.value))} />
              </div>
            )}
          </>
        )}

        <button
          type="button"
          className="btn-primary"
          disabled={busy}
          style={{ width: "100%", marginTop: 8, padding: "10px 12px", fontWeight: 700 }}
          onClick={() => runSculpt()}
        >
          {busy ? "🧬 Esculpiendo por Física..." : `🧬 Esculpir ${ORGANIC_DOMAINS.find((d) => d.id === domain)?.title}`}
        </button>
      </div>

      {/* Material & Printer card */}
      <div className="section-card">
        <div className="section-title"><span>Material e Impresora 3D</span></div>
        <div className="field-row">
          <div className="field-head"><span>Material</span></div>
          <select value={materialId} onChange={(e) => onChangeMaterial(e.target.value)}>
            {Object.entries(materials).map(([id, m]) => (
              <option key={id} value={id}>{m.name || id}</option>
            ))}
          </select>
        </div>
        <div className="field-row">
          <div className="field-head"><span>Impresora 3D</span></div>
          <select value={printerId} onChange={(e) => onChangePrinter(e.target.value)}>
            {Object.entries(printers).map(([id, p]) => (
              <option key={id} value={id}>{p.name || id}</option>
            ))}
          </select>
        </div>
      </div>
    </>
  );
};

interface OrganicInspectorProps {
  result: OrganicStudioResponse | null;
}

export const OrganicStudioInspector: React.FC<OrganicInspectorProps> = ({ result }) => {
  if (!result) {
    return (
      <div className="section-card">
        <div className="section-title"><span>🧬 Esculpido Orgánico v14.0</span></div>
        <p style={{ fontSize: 12, color: "var(--muted)", lineHeight: 1.5 }}>
          Elige cualquiera de los <strong>6 dominios</strong> a la izquierda (Eslabón, Montura, Transmisión,
          Brazo Completo, Hélice o Dron Completo), ajusta tus requisitos físicos y pulsa{" "}
          <strong>Esculpir</strong> para generar geometría orgánica única validada con Goldfish DFM.
        </p>
      </div>
    );
  }

  const hist = result.compliance_history || [];
  const cMax = hist.length ? Math.max(...hist, 1e-6) : 1;
  const cMin = hist.length ? Math.min(...hist) : 0;
  const polyPoints = hist
    .map((v, i) => {
      const x = 8 + (i / Math.max(hist.length - 1, 1)) * 224;
      const y = 56 - ((v - cMin) / Math.max(cMax - cMin, 1e-6)) * 44;
      return `${x.toFixed(1)},${y.toFixed(1)}`;
    })
    .join(" ");

  return (
    <>
      <div className="section-card">
        <div className="section-title">
          <span>{result.name}</span>
          <span className="badge-tag">{result.domain.toUpperCase()}</span>
        </div>

        {/* KPI Grid */}
        <div style={{ display: "grid", gridTemplateColumns: "1fr 1fr", gap: 6, marginBottom: 10 }}>
          {Object.entries(result.metrics).slice(0, 8).map(([k, v]) => (
            <div
              key={k}
              style={{
                background: "rgba(15, 23, 42, 0.55)",
                border: "1px solid rgba(255,255,255,0.06)",
                borderRadius: 6,
                padding: "6px 8px",
              }}
            >
              <div style={{ fontSize: 9.5, color: "var(--muted)", textTransform: "uppercase" }}>
                {METRIC_LABELS[k] || k}
              </div>
              <div style={{ fontSize: 13, fontWeight: 700, fontFamily: "var(--mono)", fontVariantNumeric: "tabular-nums" }}>
                {typeof v === "boolean" ? (v ? "Sí" : "No") : String(v)}
              </div>
            </div>
          ))}
        </div>

        {/* Downloads */}
        <div style={{ display: "grid", gridTemplateColumns: "1fr 1fr", gap: 6 }}>
          {result.stl_url && (
            <a className="btn-tool" style={{ textAlign: "center", textDecoration: "none" }} href={result.stl_url} download>
              ⬇ Descargar .STL
            </a>
          )}
          {result.threemf_url && (
            <a className="btn-tool" style={{ textAlign: "center", textDecoration: "none" }} href={result.threemf_url} download>
              ⬇ Plato .3MF
            </a>
          )}
          {result.step_url && (
            <a className="btn-tool" style={{ textAlign: "center", textDecoration: "none" }} href={result.step_url} download>
              ⬇ CAD .STEP
            </a>
          )}
          {result.dxf_url && (
            <a className="btn-tool" style={{ textAlign: "center", textDecoration: "none" }} href={result.dxf_url} download>
              ⬇ CNC .DXF
            </a>
          )}
        </div>
      </div>

      {/* SIMP 3D Compliance Convergence SVG */}
      {hist.length >= 2 && (
        <div className="section-card">
          <div className="section-title">
            <span>Convergencia SIMP 3D</span>
            <span className="badge-tag">{hist.length} iter</span>
          </div>
          <svg viewBox="0 0 240 64" style={{ width: "100%", height: 68, display: "block" }}>
            <polyline fill="none" stroke="#38bdf8" strokeWidth="2.2" points={polyPoints} />
          </svg>
        </div>
      )}

      {/* Pareto Winners (for Propellers & Drones) */}
      {result.pareto_winners && result.pareto_winners.length > 0 && (
        <div className="section-card">
          <div className="section-title"><span>Frente de Pareto NSGA-II</span></div>
          <div style={{ display: "flex", flexDirection: "column", gap: 6 }}>
            {result.pareto_winners.map((w: PropellerWinner) => (
              <div
                key={w.candidate_id}
                style={{
                  background: "rgba(15, 23, 42, 0.5)",
                  border: "1px solid rgba(56, 189, 248, 0.2)",
                  borderRadius: 6,
                  padding: "6px 8px",
                  fontSize: 11,
                }}
              >
                <strong>{w.label}</strong>
                <div style={{ color: "var(--muted)", fontFamily: "var(--mono)", fontSize: 10 }}>
                  {w.metrics.thrust_g}g · {w.metrics.efficiency_g_per_w} g/W · {w.metrics.estimated_noise_dba} dBA · FS {w.metrics.root_safety_factor}
                </div>
              </div>
            ))}
          </div>
        </div>
      )}

      {/* Notes */}
      {result.notes && result.notes.length > 0 && (
        <div className="section-card">
          <div className="section-title"><span>Resumen de Ingeniería</span></div>
          <ul style={{ margin: 0, paddingLeft: 16, fontSize: 11, color: "var(--muted)", lineHeight: 1.5 }}>
            {result.notes.map((n, i) => (
              <li key={i}>{n}</li>
            ))}
          </ul>
        </div>
      )}

      {/* Goldfish DFM Audit */}
      <DFMAuditCard audit={result.dfm_audit as DFMAuditReport} />
    </>
  );
};
