import React, { useEffect, useMemo, useState } from "react";
import { api } from "../../api";
import type {
  ProjectFamilyId,
  ProjectFamilyInfo,
  ProjectGenerateResponse,
} from "../../types";
import { DFMAuditCard } from "./DFMAuditCard";

const FALLBACK_FAMILIES: ProjectFamilyInfo[] = [
  {
    id: "fpv_drone",
    name: "Drones FPV",
    subtitle: "Chasis X/True-X · Motores 16×16/19×19 · Stack FC",
    icon: "🚁",
    presets: [
      {
        id: "cinewhoop_3in",
        name: "Cinewhoop 3\"",
        subtitle: "145 mm · 4S · Stack 20×20",
        material_id: "petg_cf",
        printer_id: "bambu_x1c",
        defaults: { prop_size_in: 3.0, plate_thickness_mm: 2.5, stack_mm: 20, battery_cells: 4 },
      },
      {
        id: "freestyle_5in",
        name: "Freestyle 5\"",
        subtitle: "225 mm · 6S · Stack 30.5×30.5",
        material_id: "pa12_cf",
        printer_id: "bambu_x1c",
        defaults: { prop_size_in: 5.0, plate_thickness_mm: 4.0, stack_mm: 30.5, battery_cells: 6 },
      },
      {
        id: "longrange_7in",
        name: "Long-Range 7\"",
        subtitle: "310 mm · 6S Li-Ion · Eficiencia",
        material_id: "pa12_cf",
        printer_id: "bambu_x1c",
        defaults: { prop_size_in: 7.0, plate_thickness_mm: 5.0, stack_mm: 30.5, battery_cells: 6 },
      },
    ],
  },
  {
    id: "rover_4wd",
    name: "Rovers 4WD",
    subtitle: "Chasis 2 Cubiertas · TT / Mecanum / LiDAR",
    icon: "🤖",
    presets: [
      {
        id: "mini_rover_tt",
        name: "Mini Rover TT",
        subtitle: "160×115 mm · 4× Motores TT · ESP32",
        material_id: "petg_cf",
        printer_id: "bambu_x1c",
        defaults: { chassis_length_mm: 160, chassis_width_mm: 115, wheel_diameter_mm: 65, lidar_turret: false },
      },
      {
        id: "mecanum_nema17",
        name: "Mecanum NEMA 17",
        subtitle: "260×190 mm · Omnidireccional 4WD",
        material_id: "petg_cf",
        printer_id: "bambu_x1c",
        defaults: { chassis_length_mm: 260, chassis_width_mm: 190, wheel_diameter_mm: 80, lidar_turret: true },
      },
      {
        id: "explorer_lidar",
        name: "Explorer LiDAR",
        subtitle: "290×210 mm · Torreta SLAM 360°",
        material_id: "pa12_cf",
        printer_id: "bambu_x1c",
        defaults: { chassis_length_mm: 290, chassis_width_mm: 210, wheel_diameter_mm: 100, lidar_turret: true },
      },
    ],
  },
  {
    id: "camera_gimbal",
    name: "Gimbals 2–3 Ejes",
    subtitle: "Antivibración · Roll/Pitch/Yaw · MSLA / Brushless",
    icon: "🎥",
    presets: [
      {
        id: "gimbal_2axis_mars5",
        name: "Gimbal 2-Ejes Mars 5",
        subtitle: "MG90S · 100% Elegoo Mars 5 Ultra",
        material_id: "resin_abs_like",
        printer_id: "elegoo_mars_5_ultra",
        defaults: { axes: 2, camera_mass_g: 45, cradle_width_mm: 46, motor_type: "servo_mg90s" },
      },
      {
        id: "gimbal_3axis_gopro",
        name: "Gimbal 3-Ejes GoPro",
        subtitle: "Brushless 2204 · Acción 150 g",
        material_id: "petg_cf",
        printer_id: "bambu_x1c",
        defaults: { axes: 3, camera_mass_g: 150, cradle_width_mm: 72, motor_type: "bldc_2204" },
      },
    ],
  },
];

const METRIC_LABELS: Record<string, { label: string; unit?: string }> = {
  mass_g: { label: "Masa Estructura", unit: "g" },
  total_mass_g: { label: "Masa Total (AUW)", unit: "g" },
  auw_g: { label: "Peso Despegue", unit: "g" },
  twr: { label: "Empuje/Peso (TWR)", unit: "×" },
  flight_time_min: { label: "Autonomía Est.", unit: "min" },
  wheelbase_mm: { label: "Diagonal Motor", unit: "mm" },
  traction_torque_nm: { label: "Torque Tracción", unit: "N·m" },
  max_climb_deg: { label: "Pendiente Máx.", unit: "°" },
  max_speed_m_s: { label: "Velocidad Máx.", unit: "m/s" },
  cg_offset_mm: { label: "Desfase CG", unit: "mm" },
  motor_torque_nm: { label: "Par Motor Req.", unit: "N·m" },
  required_torque_nm: { label: "Par Req.", unit: "N·m" },
  safety_factor: { label: "Factor Seg.", unit: "×" },
};

export interface ProjectsSidebarProps {
  materials: Record<string, any>;
  printers: Record<string, any>;
  busy: boolean;
  onResult: (res: ProjectGenerateResponse, materialId: string) => void;
  onBusyChange: (label: string | null) => void;
  onToast: (msg: string, err?: boolean) => void;
}

export const ProjectsSidebar: React.FC<ProjectsSidebarProps> = ({
  materials,
  printers,
  busy,
  onResult,
  onBusyChange,
  onToast,
}) => {
  const [families, setFamilies] = useState<ProjectFamilyInfo[]>(FALLBACK_FAMILIES);
  const [familyId, setFamilyId] = useState<ProjectFamilyId>("fpv_drone");
  const [presetId, setPresetId] = useState<string>("freestyle_5in");
  const [materialId, setMaterialId] = useState<string>("pa12_cf");
  const [printerId, setPrinterId] = useState<string>("bambu_x1c");
  const [params, setParams] = useState<Record<string, any>>({
    prop_size_in: 5.0,
    plate_thickness_mm: 4.0,
    stack_mm: 30.5,
    battery_cells: 6,
  });

  useEffect(() => {
    api
      .getProjectFamilies()
      .then((res) => {
        if (res?.families?.length) {
          setFamilies(res.families);
        }
      })
      .catch(() => {
        /* keep fallback */
      });
  }, []);

  const activeFamily = useMemo(
    () => families.find((f) => f.id === familyId) || families[0],
    [families, familyId]
  );

  const selectFamily = (fid: ProjectFamilyId) => {
    setFamilyId(fid);
    const fam = families.find((f) => f.id === fid) || FALLBACK_FAMILIES.find((f) => f.id === fid);
    const first = fam?.presets?.[0];
    if (first) {
      applyPreset(fid, first);
    }
  };

  const applyPreset = (fid: ProjectFamilyId, preset: any) => {
    setFamilyId(fid);
    setPresetId(preset.id);
    if (preset.material_id) setMaterialId(preset.material_id);
    if (preset.printer_id) setPrinterId(preset.printer_id);
    const fallbackPreset = FALLBACK_FAMILIES.find((f) => f.id === fid)?.presets.find((p) => p.id === preset.id);
    setParams({ ...(fallbackPreset?.defaults || {}), ...(preset.defaults || {}) });
  };

  const handleGenerate = async () => {
    onBusyChange(`Generando ${activeFamily?.name || familyId} (build123d + Goldfish DFM)...`);
    try {
      const res = await api.generateProject(familyId, {
        preset_id: presetId,
        material_id: materialId,
        printer_id: printerId,
        params,
      });
      onResult(res, materialId);
      const dfm = res.dfm_audit?.status?.toUpperCase() || "OK";
      onToast(`${res.name} listo (DFM ${dfm} · BOM $${res.total_bom_cost_usd.toFixed(0)} USD).`);
    } catch (err: any) {
      onToast(`Error generando proyecto: ${err.message}`, true);
    } finally {
      onBusyChange(null);
    }
  };

  return (
    <>
      <div className="section-card">
        <div className="section-title">
          <span>Proyectos 3D Multi-Familia (v9.0)</span>
        </div>

        {/* Family selector */}
        <div style={{ display: "grid", gridTemplateColumns: "1fr 1fr 1fr", gap: 5, marginBottom: 8 }}>
          {families.map((fam) => (
            <button
              key={fam.id}
              type="button"
              className={`btn-tool ${familyId === fam.id ? "active" : ""}`}
              style={{
                padding: "7px 6px",
                fontSize: 11,
                textAlign: "center",
                whiteSpace: "normal",
                lineHeight: 1.25,
                minWidth: 0,
                overflow: "hidden",
              }}
              onClick={() => selectFamily(fam.id)}
            >
              <div>{fam.icon || "⚙"}</div>
              <div>{fam.name}</div>
            </button>
          ))}
        </div>

        {/* Presets */}
        <div className="preset-grid">
          {(activeFamily?.presets || []).map((p) => (
            <button
              key={p.id}
              type="button"
              className={`preset-btn ${presetId === p.id ? "active" : ""}`}
              onClick={() => applyPreset(familyId, p)}
            >
              <strong>{p.name}</strong>
              <small>{p.subtitle || ""}</small>
            </button>
          ))}
        </div>

        {/* Family-specific parametric sliders */}
        {familyId === "fpv_drone" && (
          <>
            <div className="field-row">
              <div className="field-head">
                <span>Tamaño Hélice</span>
                <span>{Number(params.prop_size_in ?? 5).toFixed(1)}&quot;</span>
              </div>
              <input
                type="range"
                min={3}
                max={7}
                step={0.5}
                value={Number(params.prop_size_in ?? 5)}
                onChange={(e) => setParams((p) => ({ ...p, prop_size_in: Number(e.target.value) }))}
              />
            </div>
            <div className="field-row">
              <div className="field-head">
                <span>Espesor Placa Base</span>
                <span>{Number(params.plate_thickness_mm ?? 4).toFixed(1)} mm</span>
              </div>
              <input
                type="range"
                min={2}
                max={5.5}
                step={0.5}
                value={Number(params.plate_thickness_mm ?? 4)}
                onChange={(e) => setParams((p) => ({ ...p, plate_thickness_mm: Number(e.target.value) }))}
              />
            </div>
            <div className="two-col">
              <div className="field-row">
                <div className="field-head"><span>Stack FC</span></div>
                <select
                  value={Number(params.stack_mm ?? 30.5)}
                  onChange={(e) => setParams((p) => ({ ...p, stack_mm: Number(e.target.value) }))}
                >
                  <option value={20}>20 × 20 mm</option>
                  <option value={30.5}>30.5 × 30.5 mm</option>
                </select>
              </div>
              <div className="field-row">
                <div className="field-head"><span>Batería LiPo</span></div>
                <select
                  value={Number(params.battery_cells ?? 6)}
                  onChange={(e) => setParams((p) => ({ ...p, battery_cells: Number(e.target.value) }))}
                >
                  <option value={3}>3S (11.1 V)</option>
                  <option value={4}>4S (14.8 V)</option>
                  <option value={6}>6S (22.2 V)</option>
                </select>
              </div>
            </div>
          </>
        )}

        {familyId === "rover_4wd" && (
          <>
            <div className="field-row">
              <div className="field-head">
                <span>Longitud Chasis</span>
                <span>{Math.round(Number(params.chassis_length_mm ?? 200))} mm</span>
              </div>
              <input
                type="range"
                min={140}
                max={340}
                step={10}
                value={Number(params.chassis_length_mm ?? 200)}
                onChange={(e) => setParams((p) => ({ ...p, chassis_length_mm: Number(e.target.value) }))}
              />
            </div>
            <div className="field-row">
              <div className="field-head">
                <span>Ancho Chasis</span>
                <span>{Math.round(Number(params.chassis_width_mm ?? 140))} mm</span>
              </div>
              <input
                type="range"
                min={100}
                max={240}
                step={10}
                value={Number(params.chassis_width_mm ?? 140)}
                onChange={(e) => setParams((p) => ({ ...p, chassis_width_mm: Number(e.target.value) }))}
              />
            </div>
            <div className="field-row">
              <div className="field-head">
                <span>Diámetro Rueda</span>
                <span>{Math.round(Number(params.wheel_diameter_mm ?? 75))} mm</span>
              </div>
              <input
                type="range"
                min={55}
                max={120}
                step={5}
                value={Number(params.wheel_diameter_mm ?? 75)}
                onChange={(e) => setParams((p) => ({ ...p, wheel_diameter_mm: Number(e.target.value) }))}
              />
            </div>
            <label className="check-row">
              <span>Torreta LiDAR / Cámara 360°</span>
              <input
                type="checkbox"
                checked={Boolean(params.lidar_turret)}
                onChange={(e) => setParams((p) => ({ ...p, lidar_turret: e.target.checked }))}
              />
            </label>
          </>
        )}

        {familyId === "camera_gimbal" && (
          <>
            <div className="two-col">
              <div className="field-row">
                <div className="field-head"><span>Ejes Estab.</span></div>
                <select
                  value={Number(params.axes ?? 2)}
                  onChange={(e) => setParams((p) => ({ ...p, axes: Number(e.target.value) }))}
                >
                  <option value={2}>2 Ejes (Roll/Pitch)</option>
                  <option value={3}>3 Ejes (Roll/Pitch/Yaw)</option>
                </select>
              </div>
              <div className="field-row">
                <div className="field-head"><span>Actuadores</span></div>
                <select
                  value={String(params.motor_type ?? "servo_mg90s")}
                  onChange={(e) => setParams((p) => ({ ...p, motor_type: e.target.value }))}
                >
                  <option value="servo_mg90s">Micro MG90S</option>
                  <option value="bldc_2204">Brushless 2204</option>
                </select>
              </div>
            </div>
            <div className="field-row">
              <div className="field-head">
                <span>Masa Cámara</span>
                <span>{Math.round(Number(params.camera_mass_g ?? 50))} g</span>
              </div>
              <input
                type="range"
                min={25}
                max={200}
                step={5}
                value={Number(params.camera_mass_g ?? 50)}
                onChange={(e) => setParams((p) => ({ ...p, camera_mass_g: Number(e.target.value) }))}
              />
            </div>
            <div className="field-row">
              <div className="field-head">
                <span>Ancho Cuna Cámara</span>
                <span>{Math.round(Number(params.cradle_width_mm ?? 48))} mm</span>
              </div>
              <input
                type="range"
                min={38}
                max={80}
                step={2}
                value={Number(params.cradle_width_mm ?? 48)}
                onChange={(e) => setParams((p) => ({ ...p, cradle_width_mm: Number(e.target.value) }))}
              />
            </div>
          </>
        )}

        <button
          type="button"
          className="btn-primary"
          style={{ width: "100%", marginTop: 10 }}
          disabled={busy}
          onClick={handleGenerate}
        >
          {busy ? "Generando CAD 3D..." : "Generar Proyecto 3D"}
        </button>
      </div>

      {/* Printer & Material */}
      <div className="section-card">
        <div className="section-title">
          <span>Impresora y Material del Proyecto</span>
        </div>
        <div className="field-row">
          <div className="field-head"><span>Impresora 3D</span></div>
          <select value={printerId} onChange={(e) => setPrinterId(e.target.value)}>
            {Object.entries(printers).map(([id, p]: [string, any]) => (
              <option key={id} value={id}>
                {p.name || id} ({p.process_type === "msla_resin" ? "MSLA Resina" : p.enclosed ? "Cerrada" : "Abierta"})
              </option>
            ))}
          </select>
        </div>
        <div className="field-row">
          <div className="field-head"><span>Material</span></div>
          <select value={materialId} onChange={(e) => setMaterialId(e.target.value)}>
            {Object.entries(materials).map(([id, m]: [string, any]) => (
              <option key={id} value={id}>
                {m.name || id}
              </option>
            ))}
          </select>
        </div>
      </div>
    </>
  );
};

export interface ProjectsInspectorProps {
  result: ProjectGenerateResponse | null;
}

export const ProjectsInspector: React.FC<ProjectsInspectorProps> = ({ result }) => {
  if (!result) {
    return (
      <div className="fea-card">
        <div className="pareto-head">
          <strong>Estudio Multi-Proyecto v9.0</strong>
          <span className="badge neutral">Listo</span>
        </div>
        <div className="mech-empty">
          Elige una familia (<strong>Drones FPV</strong>, <strong>Rovers 4WD</strong> o <strong>Gimbals 2–3 Ejes</strong>), ajusta sus parámetros y pulsa <strong>«Generar Proyecto 3D»</strong> para sintetizar el ensamble CAD B-Rep, la auditoría <strong>Goldfish DFM</strong> y los archivos listos para imprimir.
        </div>
      </div>
    );
  }

  const kpiEntries = Object.entries(result.metrics || {}).filter(
    ([, val]) => typeof val === "number" || typeof val === "string"
  );
  const partStls = Object.entries(result.files || {}).filter(([k]) => k.startsWith("part_stl_"));

  return (
    <>
      {/* Header + Physical KPIs */}
      <div className="fea-card">
        <div className="pareto-head">
          <strong>{result.name}</strong>
          <span className="badge good">${result.total_bom_cost_usd.toFixed(2)} USD</span>
        </div>

        <div className="metrics-grid">
          {kpiEntries.slice(0, 6).map(([k, val]) => {
            const meta = METRIC_LABELS[k];
            const label = meta?.label || k.replace(/_/g, " ");
            const formatted =
              typeof val === "number"
                ? `${Number.isInteger(val) ? val : val.toFixed(2)}${meta?.unit ? ` ${meta.unit}` : ""}`
                : String(val);
            return (
              <div key={k} className="metric-box">
                <span>{label}</span>
                <strong>{formatted}</strong>
              </div>
            );
          })}
        </div>

        {/* Downloads Block 1: Kit + CAD */}
        <div className="dl-row modular-dl-row" style={{ marginTop: 8 }}>
          {result.files?.zip && (
            <a className="dl-link dl-zip" href={result.files.zip} download>
              ★ Kit Proyecto (.ZIP)
            </a>
          )}
          {result.files?.stl_parts_zip && (
            <a className="dl-link dl-zip" href={result.files.stl_parts_zip} download>
              ★ Piezas 3D (.STL · ZIP)
            </a>
          )}
          {result.files?.step && (
            <a className="dl-link" href={result.files.step} download>
              .STEP
            </a>
          )}
          {result.files?.dxf && (
            <a className="dl-link" href={result.files.dxf} download>
              CNC .DXF
            </a>
          )}
          {result.files?.print_guide_md && (
            <a className="dl-link" href={result.files.print_guide_md} download>
              Guía Slicer (.MD)
            </a>
          )}
        </div>

        {partStls.length > 0 && (
          <div style={{ marginTop: 8, display: "grid", gridTemplateColumns: "1fr 1fr", gap: 5 }}>
            {partStls.map(([k, url]) => (
              <a key={k} className="dl-link" href={url} download>
                {k.replace(/^part_stl_/, "").replace(/_/g, " ")} (.STL)
              </a>
            ))}
          </div>
        )}
      </div>

      {/* Goldfish DFM Audit Card */}
      <DFMAuditCard audit={result.dfm_audit} />

      {/* Master BOM */}
      {result.bom?.length > 0 && (
        <div className="fea-card">
          <div className="pareto-head">
            <strong>Lista de Materiales (BOM)</strong>
            <span className="badge good">${result.total_bom_cost_usd.toFixed(2)}</span>
          </div>
          <div style={{ maxHeight: 220, overflowY: "auto", marginTop: 6 }}>
            <table className="modular-parts-table">
              <thead>
                <tr>
                  <th>Componente</th>
                  <th>Cant.</th>
                  <th>Total</th>
                </tr>
              </thead>
              <tbody>
                {result.bom.map((it, idx) => (
                  <tr key={`${it.part_code}-${idx}`}>
                    <td>
                      <strong>{it.part_name}</strong>
                      <span className="part-var-sub">{it.spec}</span>
                    </td>
                    <td className="mono">{it.qty}×</td>
                    <td className="mono">${it.total_cost_usd.toFixed(2)}</td>
                  </tr>
                ))}
              </tbody>
            </table>
          </div>
        </div>
      )}
    </>
  );
};
