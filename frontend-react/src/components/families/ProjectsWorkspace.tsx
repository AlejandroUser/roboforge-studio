import React, { useEffect, useMemo, useState } from "react";
import { api } from "../../api";
import type {
  ProjectFamilyId,
  ProjectFamilyInfo,
  ProjectGenerateResponse,
  PropellerEvolveResponse,
  PropellerWinner,
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
  {
    id: "propeller",
    name: "Hélices BEMT",
    subtitle: "Aerodinámica BEMT · Evolución NSGA-II · Par CCW+CW",
    icon: "🌀",
    presets: [
      {
        id: "fpv_5043_tri",
        name: "FPV 5043 Tri-Pala",
        subtitle: "5.0\" · 3 Palas · 18k RPM",
        material_id: "pa12_cf",
        printer_id: "bambu_x1c",
        defaults: { diameter_inch: 5.0, rpm: 18000, blade_count: 3, prop_style: "standard", target_thrust_g: 480 },
      },
      {
        id: "cinewhoop_3020_quad",
        name: "Cinewhoop 3\" Silenciosa",
        subtitle: "3.0\" · 4 Palas · Toroidal",
        material_id: "resin_abs_like",
        printer_id: "elegoo_mars_5_ultra",
        defaults: { diameter_inch: 3.0, rpm: 22000, blade_count: 4, prop_style: "toroidal", target_thrust_g: 260 },
      },
      {
        id: "longrange_7035_bi",
        name: "Long-Range 7\" Bi-Pala",
        subtitle: "7.0\" · 2 Palas · Máx. Eficiencia",
        material_id: "pa12_cf",
        printer_id: "bambu_x1c",
        defaults: { diameter_inch: 7.0, rpm: 12000, blade_count: 2, prop_style: "standard", target_thrust_g: 680 },
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
  thrust_g: { label: "Empuje", unit: "g" },
  efficiency_g_per_w: { label: "Eficiencia", unit: "g/W" },
  noise_dba: { label: "Ruido Est.", unit: "dBA" },
  root_sf: { label: "FS Raíz", unit: "×" },
  mass_savings_pct: { label: "Ahorro Masa", unit: "%" },
  volume_fraction_pct: { label: "Volumen", unit: "%" },
  volume_fraction: { label: "Frac. Vol.", unit: "" },
  triangle_count: { label: "Triángulos", unit: "" },
  max_slope_deg: { label: "Pendiente Máx.", unit: "°" },
  climb_torque_nm: { label: "Torque 20°", unit: "N·m" },
  suspension_mode: { label: "Suspensión", unit: "" },
  pitch_com_offset_mm: { label: "CoM Pitch", unit: "mm" },
  roll_com_offset_mm: { label: "CoM Roll", unit: "mm" },
  recommended_counterweight_g: { label: "Contrapeso", unit: "g" },
  idle_torque_reduction_pct: { label: "Ahorro Par", unit: "%" },
};

export interface ProjectsSidebarProps {
  materials: Record<string, any>;
  printers: Record<string, any>;
  busy: boolean;
  initialMode?: "families" | "simp3d";
  onResult: (res: ProjectGenerateResponse, materialId: string) => void;
  onBusyChange: (label: string | null) => void;
  onToast: (msg: string, err?: boolean) => void;
}

export const ProjectsSidebar: React.FC<ProjectsSidebarProps> = ({
  materials,
  printers,
  busy,
  initialMode,
  onResult,
  onBusyChange,
  onToast,
}) => {
  const [families, setFamilies] = useState<ProjectFamilyInfo[]>(FALLBACK_FAMILIES);
  const [familyId, setFamilyId] = useState<ProjectFamilyId>("fpv_drone");
  const [generativeMode, setGenerativeMode] = useState<"families" | "simp3d">(initialMode || "families");
  const [customAirfoilInfo, setCustomAirfoilInfo] = useState<{
    id: string;
    name: string;
    thickness_pct: number;
    camber_pct: number;
  } | null>(null);
  const [presetId, setPresetId] = useState<string>("freestyle_5in");
  const [materialId, setMaterialId] = useState<string>("pa12_cf");
  const [printerId, setPrinterId] = useState<string>("bambu_x1c");
  const [params, setParams] = useState<Record<string, any>>({
    prop_size_in: 5.0,
    plate_thickness_mm: 4.0,
    stack_mm: 30.5,
    battery_cells: 6,
  });

  // v13.0 Propeller NSGA-II Genetic Evolution state
  const [evolveRes, setEvolveRes] = useState<PropellerEvolveResponse | null>(null);
  const [activeWinnerId, setActiveWinnerId] = useState<string | null>(null);

  // v13.0 Organic SIMP 3D Topology Optimization state
  const [simpParams, setSimpParams] = useState({
    length_mm: 180,
    root_width_mm: 38,
    tip_width_mm: 24,
    thickness_mm: 22,
    payload_kg: 2.0,
    angular_accel_rad_s2: 15,
    volume_fraction: 0.38,
    overhang_filter_45: true,
  });

  useEffect(() => {
    if (initialMode) {
      setGenerativeMode(initialMode);
    }
  }, [initialMode]);

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
    setGenerativeMode("families");
    setFamilyId(fid);
    setEvolveRes(null);
    setActiveWinnerId(null);
    const fam = families.find((f) => f.id === fid) || FALLBACK_FAMILIES.find((f) => f.id === fid);
    const first = fam?.presets?.[0];
    if (first) {
      applyPreset(fid, first);
    }
  };

  const applyPreset = (fid: ProjectFamilyId, preset: any) => {
    setGenerativeMode("families");
    setFamilyId(fid);
    setPresetId(preset.id);
    setEvolveRes(null);
    setActiveWinnerId(null);
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

  const handleEvolvePropeller = async () => {
    onBusyChange("Evolucionando hélices por física BEMT (NSGA-II)...");
    try {
      const res = await api.evolvePropeller({
        target_thrust_g: Number(params.target_thrust_g ?? 480),
        diameter_inch: Number(params.diameter_inch ?? 5.0),
        rpm: Number(params.rpm ?? 18000),
        blade_count: Number(params.blade_count ?? 3),
        prop_style: String(params.prop_style ?? "standard"),
        material_id: materialId,
      });
      setEvolveRes(res);
      setActiveWinnerId(null);
      onToast(
        `NSGA-II completado: ${res.evaluations} hélices evaluadas en ${res.elapsed_s.toFixed(2)}s (${res.winners.length} ganadoras Pareto).`
      );
    } catch (err: any) {
      onToast(`Error evolucionando hélices: ${err.message}`, true);
    } finally {
      onBusyChange(null);
    }
  };

  const handleBuildEvolvedWinner = async (w: PropellerWinner) => {
    setActiveWinnerId(w.candidate_id);
    onBusyChange(`Sintetizando par CCW+CW «${w.label}» (build123d)...`);
    try {
      const res = await api.generateProject("propeller", {
        preset_id: presetId || "fpv_5043_tri",
        material_id: materialId,
        printer_id: printerId,
        params: {
          diameter_inch: Number(params.diameter_inch ?? 5.0),
          rpm: Number(params.rpm ?? 18000),
          blade_count: Number(params.blade_count ?? 3),
          airfoil_id: w.airfoil_id,
          prop_style: String(params.prop_style ?? "standard"),
          sweep_deg: w.sweep_deg,
          rake_deg: w.rake_deg,
          custom_stations: w.stations,
          evolved_label: w.label,
          evolved_metrics: w.metrics,
        },
      });
      onResult(res, materialId);
      onToast(`Par CCW+CW «${w.label}» renderizado en el visor 3D.`);
    } catch (err: any) {
      onToast(`Error sintetizando hélice ganadora: ${err.message}`, true);
    } finally {
      onBusyChange(null);
    }
  };

  const handleGenerateSimp3D = async () => {
    onBusyChange("Esculpiendo eslabón orgánico SIMP 3D (TopOpt Volumétrico + Langelaar 45°)...");
    try {
      const res = await api.generateSimp3DLink({
        ...simpParams,
        material_id: materialId,
        printer_id: printerId,
      });
      const filesMap: Record<string, string> = {
        glb: res.glb_url,
        stl: res.stl_url,
      };
      if (res.threemf_url) filesMap.threemf = res.threemf_url;

      const adapted: ProjectGenerateResponse = {
        family_id: "simp3d_generative",
        preset_id: "simp3d_link",
        design_hash: res.design_hash,
        name: res.name,
        metrics: res.metrics || {},
        bom: [],
        total_bom_cost_usd: Number(res.metrics?.estimated_cost_usd ?? 0),
        dfm_audit: res.dfm_audit,
        explode_vectors: {},
        files: filesMap,
      };
      onResult(adapted, materialId);
      const savePct = res.metrics?.mass_savings_pct;
      onToast(
        `${res.name} esculpido${typeof savePct === "number" ? ` (-${savePct.toFixed(1)}% masa)` : ""} · DFM ${res.dfm_audit?.status?.toUpperCase() || "PASS"}.`
      );
    } catch (err: any) {
      onToast(`Error en SIMP 3D: ${err.message}`, true);
    } finally {
      onBusyChange(null);
    }
  };

  return (
    <>
      <div className="section-card">
        <div className="section-title">
          <span>Proyectos 3D &amp; Generativo (v13.0)</span>
        </div>

        {/* Family selector + Generative SIMP 3D button */}
        <div style={{ display: "grid", gridTemplateColumns: "1fr 1fr", gap: 5, marginBottom: 8 }}>
          {families.map((fam) => (
            <button
              key={fam.id}
              type="button"
              className={`btn-tool ${generativeMode === "families" && familyId === fam.id ? "active" : ""}`}
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
          <button
            type="button"
            className={`btn-tool ${generativeMode === "simp3d" ? "active" : ""}`}
            style={{
              gridColumn: "1 / -1",
              padding: "8px 8px",
              fontSize: 11.5,
              textAlign: "center",
              background:
                generativeMode === "simp3d"
                  ? "rgba(16, 185, 129, 0.22)"
                  : "rgba(56, 189, 248, 0.12)",
              borderColor:
                generativeMode === "simp3d"
                  ? "rgba(16, 185, 129, 0.65)"
                  : "rgba(56, 189, 248, 0.42)",
            }}
            onClick={() => setGenerativeMode("simp3d")}
          >
            🧬 Generativo SIMP 3D (Eslabón Orgánico TopOpt)
          </button>
        </div>

        {/* Presets (only in standard family mode) */}
        {generativeMode === "families" && (
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
        )}

        {/* Family-specific parametric sliders */}
        {generativeMode === "families" && familyId === "fpv_drone" && (
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

        {generativeMode === "families" && familyId === "rover_4wd" && (
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
            <div className="field-row">
              <div className="field-head"><span>Suspensión / Dirección</span></div>
              <select
                value={String(params.suspension_mode ?? "rigid")}
                onChange={(e) => setParams((p) => ({ ...p, suspension_mode: e.target.value }))}
              >
                <option value="rigid">Rígida 4WD</option>
                <option value="rocker_bogie">Rocker-Bogie (Basculante)</option>
                <option value="ackerman">Dirección Ackerman</option>
              </select>
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

        {generativeMode === "families" && familyId === "camera_gimbal" && (
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

        {generativeMode === "families" && familyId === "propeller" && (
          <>
            <div className="field-row">
              <div className="field-head">
                <span>Empuje Objetivo</span>
                <span>{Math.round(Number(params.target_thrust_g ?? 480))} g</span>
              </div>
              <input
                type="range"
                min={100}
                max={1500}
                step={20}
                value={Number(params.target_thrust_g ?? 480)}
                onChange={(e) => setParams((p) => ({ ...p, target_thrust_g: Number(e.target.value) }))}
              />
            </div>
            <div className="field-row">
              <div className="field-head">
                <span>Diámetro Hélice</span>
                <span>{Number(params.diameter_inch ?? 5.0).toFixed(1)}&quot;</span>
              </div>
              <input
                type="range"
                min={2.5}
                max={8.0}
                step={0.5}
                value={Number(params.diameter_inch ?? 5.0)}
                onChange={(e) => setParams((p) => ({ ...p, diameter_inch: Number(e.target.value) }))}
              />
            </div>
            <div className="field-row">
              <div className="field-head">
                <span>Régimen RPM</span>
                <span>{Math.round(Number(params.rpm ?? 18000)).toLocaleString("es")} RPM</span>
              </div>
              <input
                type="range"
                min={8000}
                max={28000}
                step={500}
                value={Number(params.rpm ?? 18000)}
                onChange={(e) => setParams((p) => ({ ...p, rpm: Number(e.target.value) }))}
              />
            </div>
            <div className="two-col">
              <div className="field-row">
                <div className="field-head"><span>Palas</span></div>
                <select
                  value={Number(params.blade_count ?? 3)}
                  onChange={(e) => setParams((p) => ({ ...p, blade_count: Number(e.target.value) }))}
                >
                  <option value={2}>2 Palas</option>
                  <option value={3}>3 Palas</option>
                  <option value={4}>4 Palas</option>
                </select>
              </div>
              <div className="field-row">
                <div className="field-head"><span>Estilo</span></div>
                <select
                  value={String(params.prop_style ?? "standard")}
                  onChange={(e) => setParams((p) => ({ ...p, prop_style: e.target.value }))}
                >
                  <option value="standard">Estándar</option>
                  <option value="toroidal">Toroidal</option>
                  <option value="scimitar">Cimitarra</option>
                </select>
              </div>
            </div>

            {/* Fase 4: Importador de Perfiles .dat UIUC */}
            <div className="field-row" style={{ marginTop: 4 }}>
              <label
                className="btn-tool"
                style={{ display: "block", textAlign: "center", cursor: "pointer" }}
                title="Cargar perfil aerodinámico en formato Selig o Lednicer (.dat UIUC)"
              >
                📄 Importar Perfil .dat (UIUC)
                <input
                  type="file"
                  accept=".dat,.txt"
                  hidden
                  onChange={async (e) => {
                    const file = e.target.files?.[0];
                    e.target.value = "";
                    if (!file) return;
                    const cleanId =
                      file.name
                        .replace(/\.[^/.]+$/, "")
                        .toLowerCase()
                        .replace(/[^a-z0-9_]+/g, "_")
                        .replace(/^_|_$/g, "") || "custom_dat";
                    onBusyChange(`Importando perfil aerodinámico ${file.name}...`);
                    try {
                      const datText = await file.text();
                      const res = await api.importAirfoilDat(cleanId, datText);
                      setCustomAirfoilInfo({
                        id: res.id,
                        name: res.name,
                        thickness_pct: res.thickness_pct,
                        camber_pct: res.camber_pct,
                      });
                      setParams((p) => ({ ...p, airfoil_id: res.id }));
                      onToast(
                        `Perfil ${res.name} importado (espesor ${res.thickness_pct.toFixed(1)}%, curvatura ${res.camber_pct.toFixed(1)}%).`
                      );
                    } catch (err: any) {
                      onToast(`Error importando .dat: ${err.message}`, true);
                    } finally {
                      onBusyChange(null);
                    }
                  }}
                />
              </label>
              {customAirfoilInfo && (
                <div className="alert-item info" style={{ marginTop: 4, fontSize: 10.5 }}>
                  <strong>Perfil activo:</strong> {customAirfoilInfo.name} · espesor{" "}
                  {customAirfoilInfo.thickness_pct.toFixed(1)}% · curvatura{" "}
                  {customAirfoilInfo.camber_pct.toFixed(1)}%
                </div>
              )}
            </div>

            <button
              type="button"
              className="btn-primary"
              style={{
                width: "100%",
                marginTop: 8,
                background: "linear-gradient(135deg, #059669, #10b981)",
              }}
              disabled={busy}
              onClick={handleEvolvePropeller}
            >
              {busy ? "Evolucionando NSGA-II..." : "🧬 Evolucionar Hélices por Física (NSGA-II)"}
            </button>

            {evolveRes && (
              <div style={{ marginTop: 8, display: "flex", flexDirection: "column", gap: 6 }}>
                <div style={{ fontSize: 10.5, color: "#94a3b8", display: "flex", justifyContent: "space-between" }}>
                  <span>
                    Baseline: {Number(evolveRes.baseline_metrics?.efficiency_g_per_w ?? 0).toFixed(2)} g/W ·{" "}
                    {Number(evolveRes.baseline_metrics?.noise_dba ?? 0).toFixed(1)} dBA
                  </span>
                  <span>
                    {evolveRes.evaluations} eval · {evolveRes.elapsed_s.toFixed(1)}s
                  </span>
                </div>
                {evolveRes.winners.map((w) => {
                  const eff = Number(w.metrics?.efficiency_g_per_w ?? 0);
                  const dba = Number(w.metrics?.noise_dba ?? 0);
                  const thr = Number(w.metrics?.thrust_g ?? 0);
                  const sf = Number(w.metrics?.root_sf ?? 0);
                  const isSel = activeWinnerId === w.candidate_id;
                  return (
                    <button
                      key={w.candidate_id}
                      type="button"
                      className={`preset-btn ${isSel ? "active" : ""}`}
                      style={{ width: "100%", padding: "8px 9px" }}
                      disabled={busy}
                      onClick={() => handleBuildEvolvedWinner(w)}
                    >
                      <div style={{ display: "flex", justifyContent: "space-between", alignItems: "center", width: "100%" }}>
                        <strong>{w.label}</strong>
                        <span className="badge good">{eff.toFixed(2)} g/W</span>
                      </div>
                      <small style={{ color: "#cbd5e1" }}>{w.summary}</small>
                      <small style={{ fontFamily: "var(--mono)", color: "#7dd3fc" }}>
                        {Math.round(thr)} g · {dba.toFixed(1)} dBA · FS {sf.toFixed(1)}× · Sweep {w.sweep_deg.toFixed(0)}°
                      </small>
                    </button>
                  );
                })}
              </div>
            )}
          </>
        )}

        {generativeMode === "simp3d" && (
          <>
            <div className="field-row">
              <div className="field-head">
                <span>Longitud Eslabón</span>
                <span>{simpParams.length_mm} mm</span>
              </div>
              <input
                type="range"
                min={100}
                max={320}
                step={10}
                value={simpParams.length_mm}
                onChange={(e) => setSimpParams((p) => ({ ...p, length_mm: Number(e.target.value) }))}
              />
            </div>
            <div className="field-row">
              <div className="field-head">
                <span>Carga Útil (Punta)</span>
                <span>{simpParams.payload_kg.toFixed(1)} kg</span>
              </div>
              <input
                type="range"
                min={0.5}
                max={6.0}
                step={0.25}
                value={simpParams.payload_kg}
                onChange={(e) => setSimpParams((p) => ({ ...p, payload_kg: Number(e.target.value) }))}
              />
            </div>
            <div className="field-row">
              <div className="field-head">
                <span>Aceleración Angular</span>
                <span>{simpParams.angular_accel_rad_s2} rad/s²</span>
              </div>
              <input
                type="range"
                min={5}
                max={30}
                step={1}
                value={simpParams.angular_accel_rad_s2}
                onChange={(e) => setSimpParams((p) => ({ ...p, angular_accel_rad_s2: Number(e.target.value) }))}
              />
            </div>
            <div className="field-row">
              <div className="field-head">
                <span>Fracción de Volumen</span>
                <span>{Math.round(simpParams.volume_fraction * 100)}%</span>
              </div>
              <input
                type="range"
                min={25}
                max={55}
                step={1}
                value={Math.round(simpParams.volume_fraction * 100)}
                onChange={(e) => setSimpParams((p) => ({ ...p, volume_fraction: Number(e.target.value) / 100 }))}
              />
            </div>
            <label className="check-row">
              <span>Filtro Voladizos Langelaar 45° (Sin Soportes)</span>
              <input
                type="checkbox"
                checked={simpParams.overhang_filter_45}
                onChange={(e) => setSimpParams((p) => ({ ...p, overhang_filter_45: e.target.checked }))}
              />
            </label>
            <button
              type="button"
              className="btn-primary"
              style={{
                width: "100%",
                marginTop: 10,
                background: "linear-gradient(135deg, #059669, #10b981)",
              }}
              disabled={busy}
              onClick={handleGenerateSimp3D}
            >
              {busy ? "Esculpiendo SIMP 3D..." : "🧬 Esculpir Eslabón Orgánico 3D"}
            </button>
          </>
        )}

        {generativeMode === "families" && (
        <button
          type="button"
          className="btn-primary"
          style={{ width: "100%", marginTop: 10 }}
          disabled={busy}
          onClick={handleGenerate}
        >
          {busy ? "Generando CAD 3D..." : "Generar Proyecto 3D"}
        </button>
        )}
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
          {result.files?.stl && (
            <a className="dl-link dl-zip" href={result.files.stl} download>
              ★ Descargar .STL
            </a>
          )}
          {result.files?.threemf && (
            <a className="dl-link" href={result.files.threemf} download>
              .3MF
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
