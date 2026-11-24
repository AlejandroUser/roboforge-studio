import React, { useMemo, useState } from "react";
import { api } from "../api";
import { LiveHardwarePanel } from "./LiveHardwarePanel";
import type {
  BusArchitecture,
  CaliperCalibrateResponse,
  JointActuatorSpec,
  JointMechatronicsReport,
  LiveArmPose,
  MasterBomCategory,
  MasterBomItem,
  MechatronicsCatalog,
  MechatronicsConfig,
  MechatronicsResponse,
  PrintReadinessSummary,
} from "../types";

const FALLBACK_ACTUATORS = [
  { id: "nema17_48mm", name: "NEMA 17 48mm (Stepper)" },
  { id: "nema17_mks_servo42d", name: "NEMA 17 MKS SERVO42D (Closed-Loop)" },
  { id: "nema23_76mm", name: "NEMA 23 76mm (Stepper)" },
  { id: "nema23_mks_servo57d", name: "NEMA 23 MKS SERVO57D (Closed-Loop)" },
  { id: "bldc_qdd_5010", name: "BLDC QDD 5010 (Direct/QDD)" },
  { id: "bldc_qdd_8308", name: "BLDC QDD 8308 (High-Torque QDD)" },
  { id: "servo_td8120mg", name: "TD-8120MG 20kg·cm (Metal Gear Servo)" },
  { id: "servo_ds3218", name: "DS3218MG 20kg·cm (Digital Servo)" },
  { id: "servo_mg996r", name: "MG996R 10kg·cm Metal Gear ($3.50)" },
  { id: "servo_mg90s", name: "MG90S Micro 9g Metal Gear ($2.00)" },
  { id: "servo_sts3215", name: "Feetech STS3215 30kg·cm (UART Bus)" },
  { id: "damiao_dm_j4310", name: "Damiao DM-J4310 QDD (7 N·m)" },
  { id: "damiao_dm_j4340", name: "Damiao DM-J4340 QDD (27 N·m)" },
  { id: "nema14_34mm", name: "NEMA 14 34mm (Compact Stepper)" },
];

const FALLBACK_REDUCERS = [
  { id: "direct", name: "Directo 1:1 (98%)" },
  { id: "gt2_belt_3", name: "Correa GT2 3:1 (94%)" },
  { id: "planetary_10", name: "Planetario 10:1 (88%)" },
  { id: "planetary_20", name: "Planetario 20:1 (85%)" },
  { id: "cycloidal_25", name: "Cicloidal 25:1 (78%)" },
  { id: "cycloidal_40", name: "Cicloidal 40:1 (74%)" },
  { id: "harmonic_50", name: "Armónico 50:1 (80%)" },
];

const CATEGORY_ORDER: Array<{ key: MasterBomCategory; title: string }> = [
  { key: "printed_cnc", title: "Piezas Impresas / CNC" },
  { key: "actuators_reducers", title: "Motores y Reductores" },
  { key: "electronics_wiring", title: "Electrónica y Cableado" },
  { key: "fasteners_hardware", title: "Tornillería ISO y Rodamientos" },
];

export interface MechatronicsPanelProps {
  config: MechatronicsConfig;
  catalog: MechatronicsCatalog | null;
  mechatronics: MechatronicsResponse | null | undefined;
  printReadiness?: PrintReadinessSummary | null;
  files: Record<string, string> | undefined;
  livePose?: LiveArmPose;
  onChangeConfig: (next: MechatronicsConfig) => void;
}

function TorqueSpeedChart({
  report,
  parallelCount,
}: {
  report: JointMechatronicsReport;
  parallelCount: number;
}) {
  const curve = report.torque_speed_curve || [];
  if (curve.length < 2) {
    return <div className="mech-empty">Sin puntos de curva Torque–RPM para {report.joint_id}.</div>;
  }

  const W = 310;
  const H = 162;
  const padL = 38;
  const padR = 12;
  const padT = 22;
  const padB = 26;
  const plotW = W - padL - padR;
  const plotH = H - padT - padB;

  const maxRpm = Math.max(...curve.map((p) => p.rpm), report.operating_rpm * 1.15, 10);
  const maxTorque = Math.max(
    ...curve.map((p) => p.peak_torque_nm),
    report.required_peak_torque_nm * 1.15,
    1
  );

  const toX = (rpm: number) => padL + (Math.min(rpm, maxRpm) / maxRpm) * plotW;
  const toY = (t: number) => padT + plotH - (Math.min(t, maxTorque) / maxTorque) * plotH;

  const peakPath = curve
    .map((p, i) => `${i === 0 ? "M" : "L"}${toX(p.rpm).toFixed(1)},${toY(p.peak_torque_nm).toFixed(1)}`)
    .join(" ");

  const contPath = curve
    .map((p, i) => `${i === 0 ? "M" : "L"}${toX(p.rpm).toFixed(1)},${toY(p.continuous_torque_nm).toFixed(1)}`)
    .join(" ");

  const contArea = `${contPath} L${toX(curve[curve.length - 1].rpm).toFixed(1)},${(padT + plotH).toFixed(
    1
  )} L${toX(curve[0].rpm).toFixed(1)},${(padT + plotH).toFixed(1)} Z`;

  const opX = toX(report.operating_rpm);
  const opY = toY(report.required_peak_torque_nm);
  const opRmsY = toY(report.required_rms_torque_nm);
  const dotColor =
    report.status === "ok" ? "#10b981" : report.status === "warning" ? "#f59e0b" : "#ef4444";
  const isParallel = (report.parallel_count ?? parallelCount) === 2;

  return (
    <div className="torque-chart-wrap">
      <div className="torque-chart-head">
        <span>
          <strong>{report.joint_name}</strong> · {report.motor_name}
        </span>
        <div className="badge-row">
          {isParallel && <span className="badge badge-parallel">×2 Paralelo</span>}
          <span className={`badge ${report.status === "ok" ? "good" : report.status === "warning" ? "warn" : "bad"}`}>
            {report.status === "ok" ? "✓ OK" : report.status === "warning" ? "⚠ Límite" : "✕ Saturado"}
          </span>
        </div>
      </div>

      <svg viewBox={`0 0 ${W} ${H}`} className="torque-svg" role="img" aria-label={`Curva Torque RPM ${report.joint_id}`}>
        <defs>
          <linearGradient id={`contGrad-${report.joint_id}`} x1="0" y1="0" x2="0" y2="1">
            <stop offset="0%" stopColor="#38bdf8" stopOpacity="0.28" />
            <stop offset="100%" stopColor="#38bdf8" stopOpacity="0.02" />
          </linearGradient>
        </defs>

        {/* Axis title clearly above top gridline (never overlaps orange Peak curve) */}
        <text x={padL - 4} y={11} textAnchor="end" fontSize="8.5" fill="#94a3b8" fontFamily="monospace">
          N·m
        </text>

        {/* Grid lines */}
        {[0, 0.5, 1].map((frac) => {
          const y = padT + plotH * (1 - frac);
          return (
            <g key={frac}>
              <line x1={padL} y1={y} x2={W - padR} y2={y} stroke="rgba(148,163,184,0.14)" strokeDasharray="2 2" />
              <text x={padL - 5} y={y + 3} textAnchor="end" fontSize="8.5" fill="#64748b" fontFamily="monospace">
                {(maxTorque * frac).toFixed(1)}
              </text>
            </g>
          );
        })}

        {/* Axes */}
        <line x1={padL} y1={padT} x2={padL} y2={padT + plotH} stroke="#334155" />
        <line x1={padL} y1={padT + plotH} x2={W - padR} y2={padT + plotH} stroke="#334155" />

        {/* X ticks */}
        {[0, 0.5, 1].map((frac) => (
          <text
            key={frac}
            x={padL + plotW * frac}
            y={H - 8}
            textAnchor={frac === 0 ? "start" : frac === 1 ? "end" : "middle"}
            fontSize="8.5"
            fill="#64748b"
            fontFamily="monospace"
          >
            {Math.round(maxRpm * frac)} RPM
          </text>
        ))}

        {/* Continuous area & line */}
        <path d={contArea} fill={`url(#contGrad-${report.joint_id})`} />
        <path d={contPath} fill="none" stroke="#38bdf8" strokeWidth="1.8" />

        {/* Peak dashed line */}
        <path d={peakPath} fill="none" stroke="#f59e0b" strokeWidth="1.4" strokeDasharray="4 2.5" />

        {/* Operating RMS & Peak points */}
        <line x1={opX} y1={opY} x2={opX} y2={padT + plotH} stroke={dotColor} strokeOpacity="0.4" strokeDasharray="2 2" />
        <circle cx={opX} cy={opRmsY} r="3" fill="#38bdf8" stroke="#0b0f17" strokeWidth="1" />
        <circle cx={opX} cy={opY} r="4.5" fill={dotColor} stroke="#0b0f17" strokeWidth="1.4" />
      </svg>

      <div className="torque-legend-row">
        <span><i style={{ background: "#f59e0b" }} /> Pico</span>
        <span><i style={{ background: "#38bdf8" }} /> Cont. RMS</span>
        <span><i style={{ background: dotColor }} /> Op: {report.operating_rpm.toFixed(0)} RPM / {report.required_peak_torque_nm.toFixed(1)} N·m</span>
      </div>

      <div className="metrics-grid">
        <div className="metric-box">
          <span>Margen Torque</span>
          <strong className={report.torque_margin_pct >= 20 ? "good" : report.torque_margin_pct >= 0 ? "warn" : "bad"}>
            {report.torque_margin_pct >= 0 ? "+" : ""}{report.torque_margin_pct.toFixed(0)}%
          </strong>
        </div>
        <div className="metric-box">
          <span>Margen Térmico</span>
          <strong className={report.thermal_margin_pct >= 20 ? "good" : report.thermal_margin_pct >= 0 ? "warn" : "bad"}>
            {report.thermal_margin_pct >= 0 ? "+" : ""}{report.thermal_margin_pct.toFixed(0)}%
          </strong>
        </div>
        <div className="metric-box">
          <span>Corriente</span>
          <strong>{report.continuous_current_a.toFixed(1)} / {report.peak_current_a.toFixed(1)} A</strong>
        </div>
      </div>
    </div>
  );
}

export const MechatronicsPanel: React.FC<MechatronicsPanelProps> = ({
  config,
  catalog,
  mechatronics,
  printReadiness,
  files,
  livePose,
  onChangeConfig,
}) => {
  const [subTab, setSubTab] = useState<"motors" | "circuit" | "bom" | "print" | "hardware">("motors");
  const [selectedJointId, setSelectedJointId] = useState<string>("J1");
  const [showSvgModal, setShowSvgModal] = useState<boolean>(false);

  // v8.0 Caliper Calibration Assistant state
  const [measuredM3, setMeasuredM3] = useState<number>(3.2);
  const [measuredBore, setMeasuredBore] = useState<number>(22.2);
  const [caliperBusy, setCaliperBusy] = useState<boolean>(false);
  const [caliperResult, setCaliperResult] = useState<CaliperCalibrateResponse | null>(null);

  const actuators = catalog?.actuators?.length ? catalog.actuators : FALLBACK_ACTUATORS;
  const reducers = catalog?.reducers?.length ? catalog.reducers : FALLBACK_REDUCERS;

  const updateJointSpec = (jointId: string, patch: Partial<JointActuatorSpec>) => {
    const prevSpec: JointActuatorSpec = config.joint_actuators[jointId] || {
      motor_id: jointId === "J1" ? "nema23_mks_servo57d" : "nema17_mks_servo42d",
      reducer_id: jointId === "J1" ? "planetary_20" : "planetary_10",
      target_rpm: 30,
    };
    onChangeConfig({
      ...config,
      joint_actuators: {
        ...config.joint_actuators,
        [jointId]: { ...prevSpec, ...patch },
      },
    });
  };

  const activeReport = useMemo(() => {
    if (!mechatronics?.joint_reports?.length) return null;
    return (
      mechatronics.joint_reports.find((r) => r.joint_id === selectedJointId) ||
      mechatronics.joint_reports[0]
    );
  }, [mechatronics, selectedJointId]);

  const groupedBom = useMemo(() => {
    const groups: Record<MasterBomCategory, MasterBomItem[]> = {
      printed_cnc: [],
      actuators_reducers: [],
      electronics_wiring: [],
      fasteners_hardware: [],
    };
    for (const item of mechatronics?.master_bom || []) {
      if (groups[item.category]) {
        groups[item.category].push(item);
      }
    }
    return groups;
  }, [mechatronics]);

  const pb = mechatronics?.power_budget;
  const circuitSvgUrl = files?.circuit_svg;
  const harnessYmlUrl = files?.harness_yml;
  const masterBomCsvUrl = files?.master_bom_csv;
  const firmwareInoUrl = files?.firmware_ino;
  const firmwareEsp32InoUrl = files?.firmware_esp32_ino;
  const cncDxfUrl = files?.cnc_dxf;
  const lerobotJsonUrl = files?.lerobot_json;
  const calibrationStlUrl = files?.calibration_stl;
  const printGuideMdUrl = files?.print_guide_md;
  const isMslaResin = printReadiness?.process_type === "msla_resin";

  const formatVoltage = (v: number) =>
    Number.isInteger(v) ? `${v.toFixed(0)} V DC` : `${v.toFixed(1)} V DC`;

  const handleCalibrateCaliper = async () => {
    setCaliperBusy(true);
    try {
      const res = await api.calibrateCaliper({
        measured_m3_mm: measuredM3,
        measured_bore_mm: measuredBore,
        nominal_m3_mm: 3.2,
        nominal_bore_mm: 22.2,
        process_type: isMslaResin ? "msla_resin" : "fdm",
      });
      setCaliperResult(res);
    } catch {
      // Offline analytical fallback matching backend formula
      const errM3 = measuredM3 - 3.2;
      const errBore = measuredBore - 22.2;
      const meanErr = (errM3 + errBore) / 2;
      const comp = Number((-meanErr / 2).toFixed(3));
      const status: "perfect" | "tight" | "loose" =
        Math.abs(meanErr) <= 0.08 ? "perfect" : meanErr < 0 ? "tight" : "loose";
      setCaliperResult({
        status,
        xy_hole_compensation_mm: comp,
        recommended_clearance_mm: Number(Math.max(0.05, Math.min(0.45, 0.15 - meanErr / 2)).toFixed(2)),
        exposure_delta_s: isMslaResin ? Number((meanErr < -0.08 ? -0.15 : meanErr > 0.08 ? 0.15 : 0).toFixed(2)) : null,
        diagnosis:
          status === "perfect"
            ? "Tolerancias dentro de ±0.08 mm. Listo para imprimir el brazo completo sin ajustes."
            : status === "tight"
              ? `Agujeros apretados (${meanErr.toFixed(2)} mm). Aplica compensación XY positiva${isMslaResin ? " o reduce exposición -0.15 s en SatelLite/Chitubox" : " en OrcaSlicer/Bambu Studio"}.`
              : `Agujeros holgados (+${meanErr.toFixed(2)} mm). Reduce compensación XY${isMslaResin ? " o sube exposición +0.15 s" : ""}.`,
      });
    } finally {
      setCaliperBusy(false);
    }
  };

  return (
    <div className="mech-card">
      <div className="pareto-head">
        <strong>Mecatrónica &amp; Impresión 3D (v10.0)</strong>
        {mechatronics && (
          <span className="badge good">${mechatronics.total_bom_cost_usd.toFixed(0)} USD</span>
        )}
      </div>

      {/* 5 Sub-tabs */}
      <div className="mech-tabs" style={{ display: "grid", gridTemplateColumns: "repeat(5, 1fr)", gap: 4 }}>
        <button
          type="button"
          className={`mech-tab ${subTab === "motors" ? "active" : ""}`}
          onClick={() => setSubTab("motors")}
        >
          Motores
        </button>
        <button
          type="button"
          className={`mech-tab ${subTab === "circuit" ? "active" : ""}`}
          onClick={() => setSubTab("circuit")}
        >
          Circuito
        </button>
        <button
          type="button"
          className={`mech-tab ${subTab === "bom" ? "active" : ""}`}
          onClick={() => setSubTab("bom")}
        >
          BOM
        </button>
        <button
          type="button"
          className={`mech-tab ${subTab === "print" ? "active" : ""}`}
          onClick={() => setSubTab("print")}
        >
          🖨 3D
        </button>
        <button
          type="button"
          className={`mech-tab ${subTab === "hardware" ? "active" : ""}`}
          onClick={() => setSubTab("hardware")}
          title="Control USB Serial / WebSerial en tiempo real"
        >
          🔌 USB
        </button>
      </div>

      {/* TAB 1: MOTORES & TORQUE */}
      {subTab === "motors" && (
        <div className="mech-pane">
          <div className="two-col">
            <div className="field-row">
              <div className="field-head"><span>Bus Eléctrico</span></div>
              <select
                value={config.bus_architecture}
                onChange={(e) => {
                  const nextBus = e.target.value as BusArchitecture;
                  const nextVolt =
                    nextBus === "esp32_pca9685_5v"
                      ? 5
                      : nextBus === "arduino_pca9685_i2c"
                        ? 6.5
                        : nextBus === "uart_serial_bus"
                          ? 7.4
                          : config.system_voltage_v < 12
                            ? 24
                            : config.system_voltage_v;
                  onChangeConfig({
                    ...config,
                    bus_architecture: nextBus,
                    system_voltage_v: nextVolt,
                  });
                }}
              >
                <option value="can_fd_bus">Bus CAN-FD (4 Hilos)</option>
                <option value="stm32_step_dir">Estrella STM32 Step/Dir</option>
                <option value="arduino_pca9685_i2c">Arduino UNO + PCA9685 I2C · 6V/10A</option>
                <option value="esp32_pca9685_5v">ESP32 Wi-Fi + PCA9685 · 5V/5A (Mars-Mini)</option>
                <option value="uart_serial_bus">Bus Serie UART (STS3215 / LeRobot)</option>
                <option value="teensy_dm542">Teensy 4.1 + DM542T (Industrial)</option>
              </select>
            </div>
            <div className="field-row">
              <div className="field-head"><span>Voltaje Sistema</span></div>
              <select
                value={config.system_voltage_v}
                onChange={(e) =>
                  onChangeConfig({ ...config, system_voltage_v: Number(e.target.value) })
                }
              >
                <option value={5}>5 V DC (ESP32/Servos 5A)</option>
                <option value={6}>6 V DC (Servos)</option>
                <option value={6.5}>6.5 V DC (Servos 10A)</option>
                <option value={7.4}>7.4 V DC (STS3215)</option>
                <option value={24}>24 V DC</option>
                <option value={48}>48 V DC</option>
              </select>
            </div>
          </div>

          {/* Per-joint actuator, reducer & 1x/2x parallel toggle */}
          <div className="joint-selector-list">
            {(["J1", "J2", "J3"] as const).map((jid) => {
              const spec = config.joint_actuators[jid] || {
                motor_id: jid === "J1" ? "nema23_mks_servo57d" : "nema17_mks_servo42d",
                reducer_id: jid === "J1" ? "planetary_20" : "planetary_10",
                target_rpm: 30,
                parallel_count: 1,
              };
              const pCount = spec.parallel_count ?? 1;
              return (
                <div key={jid} className="joint-selector-row has-parallel">
                  <span className="joint-pill">{jid}</span>
                  <select
                    value={spec.motor_id}
                    onChange={(e) => updateJointSpec(jid, { motor_id: e.target.value })}
                    title={`Motor para ${jid}`}
                  >
                    {actuators.map((a) => (
                      <option key={a.id} value={a.id}>
                        {a.name}
                      </option>
                    ))}
                  </select>
                  <select
                    value={spec.reducer_id}
                    onChange={(e) => updateJointSpec(jid, { reducer_id: e.target.value })}
                    title={`Reductor para ${jid}`}
                  >
                    {reducers.map((r) => (
                      <option key={r.id} value={r.id}>
                        {r.name}
                      </option>
                    ))}
                  </select>
                  <button
                    type="button"
                    className={`btn-parallel-toggle ${pCount === 2 ? "active" : ""}`}
                    onClick={() => updateJointSpec(jid, { parallel_count: pCount === 2 ? 1 : 2 })}
                    title={
                      pCount === 2
                        ? "2× actuadores enfrentados en paralelo (doble torque)"
                        : "Actuador simple (clic para ×2 en paralelo)"
                    }
                  >
                    {pCount}×
                  </button>
                </div>
              );
            })}
          </div>

          {/* Interactive Torque-RPM curve */}
          {mechatronics?.joint_reports?.length ? (
            <>
              <div className="joint-chart-pills">
                {mechatronics.joint_reports.map((rep) => {
                  const pCount =
                    rep.parallel_count ?? config.joint_actuators[rep.joint_id]?.parallel_count ?? 1;
                  return (
                    <button
                      key={rep.joint_id}
                      type="button"
                      className={`joint-chart-pill ${
                        activeReport?.joint_id === rep.joint_id ? "active" : ""
                      }`}
                      onClick={() => setSelectedJointId(rep.joint_id)}
                    >
                      {rep.joint_id}
                      {pCount === 2 ? " ×2" : ""}
                    </button>
                  );
                })}
              </div>
              {activeReport && (
                <TorqueSpeedChart
                  report={activeReport}
                  parallelCount={config.joint_actuators[activeReport.joint_id]?.parallel_count ?? 1}
                />
              )}
            </>
          ) : (
            <div className="mech-empty">
              Pulsa <strong>«Ensamblar Brazo Real»</strong> para calcular las curvas Torque–RPM, margen térmico y presupuesto eléctrico.
            </div>
          )}
        </div>
      )}

      {/* TAB 2: CIRCUITO & POWER BUDGET */}
      {subTab === "circuit" && (
        <div className="mech-pane">
          {pb ? (
            <>
              <div className="metrics-grid">
                <div className="metric-box">
                  <span>Voltaje</span>
                  <strong>{formatVoltage(pb.system_voltage_v)}</strong>
                </div>
                <div className="metric-box">
                  <span>Corriente</span>
                  <strong>{pb.continuous_current_a.toFixed(1)} / {pb.peak_current_a.toFixed(1)} A</strong>
                </div>
                <div className="metric-box">
                  <span>Potencia</span>
                  <strong>{pb.continuous_power_w.toFixed(0)} / {pb.peak_power_w.toFixed(0)} W</strong>
                </div>
                <div className="metric-box">
                  <span>Calibre AWG</span>
                  <strong>{pb.power_wire_awg} / {pb.signal_wire_awg}</strong>
                </div>
                <div className="metric-box">
                  <span>Arnés Total</span>
                  <strong>{pb.total_harness_length_m.toFixed(2)} m</strong>
                </div>
                <div className="metric-box">
                  <span>Fusible</span>
                  <strong>{pb.fuse_rating_a.toFixed(0)} A</strong>
                </div>
              </div>

              <div className="psu-banner">
                <span>Fuente PSU Recomendada</span>
                <strong>{pb.recommended_psu}</strong>
              </div>

              {circuitSvgUrl && (
                <div className="circuit-preview-box">
                  <img
                    src={circuitSvgUrl}
                    alt="Esquemático eléctrico del brazo"
                    className="circuit-svg-img"
                    onClick={() => setShowSvgModal(true)}
                  />
                  <button
                    type="button"
                    className="btn-tool btn-expand-svg"
                    onClick={() => setShowSvgModal(true)}
                  >
                    Ampliar Esquemático
                  </button>
                </div>
              )}

              <div className="dl-row">
                {circuitSvgUrl && (
                  <a className="dl-link" href={circuitSvgUrl} download>
                    Esquemático .SVG
                  </a>
                )}
                {harnessYmlUrl && (
                  <a className="dl-link" href={harnessYmlUrl} download>
                    WireViz .YML
                  </a>
                )}
                {firmwareInoUrl && (
                  <a className="dl-link dl-zip" href={firmwareInoUrl} download>
                    Firmware .INO
                  </a>
                )}
                {firmwareEsp32InoUrl && (
                  <a
                    className="dl-link dl-zip"
                    href={firmwareEsp32InoUrl}
                    download
                    title="Sketch ESP32 Wi-Fi Web + PCA9685 5V (Mars-Mini)"
                  >
                    ⬇ ESP32 Wi-Fi .INO
                  </a>
                )}
                {cncDxfUrl && (
                  <a className="dl-link dl-zip" href={cncDxfUrl} download title="Placas laterales CNC Aluminio 6061-T6 (.DXF R12)">
                    CNC .DXF
                  </a>
                )}
                {lerobotJsonUrl && (
                  <a className="dl-link dl-zip" href={lerobotJsonUrl} download title="Configuración calibrada HuggingFace LeRobot (.JSON)">
                    LeRobot .JSON
                  </a>
                )}
              </div>
            </>
          ) : (
            <div className="mech-empty">
              Ensambla el brazo modular para generar el <strong>Presupuesto Eléctrico</strong> y el <strong>Esquemático SVG / WireViz</strong>.
            </div>
          )}
        </div>
      )}

      {/* TAB 3: BOM UNITARIO MAESTRO */}
      {subTab === "bom" && (
        <div className="mech-pane">
          {mechatronics?.master_bom?.length ? (
            <>
              <div className="bom-total-banner">
                <span>Coste Total Estimado (BOM Unitario)</span>
                <strong>${mechatronics.total_bom_cost_usd.toFixed(2)} USD</strong>
              </div>

              <div className="bom-groups">
                {CATEGORY_ORDER.map(({ key, title }) => {
                  const items = groupedBom[key];
                  if (!items.length) return null;
                  const catSum = items.reduce((acc, it) => acc + it.total_cost_usd, 0);
                  return (
                    <div key={key} className="bom-group-block">
                      <div className="bom-group-head">
                        <span>{title}</span>
                        <strong>${catSum.toFixed(2)}</strong>
                      </div>
                      <table className="modular-parts-table">
                        <thead>
                          <tr>
                            <th>Componente</th>
                            <th>Cant.</th>
                            <th>Subtotal</th>
                          </tr>
                        </thead>
                        <tbody>
                          {items.map((it, idx) => (
                            <tr key={`${it.part_code}-${idx}`}>
                              <td>
                                <strong>{it.part_name}</strong>
                                <span className="part-var-sub">
                                  {it.part_code} · {it.spec}
                                </span>
                              </td>
                              <td className="mono">{it.qty}×</td>
                              <td className="mono">${it.total_cost_usd.toFixed(2)}</td>
                            </tr>
                          ))}
                        </tbody>
                      </table>
                    </div>
                  );
                })}
              </div>

              {masterBomCsvUrl && (
                <div className="dl-row">
                  <a className="dl-link dl-zip" href={masterBomCsvUrl} download>
                    Descargar BOM Maestro .CSV
                  </a>
                </div>
              )}
            </>
          ) : (
            <div className="mech-empty">
              Ensambla el brazo modular para generar la <strong>Lista de Materiales Unitaria</strong> (tornillos ISO 4762, rodamientos, motores, electrónica e impresión 3D).
            </div>
          )}
        </div>
      )}

      {/* TAB 4: IMPRESIÓN 3D LISTA PARA SLICER */}
      {subTab === "print" && (
        <div className="mech-pane">
          {printReadiness ? (
            <>
              <div className="psu-banner">
                <span>
                  {printReadiness.printer_name} ({(printReadiness.bed_mm || []).map((v) => Math.round(v)).join("×")} mm)
                </span>
                <span className={`badge ${printReadiness.fits_bed ? "good" : "warn"}`}>
                  {printReadiness.fits_bed ? "✓ Cabe en Cama" : "⚠ Requiere Dividir"}
                </span>
              </div>

              {(printReadiness.split_parts_count ?? 0) > 0 && (
                <div style={{ marginTop: 6 }}>
                  <span className="badge good">
                    ✂ {printReadiness.split_parts_count} piezas divididas (Cola de Milano)
                  </span>
                </div>
              )}

              <div className="metrics-grid">
                <div className="metric-box">
                  <span>Platos</span>
                  <strong>{printReadiness.total_plates}</strong>
                </div>
                <div className="metric-box">
                  <span>{isMslaResin ? "Resina Est." : "Filamento"}</span>
                  <strong>{printReadiness.total_filament_g.toFixed(0)} g</strong>
                </div>
                <div className="metric-box">
                  <span>Tiempo Est.</span>
                  <strong>{printReadiness.estimated_print_hours.toFixed(1)} h</strong>
                </div>
                {isMslaResin ? (
                  <>
                    <div className="metric-box">
                      <span>Capa / Inclinación</span>
                      <strong>
                        {printReadiness.layer_height_mm.toFixed(2)}mm · {printReadiness.recommended_tilt_deg ?? 35}°
                      </strong>
                    </div>
                    <div className="metric-box">
                      <span>Exposición</span>
                      <strong>
                        {(printReadiness.exposure_s ?? 2.3).toFixed(1)}s · Base {Math.round(printReadiness.bottom_exposure_s ?? 28)}s
                      </strong>
                    </div>
                    <div className="metric-box">
                      <span>Proceso</span>
                      <strong>MSLA 405nm</strong>
                    </div>
                  </>
                ) : (
                  <>
                    <div className="metric-box">
                      <span>Capa / Paredes</span>
                      <strong>{printReadiness.layer_height_mm.toFixed(2)}mm · {printReadiness.wall_loops}p</strong>
                    </div>
                    <div className="metric-box">
                      <span>Relleno</span>
                      <strong>{printReadiness.infill_pct}% {printReadiness.infill_pattern}</strong>
                    </div>
                    <div className="metric-box">
                      <span>Temp. Boq/Cama</span>
                      <strong>{printReadiness.nozzle_temp_c}° / {printReadiness.bed_temp_c}°C</strong>
                    </div>
                  </>
                )}
              </div>

              {isMslaResin ? (
                <div className="slicer-recipe-box">
                  <strong>Receta Elegoo SatelLite / Chitubox (Resina MSLA):</strong>{" "}
                  Capa {printReadiness.layer_height_mm.toFixed(2)} mm · Exposición {(printReadiness.exposure_s ?? 2.3).toFixed(1)} s ·{" "}
                  Base {Math.round(printReadiness.bottom_exposure_s ?? 28)} s · Ángulo {printReadiness.recommended_tilt_deg ?? 35}° ·{" "}
                  Lavado IPA + Curado UV 3–5 min.
                </div>
              ) : (
                <div className="slicer-recipe-box">
                  <strong>Receta Bambu Studio / OrcaSlicer / PrusaSlicer:</strong>{" "}
                  Capa {printReadiness.layer_height_mm.toFixed(2)} mm · {printReadiness.wall_loops} paredes ·{" "}
                  {printReadiness.infill_pct}% {printReadiness.infill_pattern} · Boquilla {printReadiness.nozzle_temp_c}°C / Cama{" "}
                  {printReadiness.bed_temp_c}°C.
                </div>
              )}

              {printReadiness.warnings?.length > 0 && (
                <div className="modular-issues-list">
                  {printReadiness.warnings.map((w, idx) => (
                    <div key={idx} className="alert-item warning">
                      {w}
                    </div>
                  ))}
                </div>
              )}

              {/* Build plates list */}
              <div className="print-plates-list">
                {(printReadiness.plates || []).map((plate, idx) => {
                  const pNum = plate.plate_index ?? plate.plate_number ?? idx + 1;
                  const plateUrl =
                    plate.url || files?.[`build_plate_${pNum}_3mf`] || (pNum === 1 ? files?.build_plate_1_3mf : undefined);
                  return (
                    <div key={pNum} className="print-plate-row">
                      <div className="print-plate-info">
                        <strong>
                          Plato {pNum} ({plate.part_count ?? plate.parts.length} piezas)
                        </strong>
                        <small>{plate.parts.join(", ")}</small>
                      </div>
                      {plateUrl && (
                        <a className="dl-link dl-plate-btn" href={plateUrl} download>
                          Plato {pNum} (.3MF)
                        </a>
                      )}
                    </div>
                  );
                })}
              </div>

              <div className="dl-row modular-dl-row">
                {files?.stl_parts_zip && (
                  <a
                    className="dl-link dl-zip"
                    href={files.stl_parts_zip}
                    download
                    title="ZIP organizado en carpetas: 01_CALIBRACION_15MIN, 02_PIEZAS_STL_SEPARADAS y 03_PLATOS_3MF_SLICER"
                  >
                    ★ Paquete Piezas 3D (.STL + .3MF)
                  </a>
                )}
                {calibrationStlUrl && (
                  <a
                    className="dl-link dl-zip"
                    href={calibrationStlUrl}
                    download
                    title="Cupón de calibración de tolerancias M3/M4/608/eje imprimible en 15 minutos"
                  >
                    Test Tolerancias 15min (.STL)
                  </a>
                )}
                {printGuideMdUrl && (
                  <a className="dl-link" href={printGuideMdUrl} download>
                    Guía Slicer (.MD)
                  </a>
                )}
              </div>

              {/* Individual flat-oriented .STL buttons per printable part with clean Spanish labels */}
              {files && Object.keys(files).some((k) => k.startsWith("part_stl_")) && (
                <div
                  style={{
                    marginTop: 8,
                    padding: "8px 9px",
                    borderRadius: 8,
                    background: "rgba(15, 23, 42, 0.55)",
                    border: "1px solid rgba(148, 163, 184, 0.18)",
                  }}
                >
                  <div style={{ display: "flex", justifyContent: "space-between", alignItems: "center", marginBottom: 6 }}>
                    <strong style={{ fontSize: 11.5, color: "#e2e8f0" }}>
                      Piezas Individuales (.STL · Planas Z=0)
                    </strong>
                    <span style={{ fontSize: 10.5, color: "#94a3b8" }}>
                      {Object.keys(files).filter((k) => k.startsWith("part_stl_")).length} archivos
                    </span>
                  </div>
                  <div
                    style={{
                      display: "grid",
                      gridTemplateColumns: "1fr 1fr",
                      gap: 5,
                    }}
                  >
                    {Object.entries(files)
                      .filter(([k]) => k.startsWith("part_stl_"))
                      .map(([k, url]) => {
                        const raw = k.replace(/^part_stl_/, "");
                        const niceLabels: Record<string, string> = {
                          base_pedestal: "Pedestal Base",
                          shoulder_mount: "Montura Hombro",
                          shoulder_gear: "Engranaje Hombro",
                          elbow_gear: "Engranaje Codo",
                          clevis_shoulder: "Horquilla Hombro",
                          clevis_elbow: "Horquilla Codo",
                          upper_arm_link: "Brazo Superior",
                          forearm_link: "Antebrazo",
                          wrist_tube: "Tubo Muñeca",
                          wrist_j4_roll: "Muñeca J4 Roll",
                          wrist_j5_pitch: "Muñeca J5 Pitch",
                          wrist_j6_flange: "Brida J6 TCP",
                          tcp_flange: "Brida TCP",
                          gripper_base: "Pinza Base",
                          gripper_finger_left: "Dedo Izquierdo",
                          gripper_finger_right: "Dedo Derecho",
                        };
                        const label = niceLabels[raw] || raw.replace(/_/g, " ");
                        return (
                          <a key={k} className="dl-link" href={url} download title={`Descargar part_${raw}.stl`}>
                            {label} (.STL)
                          </a>
                        );
                      })}
                  </div>
                </div>
              )}
              {/* v8.0 Asistente de Calibración Pie de Rey (Cupón 15 min) */}
              <div
                style={{
                  marginTop: 8,
                  padding: "8px 9px",
                  borderRadius: 8,
                  background: "rgba(56, 189, 248, 0.08)",
                  border: "1px solid rgba(56, 189, 248, 0.28)",
                }}
              >
                <div style={{ display: "flex", justifyContent: "space-between", alignItems: "center", marginBottom: 6 }}>
                  <strong style={{ fontSize: 11.5, color: "#7dd3fc" }}>
                    Calibrar con Pie de Rey (Cupón 15 min)
                  </strong>
                  {caliperResult && (
                    <span
                      className={`badge ${
                        caliperResult.status === "perfect"
                          ? "good"
                          : caliperResult.status === "tight"
                            ? "warn"
                            : "neutral"
                      }`}
                    >
                      {caliperResult.status === "perfect"
                        ? "✓ Ideal"
                        : caliperResult.status === "tight"
                          ? "⚠ Apretado"
                          : "⚠ Holgado"}
                    </span>
                  )}
                </div>
                <div className="two-col">
                  <div className="field-row">
                    <div className="field-head">
                      <span>M3 medido (nom. 3.20)</span>
                    </div>
                    <input
                      type="number"
                      step={0.05}
                      min={2.4}
                      max={4.0}
                      value={measuredM3}
                      onChange={(e) => setMeasuredM3(Number(e.target.value))}
                    />
                  </div>
                  <div className="field-row">
                    <div className="field-head">
                      <span>Aloj. 608 (nom. 22.20)</span>
                    </div>
                    <input
                      type="number"
                      step={0.05}
                      min={21.0}
                      max={23.5}
                      value={measuredBore}
                      onChange={(e) => setMeasuredBore(Number(e.target.value))}
                    />
                  </div>
                </div>
                <button
                  type="button"
                  className="btn-tool"
                  style={{ width: "100%", marginTop: 6 }}
                  disabled={caliperBusy}
                  onClick={handleCalibrateCaliper}
                >
                  {caliperBusy ? "Calculando..." : "Calcular Compensación"}
                </button>
                {caliperResult && (
                  <div style={{ marginTop: 7 }}>
                    <div className="metrics-grid">
                      <div className="metric-box">
                        <span>Comp. XY</span>
                        <strong>
                          {caliperResult.xy_hole_compensation_mm >= 0 ? "+" : ""}
                          {caliperResult.xy_hole_compensation_mm.toFixed(2)} mm
                        </strong>
                      </div>
                      <div className="metric-box">
                        <span>Holgura Rec.</span>
                        <strong>{caliperResult.recommended_clearance_mm.toFixed(2)} mm</strong>
                      </div>
                      <div className="metric-box">
                        <span>{isMslaResin ? "Δ Expos." : "Slicer"}</span>
                        <strong>
                          {isMslaResin && caliperResult.exposure_delta_s != null
                            ? `${caliperResult.exposure_delta_s >= 0 ? "+" : ""}${caliperResult.exposure_delta_s.toFixed(2)} s`
                            : "Orca / Bambu"}
                        </strong>
                      </div>
                    </div>
                    {(caliperResult.diagnosis || caliperResult.message || caliperResult.slicer_tip) && (
                      <div className="alert-item info" style={{ marginTop: 6, fontSize: 11 }}>
                        {caliperResult.diagnosis || caliperResult.message || caliperResult.slicer_tip}
                      </div>
                    )}
                  </div>
                )}
              </div>
            </>
          ) : (
            <div className="mech-empty">
              Pulsa <strong>«Ensamblar Brazo Real»</strong> para generar automáticamente los <strong>Platos `.3MF`</strong>, el <strong>Test de Tolerancias 15min (`.STL`)</strong> y la <strong>Guía de Slicer (`.MD`)</strong>.
            </div>
          )}
        </div>
      )}

      {/* TAB 5: CONTROL USB EN VIVO (v8.0) */}
      {subTab === "hardware" && (
        <LiveHardwarePanel
          pose={
            livePose || {
              j0_base: 0,
              j1_shoulder: 35,
              j2_elbow: -55,
              j3_wrist: 20,
              gripper_pct: 50,
            }
          }
        />
      )}

      {/* Fullscreen Circuit Schematic Modal */}
      {showSvgModal && circuitSvgUrl && (
        <div className="svg-modal-backdrop" onClick={() => setShowSvgModal(false)}>
          <div className="svg-modal-card" onClick={(e) => e.stopPropagation()}>
            <div className="pareto-head">
              <strong>
                Esquemático Eléctrico &amp; Arnés (
                {config.bus_architecture === "esp32_pca9685_5v"
                  ? "ESP32 Wi-Fi + PCA9685 5V"
                  : config.bus_architecture === "arduino_pca9685_i2c"
                    ? "Arduino UNO + PCA9685 I2C"
                    : config.bus_architecture === "uart_serial_bus"
                      ? "Bus Serie UART (STS3215 / LeRobot)"
                      : config.bus_architecture === "teensy_dm542"
                        ? "Teensy 4.1 + DM542T"
                        : config.bus_architecture === "can_fd_bus"
                          ? "CAN-FD"
                          : "STM32 Step/Dir"}
                )
              </strong>
              <button type="button" className="btn-tool" onClick={() => setShowSvgModal(false)}>
                ✕ Cerrar
              </button>
            </div>
            <div className="svg-modal-body">
              <img src={circuitSvgUrl} alt="Esquemático eléctrico ampliado" />
            </div>
          </div>
        </div>
      )}
    </div>
  );
};
