import React, { useMemo, useState } from "react";
import type {
  JointActuatorSpec,
  JointMechatronicsReport,
  MasterBomCategory,
  MasterBomItem,
  MechatronicsCatalog,
  MechatronicsConfig,
  MechatronicsResponse,
} from "../types";

const FALLBACK_ACTUATORS = [
  { id: "nema17_48mm", name: "NEMA 17 48mm (Stepper)" },
  { id: "nema17_mks_servo42d", name: "NEMA 17 MKS SERVO42D (Closed-Loop)" },
  { id: "nema23_76mm", name: "NEMA 23 76mm (Stepper)" },
  { id: "nema23_mks_servo57d", name: "NEMA 23 MKS SERVO57D (Closed-Loop)" },
  { id: "bldc_qdd_5010", name: "BLDC QDD 5010 (Direct/QDD)" },
  { id: "bldc_qdd_8308", name: "BLDC QDD 8308 (High-Torque QDD)" },
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
  files: Record<string, string> | undefined;
  onChangeConfig: (next: MechatronicsConfig) => void;
}

function TorqueSpeedChart({ report }: { report: JointMechatronicsReport }) {
  const curve = report.torque_speed_curve || [];
  if (curve.length < 2) {
    return <div className="mech-empty">Sin puntos de curva Torque–RPM para {report.joint_id}.</div>;
  }

  const W = 310;
  const H = 152;
  const padL = 36;
  const padR = 12;
  const padT = 14;
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

  return (
    <div className="torque-chart-wrap">
      <div className="torque-chart-head">
        <span>
          <strong>{report.joint_name}</strong> · {report.motor_name}
        </span>
        <span className={`badge ${report.status === "ok" ? "good" : report.status === "warning" ? "warn" : "bad"}`}>
          {report.status === "ok" ? "✓ OK" : report.status === "warning" ? "⚠ Límite" : "✕ Saturado"}
        </span>
      </div>

      <svg viewBox={`0 0 ${W} ${H}`} className="torque-svg" role="img" aria-label={`Curva Torque RPM ${report.joint_id}`}>
        <defs>
          <linearGradient id={`contGrad-${report.joint_id}`} x1="0" y1="0" x2="0" y2="1">
            <stop offset="0%" stopColor="#38bdf8" stopOpacity="0.28" />
            <stop offset="100%" stopColor="#38bdf8" stopOpacity="0.02" />
          </linearGradient>
        </defs>

        {/* Grid lines */}
        {[0, 0.5, 1].map((frac) => {
          const y = padT + plotH * (1 - frac);
          return (
            <g key={frac}>
              <line x1={padL} y1={y} x2={W - padR} y2={y} stroke="rgba(148,163,184,0.14)" strokeDasharray="2 2" />
              <text x={padL - 4} y={y + 3} textAnchor="end" fontSize="8.5" fill="#64748b" fontFamily="monospace">
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

        {/* Axis title */}
        <text x={4} y={padT + 4} fontSize="8" fill="#94a3b8" fontFamily="monospace">
          N·m
        </text>
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
  files,
  onChangeConfig,
}) => {
  const [subTab, setSubTab] = useState<"motors" | "circuit" | "bom">("motors");
  const [selectedJointId, setSelectedJointId] = useState<string>("J1");
  const [showSvgModal, setShowSvgModal] = useState<boolean>(false);

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

  return (
    <div className="mech-card">
      <div className="pareto-head">
        <strong>Mecatrónica v4.0 (Motores · Circuito · BOM)</strong>
        {mechatronics && (
          <span className="badge good">${mechatronics.total_bom_cost_usd.toFixed(0)} USD</span>
        )}
      </div>

      {/* 3 Sub-tabs */}
      <div className="mech-tabs">
        <button
          type="button"
          className={`mech-tab ${subTab === "motors" ? "active" : ""}`}
          onClick={() => setSubTab("motors")}
        >
          Motores &amp; Torque
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
          BOM Unitario
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
                onChange={(e) =>
                  onChangeConfig({
                    ...config,
                    bus_architecture: e.target.value as "can_fd_bus" | "stm32_step_dir",
                  })
                }
              >
                <option value="can_fd_bus">Bus CAN-FD (4 Hilos)</option>
                <option value="stm32_step_dir">Estrella STM32 Step/Dir</option>
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
                <option value={24}>24 V DC</option>
                <option value={48}>48 V DC</option>
              </select>
            </div>
          </div>

          {/* Per-joint actuator & reducer selectors */}
          <div className="joint-selector-list">
            {(["J1", "J2", "J3"] as const).map((jid) => {
              const spec = config.joint_actuators[jid] || {
                motor_id: jid === "J1" ? "nema23_mks_servo57d" : "nema17_mks_servo42d",
                reducer_id: jid === "J1" ? "planetary_20" : "planetary_10",
                target_rpm: 30,
              };
              return (
                <div key={jid} className="joint-selector-row">
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
                </div>
              );
            })}
          </div>

          {/* Interactive Torque-RPM curve */}
          {mechatronics?.joint_reports?.length ? (
            <>
              <div className="joint-chart-pills">
                {mechatronics.joint_reports.map((rep) => (
                  <button
                    key={rep.joint_id}
                    type="button"
                    className={`joint-chart-pill ${
                      activeReport?.joint_id === rep.joint_id ? "active" : ""
                    }`}
                    onClick={() => setSelectedJointId(rep.joint_id)}
                  >
                    {rep.joint_id}
                  </button>
                ))}
              </div>
              {activeReport && <TorqueSpeedChart report={activeReport} />}
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
                  <strong>{pb.system_voltage_v.toFixed(0)} V DC</strong>
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

      {/* Fullscreen Circuit Schematic Modal */}
      {showSvgModal && circuitSvgUrl && (
        <div className="svg-modal-backdrop" onClick={() => setShowSvgModal(false)}>
          <div className="svg-modal-card" onClick={(e) => e.stopPropagation()}>
            <div className="pareto-head">
              <strong>Esquemático Eléctrico &amp; Arnés ({config.bus_architecture === "can_fd_bus" ? "CAN-FD" : "STM32 Step/Dir"})</strong>
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
