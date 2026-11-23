import React from "react";
import type { ArmSlotKey, ModularArmResponse, ModularArmSlots } from "../types";

const SLOT_DEFINITIONS: Array<{
  key: ArmSlotKey;
  variantKey: keyof ModularArmSlots;
  label: string;
  familyHint: string;
}> = [
  { key: "shoulder_mount", variantKey: "shoulder_mount_variant", label: "Montura Hombro", familyHint: "Monturas" },
  { key: "shoulder_gear", variantKey: "shoulder_gear_variant", label: "Transmisión Hombro", familyHint: "Transmisión" },
  { key: "upper_arm", variantKey: "upper_arm_variant", label: "Brazo Superior", familyHint: "Eslabones" },
  { key: "elbow_gear", variantKey: "elbow_gear_variant", label: "Transmisión Codo", familyHint: "Transmisión" },
  { key: "forearm", variantKey: "forearm_variant", label: "Antebrazo", familyHint: "Eslabones" },
];

export interface ModularArmPanelProps {
  slots: ModularArmSlots;
  slotSummaries: Partial<Record<ArmSlotKey, string>>;
  result: ModularArmResponse | null;
  isBusy: boolean;
  onClearSlot: (slot: ArmSlotKey) => void;
  onAutoMatch: () => void;
  onAssemble: () => void;
}

export const ModularArmPanel: React.FC<ModularArmPanelProps> = ({
  slots,
  slotSummaries,
  result,
  isBusy,
  onClearSlot,
  onAutoMatch,
  onAssemble,
}) => {
  const assignedCount = SLOT_DEFINITIONS.filter((s) => Boolean(slots[s.key])).length;

  const hasCritical = result?.compatibility_issues?.some((i) => i.severity === "critical") ?? false;
  const hasWarning = result?.compatibility_issues?.some((i) => i.severity === "warning") ?? false;

  const statusBadge = result
    ? hasCritical
      ? { cls: "bad", text: "⛔ Crítico" }
      : hasWarning
        ? { cls: "warn", text: "⚠ Advertencia" }
        : { cls: "good", text: "✓ Compatible" }
    : { cls: "neutral", text: `${assignedCount}/5 asignadas` };

  const zipUrl = result?.files?.zip || result?.files?.manufacturing_zip || null;

  return (
    <div className="modular-card">
      <div className="pareto-head">
        <strong>Ensamblador Modular (Mix &amp; Match)</strong>
        <span className={`badge ${statusBadge.cls}`}>{statusBadge.text}</span>
      </div>

      {/* 5 Slots List */}
      <div className="modular-slots-list">
        {SLOT_DEFINITIONS.map((def) => {
          const isAssigned = Boolean(slots[def.key]);
          const variantName = String(slots[def.variantKey] || "");
          const summary = slotSummaries[def.key];

          return (
            <div key={def.key} className={`modular-slot-row ${isAssigned ? "assigned" : ""}`}>
              <div className="modular-slot-info">
                <span className="modular-slot-label">{def.label}</span>
                {isAssigned ? (
                  <small className="modular-slot-val">
                    {summary || variantName}
                  </small>
                ) : (
                  <small className="modular-slot-default">
                    Por defecto ({variantName})
                  </small>
                )}
              </div>
              {isAssigned ? (
                <button
                  type="button"
                  className="btn-slot-clear"
                  onClick={() => onClearSlot(def.key)}
                  title={`Quitar pieza personalizada de ${def.label}`}
                >
                  ✕
                </button>
              ) : (
                <span className="slot-hint-tag">{def.familyHint}</span>
              )}
            </div>
          );
        })}
      </div>

      {/* Primary Actions */}
      <div className="two-col">
        <button type="button" className="btn-tool" disabled={isBusy} onClick={onAutoMatch}>
          Auto-Ajustar Interfaces
        </button>
        <button type="button" className="btn-primary" disabled={isBusy} onClick={onAssemble}>
          {isBusy ? "Ensamblando..." : "Ensamblar Brazo Real"}
        </button>
      </div>

      {/* Compatibility Issues & Auto-Fix Notes */}
      {result && result.auto_fixed && Array.isArray(result.auto_fix_notes) && result.auto_fix_notes.length > 0 && (
        <div className="alert-item info">
          <strong>Auto-Ajuste aplicado:</strong> {result.auto_fix_notes.join(" · ")}
        </div>
      )}

      {result && Array.isArray(result.compatibility_issues) && result.compatibility_issues.length > 0 && (
        <div className="modular-issues-list">
          {result.compatibility_issues.map((issue, idx) => (
            <div
              key={`${issue.slot}-${issue.parameter}-${idx}`}
              className={`alert-item ${
                issue.severity === "critical" ? "critical" : issue.severity === "warning" ? "warning" : "info"
              }`}
            >
              <div>
                <strong>
                  [{issue.slot}] {issue.parameter}:
                </strong>{" "}
                {issue.message}
              </div>
              {issue.current_value !== issue.suggested_value && (
                <div className="issue-diff">
                  Actual: <code>{issue.current_value}</code> → Sugerido: <code>{issue.suggested_value}</code>
                </div>
              )}
            </div>
          ))}
        </div>
      )}

      {/* Real Cascaded Metrics & Per-Part Safety Factors */}
      {result && typeof result.total_mass_g === "number" && (
        <>
          <div className="metrics-grid">
            <div className="metric-box">
              <span>Masa Real</span>
              <strong>{(result.total_mass_g / 1000).toFixed(2)} kg</strong>
            </div>
            <div className="metric-box">
              <span>Torque J1</span>
              <strong>{(result.shoulder_torque_nm ?? 0).toFixed(2)} N·m</strong>
            </div>
            <div className="metric-box">
              <span>FS Mínimo</span>
              <strong
                className={
                  (result.min_safety_factor ?? 0) >= 2.0 ? "good" : (result.min_safety_factor ?? 0) >= 1.25 ? "warn" : "bad"
                }
              >
                {(result.min_safety_factor ?? 0).toFixed(2)}×
              </strong>
            </div>
          </div>

          {Array.isArray(result.per_part_metrics) && result.per_part_metrics.length > 0 && (
            <div className="modular-parts-table-wrap">
              <table className="modular-parts-table">
                <thead>
                  <tr>
                    <th>Pieza</th>
                    <th>Masa</th>
                    <th>Torque</th>
                    <th>FS</th>
                  </tr>
                </thead>
                <tbody>
                  {result.per_part_metrics.map((p) => (
                    <tr key={`${p.slot}-${p.part_name}`}>
                      <td>
                        <strong>{p.part_name}</strong>
                        <span className="part-var-sub">{p.variant}</span>
                      </td>
                      <td className="mono">{p.mass_g.toFixed(0)} g</td>
                      <td className="mono">{p.torque_nm.toFixed(2)}</td>
                      <td className="mono">
                        <span className={p.status === "ok" ? "good" : p.status === "warning" ? "warn" : "bad"}>
                          {p.safety_factor.toFixed(2)}×
                        </span>
                      </td>
                    </tr>
                  ))}
                </tbody>
              </table>
            </div>
          )}

          {/* Bloque 1: Kit de Manufactura (Ingeniería, CAD editable, Electrónica, BOM y Simulación) */}
          <div
            style={{
              marginTop: 8,
              padding: "8px 9px",
              borderRadius: 8,
              background: "rgba(15, 23, 42, 0.55)",
              border: "1px solid rgba(56, 189, 248, 0.22)",
            }}
          >
            <div style={{ display: "flex", justifyContent: "space-between", alignItems: "center", marginBottom: 6 }}>
              <strong style={{ fontSize: 11.5, color: "#e2e8f0" }}>Kit de Manufactura (CAD · BOM · Electrónica)</strong>
            </div>
            <div className="dl-row modular-dl-row">
              {zipUrl && (
                <a
                  className="dl-link dl-zip"
                  href={zipUrl}
                  download
                  title="Kit completo de ingeniería: .STEP editable, Esquemático .SVG, Firmware .INO/.JSON, BOM .CSV, URDF y MuJoCo"
                >
                  ★ Descargar Kit Manufactura (.ZIP)
                </a>
              )}
              {result.files?.step && (
                <a className="dl-link" href={result.files.step} download>
                  .STEP
                </a>
              )}
              {result.files?.urdf && (
                <a className="dl-link" href={result.files.urdf} download>
                  .URDF
                </a>
              )}
              {result.files?.mujoco && (
                <a className="dl-link" href={result.files.mujoco} download>
                  .XML
                </a>
              )}
              {result.files?.bom_csv && (
                <a className="dl-link" href={result.files.bom_csv} download>
                  BOM .CSV
                </a>
              )}
              {result.files?.cnc_dxf && (
                <a className="dl-link" href={result.files.cnc_dxf} download title="Placas 2.5D CNC Aluminio 6061 (.DXF)">
                  CNC .DXF
                </a>
              )}
              {result.files?.lerobot_json && (
                <a className="dl-link" href={result.files.lerobot_json} download title="Perfil calibrado HuggingFace LeRobot">
                  LeRobot .JSON
                </a>
              )}
              {result.files?.firmware_ino && (
                <a className="dl-link" href={result.files.firmware_ino} download title="Sketch Arduino UNO + PCA9685">
                  .INO
                </a>
              )}
            </div>
          </div>

          {/* Bloque 2: Archivos de Piezas Separados para Impresión 3D */}
          {(result.files?.stl_parts_zip || result.files?.calibration_stl) && (
            <div
              style={{
                marginTop: 7,
                padding: "8px 9px",
                borderRadius: 8,
                background: "rgba(16, 185, 129, 0.08)",
                border: "1px solid rgba(16, 185, 129, 0.30)",
              }}
            >
              <div style={{ display: "flex", justifyContent: "space-between", alignItems: "center", marginBottom: 6 }}>
                <strong style={{ fontSize: 11.5, color: "#6ee7b7" }}>Archivos de Piezas (Impresión 3D)</strong>
                <span style={{ fontSize: 10.5, color: "#94a3b8" }}>Separadas · Planas Z=0</span>
              </div>
              <div className="dl-row modular-dl-row">
                {result.files?.stl_parts_zip && (
                  <a
                    className="dl-link dl-zip"
                    href={result.files.stl_parts_zip}
                    download
                    title="ZIP organizado en carpetas: 01_CALIBRACION_15MIN, 02_PIEZAS_STL_SEPARADAS y 03_PLATOS_3MF_SLICER"
                  >
                    ★ Paquete Piezas 3D (.STL + .3MF)
                  </a>
                )}
                {result.files?.calibration_stl && (
                  <a
                    className="dl-link"
                    href={result.files.calibration_stl}
                    download
                    title="Cupón de calibración de tolerancias M3/M4/608 imprimible en 15 minutos"
                  >
                    Test Tolerancias (.STL)
                  </a>
                )}
              </div>
            </div>
          )}
        </>
      )}
    </div>
  );
};
