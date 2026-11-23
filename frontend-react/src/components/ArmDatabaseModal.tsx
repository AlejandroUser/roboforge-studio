import React, { useMemo, useState } from "react";
import type { ArmDatabaseEntry } from "../types";

export interface ArmDatabaseModalProps {
  open: boolean;
  arms: ArmDatabaseEntry[];
  averageReplicabilityPct?: number;
  onClose: () => void;
  onLoadArm: (arm: ArmDatabaseEntry) => void;
}

function getTopologyBadges(arm: ArmDatabaseEntry): string[] {
  const badges: string[] = [];
  const kt = (arm.kinematics_type || "").toLowerCase();
  const id = (arm.id || "").toLowerCase();
  const tx = (arm.transmission_summary || "").toLowerCase();
  const elec = (arm.electronics_summary || "").toLowerCase();
  const mats = (arm.primary_materials || []).join(" ").toLowerCase();

  if (kt.includes("scara")) badges.push("SCARA");
  if (kt.includes("cartesian") || id.includes("lumenpnp")) badges.push("Cartesiano");
  if (kt.includes("delta") || id.includes("delta")) badges.push("Delta");
  if (kt.includes("continuum") || id.includes("continuum")) badges.push("Continuo PCC");
  if (kt.includes("bimanual") || arm.dof >= 14) badges.push("Bimanual 14-DOF");
  if (kt.includes("stewart") || id.includes("stewart")) badges.push("Stewart 6-UPS");
  if (kt.includes("pneumatic") || id.includes("festo")) badges.push("Fuelle TPU");
  if (kt.includes("zero_g") || id.includes("canadarm")) badges.push("0-G Espacio");
  if (kt.includes("subsea") || id.includes("rov")) badges.push("Submarino ROV");
  if (arm.payload_kg >= 50 || id.includes("kuka")) badges.push("Heavy 210kg");
  if (kt.includes("parallelogram") || id.includes("dobot")) badges.push("4-Barras");
  if (kt.includes("cable") || tx.includes("capstan")) badges.push("Capstan");
  if (tx.includes("bevel") || id.includes("thor")) badges.push("Bevel");
  if (arm.dof === 7) badges.push("7-DOF");
  if (tx.includes("cicloidal")) badges.push("Cicloidal");
  if (id.includes("lerobot") || id.includes("so_arm") || elec.includes("sts3215")) badges.push("LeRobot");
  if (id.includes("mars_mini") || mats.includes("resin") || mats.includes("msla")) badges.push("MSLA Resina");
  if (elec.includes("esp32")) badges.push("ESP32 Wi-Fi");
  if (id.includes("fabri") || (elec.includes("pca9685") && !elec.includes("esp32"))) badges.push("Servo 5.0");
  if (mats.includes("6061") || mats.includes("cnc")) badges.push("CNC 6061");
  return badges;
}

export const ArmDatabaseModal: React.FC<ArmDatabaseModalProps> = ({
  open,
  arms,
  averageReplicabilityPct,
  onClose,
  onLoadArm,
}) => {
  const [selectedCategory, setSelectedCategory] = useState<string>("Todos");

  const categories = useMemo(() => {
    const uniq = Array.from(new Set(arms.map((a) => a.category).filter(Boolean)));
    return ["Todos", ...uniq];
  }, [arms]);

  const filteredArms = useMemo(() => {
    if (selectedCategory === "Todos") return arms;
    return arms.filter((a) => a.category === selectedCategory);
  }, [arms, selectedCategory]);

  if (!open) return null;

  return (
    <div className="svg-modal-backdrop" onClick={onClose} role="dialog" aria-modal="true" aria-label="Base de Datos de Brazos Robóticos">
      <div className="arm-db-modal-card" onClick={(e) => e.stopPropagation()}>
        <div className="arm-db-head">
          <div>
            <h3>📚 Base de Datos de Brazos Robóticos Reales ({arms.length})</h3>
            <small>
              Benchmark de arquitecturas reales · Replicabilidad media:{" "}
              <strong>{averageReplicabilityPct ? `${averageReplicabilityPct.toFixed(1)}%` : "v5.0"}</strong> · 1 clic para cargar y ensamblar
            </small>
          </div>
          <button type="button" className="btn-tool" onClick={onClose}>
            ✕ Cerrar
          </button>
        </div>

        {/* Category filter bar */}
        <div className="arm-db-filters">
          {categories.map((cat) => (
            <button
              key={cat}
              type="button"
              className={`arm-db-filter-pill ${selectedCategory === cat ? "active" : ""}`}
              onClick={() => setSelectedCategory(cat)}
            >
              {cat}
            </button>
          ))}
        </div>

        {/* Cards grid */}
        <div className="arm-db-grid">
          {filteredArms.map((arm) => {
            const badges = getTopologyBadges(arm);
            const rep = Math.min(Math.max(arm.roboforge_replicability_pct || 0, 0), 100);
            const barClass = rep >= 80 ? "good" : rep >= 60 ? "cyan" : "warn";

            return (
              <article key={arm.id} className="arm-db-card">
                <div className="arm-db-card-top">
                  <div>
                    <h4>{arm.name}</h4>
                    <span className="arm-db-maker">
                      {arm.manufacturer_or_project} · {arm.category}
                    </span>
                  </div>
                  <div className="badge-row">
                    {badges.map((b) => (
                      <span key={b} className="badge badge-db-tag">
                        {b}
                      </span>
                    ))}
                  </div>
                </div>

                {/* Replicability progress bar */}
                <div className="arm-db-rep-row">
                  <div className="arm-db-rep-head">
                    <span>Replicabilidad en RoboForge</span>
                    <strong>{rep}%</strong>
                  </div>
                  <div className="arm-db-rep-track">
                    <div className={`arm-db-rep-fill ${barClass}`} style={{ width: `${rep}%` }} />
                  </div>
                </div>

                {/* Specs strip */}
                <div className="arm-db-specs">
                  <div><small>DOF</small><strong>{arm.dof}</strong></div>
                  <div><small>Alcance</small><strong>{Math.round(arm.reach_mm)} mm</strong></div>
                  <div><small>Carga</small><strong>{arm.payload_kg} kg</strong></div>
                  <div><small>Repetib.</small><strong>±{arm.repeatability_mm} mm</strong></div>
                  <div><small>Costo Est.</small><strong>${arm.estimated_cost_usd}</strong></div>
                </div>

                <p className="arm-db-desc">
                  <strong>Actuadores:</strong> {arm.actuators_summary}
                </p>

                {arm.missing_capabilities?.length > 0 && (
                  <div className="arm-db-limits">
                    <strong>Brechas / Notas:</strong> {arm.missing_capabilities[0]}
                  </div>
                )}

                <div className="arm-db-card-foot">
                  <button
                    type="button"
                    className="btn-primary arm-db-load-btn"
                    onClick={() => {
                      onLoadArm(arm);
                      onClose();
                    }}
                  >
                    Cargar en RoboForge
                  </button>
                </div>
              </article>
            );
          })}

          {filteredArms.length === 0 && (
            <div className="mech-empty">No hay brazos registrados en esta categoría.</div>
          )}
        </div>
      </div>
    </div>
  );
};
