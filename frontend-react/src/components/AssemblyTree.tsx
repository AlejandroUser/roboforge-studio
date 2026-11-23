import React from "react";
import type { ArmAssemblyResponse } from "../types";

interface AssemblyTreeProps {
  arm: ArmAssemblyResponse;
  selectedNode: string | null;
  onSelectNode: (node: string) => void;
  onInspectSegmentInLinks: (lengthMm: number, loadN: number, label: string) => void;
}

export const AssemblyTree: React.FC<AssemblyTreeProps> = ({
  arm,
  selectedNode,
  onSelectNode,
  onInspectSegmentInLinks,
}) => {
  const nodes = [
    { id: "base_pedestal", label: "01 · Pedestal Base (Anclaje)", meta: "Fijo al bastidor", editable: false, len: 0, load: 0 },
    ...arm.joints.map((j, idx) => ({
      id: idx === 0 ? "upper_arm" : idx === 1 ? "forearm" : "wrist",
      label: `0${idx + 2} · ${j.joint_name}`,
      meta: `${j.segment_length_mm} mm · ${j.segment_mass_g.toFixed(0)} g · ${j.dynamic_torque_nm.toFixed(1)} N·m · ${j.recommended_actuator}`,
      editable: true,
      len: j.segment_length_mm,
      load: Math.max(10, Math.round((j.dynamic_torque_nm * 1000) / Math.max(j.segment_length_mm, 50))),
    })),
    { id: "tcp_flange", label: "05 · Brida Efector Final (TCP)", meta: "Patrón ISO 4×M4", editable: false, len: 0, load: 0 },
  ];

  return (
    <div className="tree-card">
      <div className="pareto-head">
        <strong>Árbol de Ensamblaje Interactivo</strong>
        <span>Clic para resaltar</span>
      </div>
      <div className="tree-list">
        {nodes.map((n) => (
          <div
            key={n.id}
            className={`tree-row ${selectedNode === n.id ? "active" : ""}`}
            onClick={() => onSelectNode(n.id)}
          >
            <div className="tree-row-main">
              <strong>{n.label}</strong>
              <small>{n.meta}</small>
            </div>
            {n.editable && (
              <button
                type="button"
                className="btn-mini"
                title="Abrir este eslabón en Eslabones Robóticos para optimización SIMP y FEA 3D"
                onClick={(e) => {
                  e.stopPropagation();
                  onInspectSegmentInLinks(n.len, n.load, n.label);
                }}
              >
                Editar / FEA
              </button>
            )}
          </div>
        ))}
      </div>
    </div>
  );
};
