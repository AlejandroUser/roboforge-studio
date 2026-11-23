import React from "react";
import type { VariantResult } from "../types";

const COLORS: Record<string, string> = {
  tapered_solid: "#38bdf8",
  organic_ibeam: "#22d3ee",
  rounded_truss: "#10b981",
  hollow_cobot: "#a78bfa",
  organic_topopt: "#f59e0b",
  mount_standard: "#38bdf8",
  mount_gusseted: "#10b981",
  mount_lightweight: "#f59e0b",
  gear_solid: "#38bdf8",
  gear_webbed: "#22d3ee",
  gear_spoked: "#10b981",
};

interface ParetoChartProps {
  variants: VariantResult[];
  activeVariantId: string | null;
  onSelect: (v: VariantResult) => void;
}

export const ParetoChart: React.FC<ParetoChartProps> = ({ variants, activeVariantId, onSelect }) => {
  if (!variants.length) return null;

  const W = 310;
  const H = 152;
  const pad = { l: 38, r: 16, t: 14, b: 26 };
  const masses = variants.map((v) => v.metrics.mass_g);
  const scores = variants.map((v) => v.metrics.stiffness_to_weight_score);
  const minM = Math.min(...masses) * 0.88;
  const maxM = Math.max(...masses) * 1.08 + 0.1;
  const minS = Math.min(...scores) * 0.82;
  const maxS = Math.max(...scores) * 1.12 + 1;

  const xOf = (m: number) => pad.l + ((m - minM) / (maxM - minM)) * (W - pad.l - pad.r);
  const yOf = (s: number) => H - pad.b - ((s - minS) / (maxS - minS)) * (H - pad.t - pad.b);

  // Pareto frontier (minimize mass, maximize stiffness/weight score)
  const sorted = [...variants].sort((a, b) => a.metrics.mass_g - b.metrics.mass_g);
  const frontier: VariantResult[] = [];
  let bestScore = -Infinity;
  for (const v of sorted) {
    if (v.metrics.stiffness_to_weight_score >= bestScore) {
      frontier.push(v);
      bestScore = v.metrics.stiffness_to_weight_score;
    }
  }
  const linePts = frontier.map((v) => `${xOf(v.metrics.mass_g)},${yOf(v.metrics.stiffness_to_weight_score)}`).join(" ");

  return (
    <div className="pareto-card">
      <div className="pareto-head">
        <strong>Frontera de Pareto (Masa vs. Rigidez/Peso)</strong>
        <span>Clic en un nodo</span>
      </div>
      <svg viewBox={`0 0 ${W} ${H}`} className="pareto-svg" role="img" aria-label="Gráfico Frontera de Pareto">
        <line x1={pad.l} y1={H - pad.b} x2={W - pad.r} y2={H - pad.b} stroke="#334155" strokeWidth="1" />
        <line x1={pad.l} y1={pad.t} x2={pad.l} y2={H - pad.b} stroke="#334155" strokeWidth="1" />
        {frontier.length > 1 && (
          <polyline fill="none" stroke="#38bdf8" strokeWidth="1.5" strokeDasharray="4 3" points={linePts} opacity={0.7} />
        )}
        {variants.map((v) => {
          const cx = xOf(v.metrics.mass_g);
          const cy = yOf(v.metrics.stiffness_to_weight_score);
          const color = COLORS[v.variant_id] || "#38bdf8";
          const isActive = v.variant_id === activeVariantId;
          return (
            <g key={v.variant_id} className="pareto-node" onClick={() => onSelect(v)} style={{ cursor: "pointer" }}>
              <title>{`${v.name}: ${v.metrics.mass_g.toFixed(1)} g · Score ${Math.round(v.metrics.stiffness_to_weight_score)} · FS ${v.metrics.safety_factor.toFixed(1)}×`}</title>
              {isActive && <circle cx={cx} cy={cy} r={10} fill={color} opacity={0.22} />}
              <circle cx={cx} cy={cy} r={isActive ? 6 : 4.5} fill={color} stroke="#090d14" strokeWidth={1.5} />
            </g>
          );
        })}
        <text x={W / 2} y={H - 5} textAnchor="middle" fill="#94a3b8" fontSize="9.5">
          ← Menor Masa (g) · Óptimo arriba-izquierda
        </text>
        <text x={11} y={H / 2} textAnchor="middle" fill="#94a3b8" fontSize="9" transform={`rotate(-90 11 ${H / 2})`}>
          Rigidez/Peso
        </text>
      </svg>
    </div>
  );
};
