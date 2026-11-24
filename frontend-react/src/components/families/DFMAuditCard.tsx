import React from "react";
import type { DFMAuditReport, DFMStatus } from "../../types";

export interface DFMAuditCardProps {
  audit: DFMAuditReport | null | undefined;
}

function statusMeta(status: DFMStatus): { cls: string; badge: string; icon: string } {
  if (status === "pass") return { cls: "good", badge: "✓ PASS", icon: "✓" };
  if (status === "warn") return { cls: "warn", badge: "⚠ WARN", icon: "⚠" };
  return { cls: "bad", badge: "⛔ FAIL", icon: "⛔" };
}

export const DFMAuditCard: React.FC<DFMAuditCardProps> = ({ audit }) => {
  if (!audit) return null;
  const overall = statusMeta(audit.status);

  return (
    <div
      className="fea-card"
      style={{
        borderColor:
          audit.status === "pass"
            ? "rgba(16, 185, 129, 0.42)"
            : audit.status === "warn"
              ? "rgba(245, 158, 11, 0.45)"
              : "rgba(239, 68, 68, 0.5)",
      }}
    >
      <div className="pareto-head">
        <strong>🛡 Auditoría Pre-Impresión (Goldfish)</strong>
        <span className={`badge ${overall.cls}`}>{overall.badge}</span>
      </div>

      {audit.summary && (
        <div style={{ fontSize: 11.5, color: "#cbd5e1", marginTop: 4, marginBottom: 8 }}>
          {audit.summary}
        </div>
      )}

      <div style={{ display: "flex", flexDirection: "column", gap: 5 }}>
        {(audit.checks || []).map((chk) => {
          const m = statusMeta(chk.status);
          return (
            <div
              key={chk.id}
              style={{
                display: "flex",
                alignItems: "flex-start",
                justifyContent: "space-between",
                gap: 8,
                padding: "6px 8px",
                borderRadius: 7,
                background: "rgba(15, 23, 42, 0.55)",
                border: "1px solid rgba(148, 163, 184, 0.16)",
              }}
            >
              <div style={{ minWidth: 0 }}>
                <div style={{ fontSize: 11.5, fontWeight: 600, color: "#e2e8f0" }}>
                  <span className={m.cls} style={{ marginRight: 5 }}>
                    {m.icon}
                  </span>
                  {chk.label}
                </div>
                <div style={{ fontSize: 10.5, color: "#94a3b8", marginTop: 2 }}>{chk.detail}</div>
              </div>
              <span className={`badge ${m.cls}`} style={{ flexShrink: 0 }}>
                {chk.status.toUpperCase()}
              </span>
            </div>
          );
        })}
      </div>
    </div>
  );
};
