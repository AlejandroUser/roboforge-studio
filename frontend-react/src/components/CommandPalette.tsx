import React, { useEffect, useMemo, useRef, useState } from "react";

export interface CommandItem {
  id: string;
  label: string;
  category: string;
  keywords?: string;
  shortcut?: string;
  action: () => void;
}

export interface CommandPaletteProps {
  open: boolean;
  items: CommandItem[];
  onClose: () => void;
}

function fuzzyMatch(query: string, text: string): boolean {
  const q = query.trim().toLowerCase();
  if (!q) return true;
  const hay = text.toLowerCase();
  if (hay.includes(q)) return true;
  // Token match
  const tokens = q.split(/\s+/).filter(Boolean);
  if (tokens.every((t) => hay.includes(t))) return true;
  // Subsequence fuzzy match
  let qi = 0;
  for (let i = 0; i < hay.length && qi < q.length; i++) {
    if (hay[i] === q[qi]) qi++;
  }
  return qi === q.length;
}

export const CommandPalette: React.FC<CommandPaletteProps> = ({ open, items, onClose }) => {
  const [query, setQuery] = useState("");
  const [activeIdx, setActiveIdx] = useState(0);
  const inputRef = useRef<HTMLInputElement | null>(null);

  const filtered = useMemo(() => {
    return items.filter((item) =>
      fuzzyMatch(query, `${item.label} ${item.category} ${item.keywords || ""}`)
    );
  }, [items, query]);

  useEffect(() => {
    if (open) {
      setQuery("");
      setActiveIdx(0);
      setTimeout(() => inputRef.current?.focus(), 10);
    }
  }, [open]);

  useEffect(() => {
    setActiveIdx(0);
  }, [query]);

  if (!open) return null;

  const runItem = (item: CommandItem) => {
    onClose();
    item.action();
  };

  return (
    <div
      className="cmdk-backdrop"
      onClick={onClose}
      role="dialog"
      aria-modal="true"
      aria-label="Paleta de Comandos RoboForge"
    >
      <div className="cmdk-modal" onClick={(e) => e.stopPropagation()}>
        <div className="cmdk-search-row">
          <span className="cmdk-icon">⌘</span>
          <input
            ref={inputRef}
            type="text"
            className="cmdk-input"
            placeholder="Buscar brazos (Mars-Mini, Franka Panda...), familias (FPV, Rover, Hélice), vistas o exportar..."
            value={query}
            onChange={(e) => setQuery(e.target.value)}
            onKeyDown={(e) => {
              if (e.key === "Escape") {
                e.preventDefault();
                onClose();
              } else if (e.key === "ArrowDown") {
                e.preventDefault();
                setActiveIdx((i) => (filtered.length ? (i + 1) % filtered.length : 0));
              } else if (e.key === "ArrowUp") {
                e.preventDefault();
                setActiveIdx((i) => (filtered.length ? (i - 1 + filtered.length) % filtered.length : 0));
              } else if (e.key === "Enter" && filtered[activeIdx]) {
                e.preventDefault();
                runItem(filtered[activeIdx]);
              }
            }}
          />
          <kbd className="cmdk-kbd">ESC</kbd>
        </div>

        <div className="cmdk-list">
          {filtered.length === 0 ? (
            <div className="cmdk-empty">Sin coincidencias para «{query}».</div>
          ) : (
            filtered.slice(0, 18).map((item, idx) => (
              <button
                key={item.id}
                type="button"
                className={`cmdk-item ${idx === activeIdx ? "active" : ""}`}
                onMouseEnter={() => setActiveIdx(idx)}
                onClick={() => runItem(item)}
              >
                <div className="cmdk-item-main">
                  <span className="cmdk-cat">{item.category}</span>
                  <span className="cmdk-label">{item.label}</span>
                </div>
                {item.shortcut && <kbd className="cmdk-kbd">{item.shortcut}</kbd>}
              </button>
            ))
          )}
        </div>

        <div className="cmdk-foot">
          <span>↑↓ Navegar</span>
          <span>↵ Ejecutar</span>
          <span>[ / ] Paneles · Tab Zen · F Ajustar Cámara</span>
        </div>
      </div>
    </div>
  );
};
