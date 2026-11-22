/**
 * api.js — Cliente HTTP para RoboForge Studio FastAPI Backend
 */

const API_BASE = "";

async function requestJson(url, options = {}) {
  const response = await fetch(`${API_BASE}${url}`, {
    headers: {
      "Content-Type": "application/json",
      ...(options.headers || {}),
    },
    ...options,
  });

  if (!response.ok) {
    let detail = `Error HTTP ${response.status}`;
    try {
      const errBody = await response.json();
      if (typeof errBody?.detail === "string") {
        detail = errBody.detail;
      } else if (Array.isArray(errBody?.detail)) {
        detail = errBody.detail
          .map((e) => `${(e.loc || []).slice(1).join(".")}: ${e.msg}`)
          .join(" | ");
      }
    } catch {
      // ignore json parse error
    }
    throw new Error(detail);
  }

  return response.json();
}

export async function checkHealth() {
  return requestJson("/api/health", { method: "GET" });
}

/**
 * Normaliza la respuesta de GET /api/materials a un diccionario { [id]: material }
 * y un arreglo ordenado list.
 */
export async function fetchMaterials() {
  const raw = await requestJson("/api/materials", { method: "GET" });
  const map = {};
  const list = [];

  const source = Array.isArray(raw)
    ? raw
    : Array.isArray(raw?.materials)
      ? raw.materials
      : raw?.materials && typeof raw.materials === "object"
        ? Object.entries(raw.materials).map(([id, m]) => ({ id, ...m }))
        : Object.entries(raw || {}).map(([id, m]) => ({ id, ...m }));

  for (const item of source) {
    if (!item || typeof item !== "object") continue;
    const id = item.id;
    if (!id) continue;
    map[id] = item;
    list.push(item);
  }

  return { map, list };
}

/**
 * Obtiene GET /api/presets y normaliza arquitecturas e interfaces de extremos.
 */
export async function fetchPresets() {
  const raw = await requestJson("/api/presets", { method: "GET" });

  const architectures =
    raw?.architectures ||
    raw?.arm_architectures ||
    raw?.arms ||
    {};

  const endInterfaces =
    raw?.end_interfaces ||
    raw?.end_presets ||
    raw?.ends ||
    {};

  return { architectures, endInterfaces, raw };
}

/**
 * Envía LinkDesignRequest a POST /api/generate y retorna GenerateResponse.
 */
export async function generateDesign(payload, signal) {
  return requestJson("/api/generate", {
    method: "POST",
    body: JSON.stringify(payload),
    signal,
  });
}

/**
 * Sube un archivo .STEP/.STP a POST /api/analyze-step y retorna StepReferenceAnalysis.
 */
export async function analyzeStepFile(file) {
  const formData = new FormData();
  formData.append("file", file, file.name);

  const response = await fetch(`${API_BASE}/api/analyze-step`, {
    method: "POST",
    body: formData,
  });

  if (!response.ok) {
    let detail = `Error HTTP ${response.status}`;
    try {
      const errBody = await response.json();
      if (typeof errBody?.detail === "string") {
        detail = errBody.detail;
      } else if (Array.isArray(errBody?.detail)) {
        detail = errBody.detail
          .map((e) => `${(e.loc || []).slice(1).join(".")}: ${e.msg}`)
          .join(" | ");
      }
    } catch {
      // ignore json parse error
    }
    throw new Error(detail);
  }

  return response.json();
}
