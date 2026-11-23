import type {
  ArmAssemblyResponse,
  BlueprintAnalysisResponse,
  CopilotResponse,
  FEAResult,
  GenerateResponse,
  GyroidResult,
  ModularArmRequest,
  ModularArmResponse,
} from "./types";

async function reqJson<T>(url: string, init?: RequestInit): Promise<T> {
  const res = await fetch(url, {
    headers: { "Content-Type": "application/json", ...(init?.headers || {}) },
    ...init,
  });
  if (!res.ok) {
    let msg = `HTTP ${res.status}`;
    try {
      const data = await res.json();
      msg = data.detail || msg;
    } catch {
      /* ignore */
    }
    throw new Error(msg);
  }
  return res.json();
}

export const api = {
  health: () => reqJson<{ status: string }>("/api/health"),
  materials: () => reqJson<Record<string, any>>("/api/materials"),
  printers: () => reqJson<Record<string, any>>("/api/printers"),
  linkPresets: () => reqJson<Record<string, any>>("/api/presets"),
  mountPresets: () => reqJson<Record<string, any>>("/api/mounts/presets"),
  gearPresets: () => reqJson<Record<string, any>>("/api/gears/presets"),
  armPresets: () => reqJson<Record<string, any>>("/api/arm/presets"),

  generateLinks: (body: any) =>
    reqJson<GenerateResponse>("/api/generate", { method: "POST", body: JSON.stringify(body) }),
  generateMounts: (body: any) =>
    reqJson<GenerateResponse>("/api/mounts/generate", { method: "POST", body: JSON.stringify(body) }),
  generateGears: (body: any) =>
    reqJson<GenerateResponse>("/api/gears/generate", { method: "POST", body: JSON.stringify(body) }),
  generateArm: (body: any) =>
    reqJson<ArmAssemblyResponse>("/api/arm/generate", { method: "POST", body: JSON.stringify(body) }),
  autoMatchArm: (body: ModularArmRequest) =>
    reqJson<ModularArmResponse>("/api/arm/auto-match", { method: "POST", body: JSON.stringify(body) }),
  assembleModularArm: (body: ModularArmRequest) =>
    reqJson<ModularArmResponse>("/api/arm/assemble", { method: "POST", body: JSON.stringify(body) }),

  runFea: (family: "links" | "mounts" | "gears", body: any) => {
    const endpoint =
      family === "mounts" ? "/api/mounts/fea" : family === "gears" ? "/api/gears/fea" : "/api/fea";
    return reqJson<FEAResult>(endpoint, { method: "POST", body: JSON.stringify(body) });
  },

  generateGyroid: (body: any) =>
    reqJson<GyroidResult>("/api/sdf/gyroid", { method: "POST", body: JSON.stringify(body) }),

  copilotOptimize: (body: any) =>
    reqJson<CopilotResponse>("/api/copilot/optimize", { method: "POST", body: JSON.stringify(body) }),

  analyzeBlueprint: async (file: File, prompt: string, verifyWithFea: boolean): Promise<BlueprintAnalysisResponse> => {
    const fd = new FormData();
    fd.append("file", file);
    fd.append("prompt", prompt);
    fd.append("verify_with_fea", String(verifyWithFea));
    const res = await fetch("/api/copilot/blueprint", { method: "POST", body: fd });
    if (!res.ok) throw new Error(`HTTP ${res.status}`);
    return res.json();
  },

  analyzeStep: async (file: File): Promise<any> => {
    const fd = new FormData();
    fd.append("file", file);
    const res = await fetch("/api/analyze-step", { method: "POST", body: fd });
    if (!res.ok) throw new Error(`HTTP ${res.status}`);
    return res.json();
  },
};
