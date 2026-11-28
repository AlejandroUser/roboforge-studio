import type {
  ArmAssemblyResponse,
  ArmDatabaseResponse,
  ArmIkRequest,
  ArmIkResponse,
  BlueprintAnalysisResponse,
  CaliperCalibrateRequest,
  CaliperCalibrateResponse,
  CopilotResponse,
  FEAResult,
  GenerateResponse,
  GyroidResult,
  HardwarePortInfo,
  HardwarePoseRequest,
  HardwarePoseResponse,
  MechatronicsCatalog,
  ModularArmRequest,
  ModularArmResponse,
  ProjectFamilyInfo,
  ProjectGenerateRequest,
  ProjectGenerateResponse,
  PropellerEvolveRequest,
  PropellerEvolveResponse,
  Simp3DLinkRequest,
  Simp3DLinkResponse,
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
  getMechatronicsCatalog: () =>
    reqJson<MechatronicsCatalog>("/api/mechatronics/catalog"),
  getArmDatabase: () =>
    reqJson<ArmDatabaseResponse>("/api/arm/database"),
  exportFirmwareIno: async (body: {
    arm_name: string;
    poses: Array<{
      j0_base: number;
      j1_shoulder: number;
      j2_elbow: number;
      j3_wrist: number;
      gripper_pct: number;
    }>;
    speed_ms?: number;
  }): Promise<string> => {
    const res = await fetch("/api/arm/firmware-ino", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body),
    });
    if (!res.ok) throw new Error(`HTTP ${res.status}`);
    return res.text();
  },

  downloadEsp32FirmwareIno: async (body: {
    arm_name: string;
    poses: Array<{
      j0_base: number;
      j1_shoulder: number;
      j2_elbow: number;
      j3_wrist: number;
      gripper_pct: number;
    }>;
    speed_ms?: number;
  }): Promise<string> => {
    const res = await fetch("/api/arm/firmware-esp32-ino", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body),
    });
    if (!res.ok) throw new Error(`HTTP ${res.status}`);
    return res.text();
  },

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

  // v8.0: IK 3D, Calibración Pie de Rey y Puente USB Serial
  solveArmIk: (body: ArmIkRequest) =>
    reqJson<ArmIkResponse>("/api/arm/ik", { method: "POST", body: JSON.stringify(body) }),

  calibrateCaliper: (body: CaliperCalibrateRequest) =>
    reqJson<CaliperCalibrateResponse>("/api/print/calibrate", {
      method: "POST",
      body: JSON.stringify(body),
    }),

  getHardwarePorts: () =>
    reqJson<{ ports: HardwarePortInfo[] } | HardwarePortInfo[]>("/api/hardware/ports"),

  sendHardwarePose: (body: HardwarePoseRequest) =>
    reqJson<HardwarePoseResponse>("/api/hardware/send-pose", {
      method: "POST",
      body: JSON.stringify(body),
    }),

  // v9.0: Multi-Proyecto (Drones FPV, Rovers 4WD, Gimbals 2-3 Ejes)
  getProjectFamilies: () =>
    reqJson<{ families: ProjectFamilyInfo[] }>("/api/projects/families"),

  generateProject: (familyId: string, body: ProjectGenerateRequest) =>
    reqJson<ProjectGenerateResponse>(`/api/projects/${familyId}/generate`, {
      method: "POST",
      body: JSON.stringify(body),
    }),

  // v13.0: Diseño Generativo por Física (NSGA-II Hélices + SIMP 3D Orgánico)
  evolvePropeller: (body: PropellerEvolveRequest) =>
    reqJson<PropellerEvolveResponse>("/api/projects/propeller/evolve", {
      method: "POST",
      body: JSON.stringify(body),
    }),

  generateSimp3DLink: (body: Simp3DLinkRequest) =>
    reqJson<Simp3DLinkResponse>("/api/projects/generative/simp3d-link", {
      method: "POST",
      body: JSON.stringify(body),
    }),

  importAirfoilDat: (airfoil_id: string, dat_text: string) =>
    reqJson<{
      id: string;
      name: string;
      thickness_pct: number;
      camber_pct: number;
      cl0: number;
      points: number;
    }>("/api/projects/propeller/import-dat", {
      method: "POST",
      body: JSON.stringify({ airfoil_id, dat_text }),
    }),

  generateOrganicStudio: (body: import("./types").OrganicStudioRequest) =>
    reqJson<import("./types").OrganicStudioResponse>("/api/projects/organic-studio/generate", {
      method: "POST",
      body: JSON.stringify(body),
    }),
};
