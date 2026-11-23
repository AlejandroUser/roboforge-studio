import React, { useCallback, useEffect, useMemo, useRef, useState } from "react";
import { api } from "./api";
import { ArmDatabaseModal } from "./components/ArmDatabaseModal";
import { AssemblyTree } from "./components/AssemblyTree";
import { MechatronicsPanel } from "./components/MechatronicsPanel";
import { ModularArmPanel } from "./components/ModularArmPanel";
import { ParetoChart } from "./components/ParetoChart";
import { ViewportR3F } from "./components/ViewportR3F";
import type {
  ArmAssemblyResponse,
  ArmDatabaseEntry,
  ArmDatabaseResponse,
  ArmSlotKey,
  FEAResult,
  FamilyId,
  GenerateResponse,
  GyroidResult,
  KinematicsType,
  MechatronicsCatalog,
  MechatronicsConfig,
  ModularArmRequest,
  ModularArmResponse,
  ModularArmSlots,
  VariantResult,
} from "./types";

const DEFAULT_LINK = {
  architecture: "articulated_6dof",
  length_mm: 180,
  root_width_mm: 38,
  tip_width_mm: 24,
  thickness_mm: 20,
  organic_taper: 0.75,
  cable_channel: true,
  root_end: {
    preset: "nema17",
    outer_diameter_mm: 46,
    bore_diameter_mm: 22,
    bolt_count: 4,
    bolt_pcd_mm: 31,
    bolt_size: "M3",
    use_heat_inserts: true,
  },
  tip_end: {
    preset: "bearing_608",
    outer_diameter_mm: 32,
    bore_diameter_mm: 22,
    bolt_count: 0,
    bolt_pcd_mm: 26,
    bolt_size: "M3",
    use_heat_inserts: false,
  },
  tip_load_n: 25,
  angular_accel_rad_s2: 15,
  target_safety_factor: 2.0,
  max_deflection_mm: 0.5,
  operating_temp_c: 45,
  material_id: "petg_cf",
  print_orientation: "xy_flat",
  nozzle_diameter_mm: 0.4,
  printer_clearance_mm: 0.15,
  topopt_volume_fraction: 0.52,
  printer_id: "bambu_x1c",
};

const DEFAULT_MOUNT = {
  actuator_type: "nema17",
  base_type: "profile_2020",
  wall_thickness_mm: 5,
  base_depth_mm: 42,
  shaft_height_mm: 32,
  gusset_thickness_mm: 4,
  use_heat_inserts: false,
  ventilation_slots: true,
  radial_load_n: 35,
  motor_torque_nm: 0.6,
  target_safety_factor: 2.0,
  max_deflection_mm: 0.3,
  motor_temp_c: 58,
  material_id: "petg_cf",
  print_orientation: "xy_flat",
  nozzle_diameter_mm: 0.4,
  printer_clearance_mm: 0.15,
  printer_id: "bambu_x1c",
};

const DEFAULT_GEAR = {
  preset: "spur_nema17",
  gear_kind: "spur_involute",
  teeth: 24,
  module_mm: 2.0,
  pressure_angle_deg: 20,
  face_width_mm: 10,
  bore_diameter_mm: 5,
  d_shaft_flat_mm: 0.5,
  hub_diameter_mm: 16,
  hub_height_mm: 5,
  backlash_mm: 0.15,
  torque_nm: 1.2,
  rpm: 120,
  target_safety_factor: 2.0,
  operating_temp_c: 45,
  material_id: "petg_cf",
  print_orientation: "xy_flat",
  nozzle_diameter_mm: 0.4,
  printer_clearance_mm: 0.15,
  printer_id: "bambu_x1c",
};

const DEFAULT_ARM = {
  preset: "parol6_desktop",
  dof: 6,
  payload_kg: 1.0,
  base_height_mm: 95,
  upper_arm_mm: 200,
  forearm_mm: 170,
  wrist_mm: 75,
  link_style: "hollow_cobot",
  kinematics_type: "articulated" as KinematicsType,
  z_stroke_mm: 120,
  z_axis_mm: 40,
  wrist_3dof: true,
  cnc_plate_mode: false,
  remote_base_motors: false,
  four_bar_parallelogram: false,
  bevel_differential_wrist: false,
  vacuum_eoat: false,
  base_yaw_deg: 0,
  shoulder_joint_deg: 35,
  elbow_joint_deg: -55,
  wrist_joint_deg: 20,
  gripper_open_pct: 50,
  gear_ratio_shoulder: 20,
  gear_ratio_elbow: 10,
  max_accel_rad_s2: 12,
  material_id: "petg_cf",
  print_orientation: "xy_flat",
  printer_id: "bambu_x1c",
};

interface StudioState {
  family: FamilyId;
  link: typeof DEFAULT_LINK;
  mount: typeof DEFAULT_MOUNT;
  gear: typeof DEFAULT_GEAR;
  arm: typeof DEFAULT_ARM;
}

const INITIAL_STATE: StudioState = {
  family: "links",
  link: DEFAULT_LINK,
  mount: DEFAULT_MOUNT,
  gear: DEFAULT_GEAR,
  arm: DEFAULT_ARM,
};

export default function App() {
  // Undo / Redo History Stack (Eje 5)
  const [history, setHistory] = useState<StudioState[]>([INITIAL_STATE]);
  const [histIdx, setHistIdx] = useState(0);
  const state = history[histIdx];

  const pushState = useCallback(
    (updater: (prev: StudioState) => StudioState) => {
      setHistory((prevHist) => {
        const current = prevHist[histIdx];
        const next = updater(current);
        const sliced = prevHist.slice(0, histIdx + 1);
        const updated = [...sliced, next].slice(-40);
        setHistIdx(updated.length - 1);
        return updated;
      });
    },
    [histIdx]
  );

  const canUndo = histIdx > 0;
  const canRedo = histIdx < history.length - 1;
  const undo = useCallback(() => setHistIdx((i) => Math.max(0, i - 1)), []);
  const redo = useCallback(() => setHistIdx((i) => Math.min(history.length - 1, i + 1)), [history.length]);

  // Catalogs
  const [materials, setMaterials] = useState<Record<string, any>>({});
  const [printers, setPrinters] = useState<Record<string, any>>({});
  const [presets, setPresets] = useState<{ links: any; mounts: any; gears: any; arms: any }>({
    links: {},
    mounts: {},
    gears: {},
    arms: {},
  });

  // Generation & Simulation Results
  const [genRes, setGenRes] = useState<GenerateResponse | null>(null);
  const [armRes, setArmRes] = useState<ArmAssemblyResponse | null>(null);
  const [activeVariant, setActiveVariant] = useState<VariantResult | null>(null);
  const [feaRes, setFeaRes] = useState<FEAResult | null>(null);
  const [gyroidRes, setGyroidRes] = useState<GyroidResult | null>(null);
  const [viewingFea, setViewingFea] = useState(false);
  const [viewingGyroid, setViewingGyroid] = useState(false);

  // Viewport flags
  const [wireframe, setWireframe] = useState(false);
  const [autoRotate, setAutoRotate] = useState(false);
  const [showLoads, setShowLoads] = useState(true);
  const [splitCompare, setSplitCompare] = useState(false);
  const [meshOpacity, setMeshOpacity] = useState(1.0);
  const [loadDir, setLoadDir] = useState<"y_inplane" | "z_outofplane">("y_inplane");
  const [deformScale, setDeformScale] = useState(0);
  const [camPreset, setCamPreset] = useState<{ view: "iso" | "top" | "front" | "side"; nonce: number }>({
    view: "iso",
    nonce: 0,
  });

  // Ghost STEP & Assembly selection
  const [ghostUrl, setGhostUrl] = useState<string | null>(null);
  const [showGhost, setShowGhost] = useState(false);
  const [selectedArmNode, setSelectedArmNode] = useState<string | null>(null);

  // v3.0 Modular Arm (Mix & Match) + 60 FPS Exploded View
  const [armSlots, setArmSlots] = useState<ModularArmSlots>({
    shoulder_mount: null,
    shoulder_mount_variant: "mount_gusseted",
    shoulder_gear: null,
    shoulder_gear_variant: "gear_spoked",
    upper_arm: null,
    upper_arm_variant: "organic_ibeam",
    elbow_gear: null,
    elbow_gear_variant: "gear_webbed",
    forearm: null,
    forearm_variant: "hollow_cobot",
  });
  const [slotSummaries, setSlotSummaries] = useState<Partial<Record<ArmSlotKey, string>>>({});
  const [modularArmRes, setModularArmRes] = useState<ModularArmResponse | null>(null);
  const [isAssemblingModularArm, setIsAssemblingModularArm] = useState(false);
  const [explodeFactor, setExplodeFactor] = useState(0);
  const [hoveredArmPart, setHoveredArmPart] = useState<string | null>(null);

  // v4.0 Mechatronics Catalog & Config
  const [mechatronicsCatalog, setMechatronicsCatalog] = useState<MechatronicsCatalog | null>(null);
  const [mechatronicsConfig, setMechatronicsConfig] = useState<MechatronicsConfig>({
    bus_architecture: "can_fd_bus",
    system_voltage_v: 24,
    joint_actuators: {
      J1: { motor_id: "nema23_mks_servo57d", reducer_id: "planetary_20", target_rpm: 30, parallel_count: 1 },
      J2: { motor_id: "nema17_mks_servo42d", reducer_id: "planetary_10", target_rpm: 35, parallel_count: 1 },
      J3: { motor_id: "nema17_mks_servo42d", reducer_id: "gt2_belt_3", target_rpm: 45, parallel_count: 1 },
    },
  });

  // v5.0 Arm Database Modal
  const [armDatabase, setArmDatabase] = useState<ArmDatabaseResponse | null>(null);
  const [showArmDbModal, setShowArmDbModal] = useState(false);

  // v4.1/v4.2 Pose Sequencer (5-axis 60 FPS GPU FK playback + .INO export)
  const [recordedPoses, setRecordedPoses] = useState<
    Array<{
      id: number;
      label: string;
      j0_base: number;
      shoulder: number;
      elbow: number;
      wrist: number;
      gripper_pct: number;
    }>
  >([]);
  const [isPlayingSeq, setIsPlayingSeq] = useState(false);
  const [activePoseIdx, setActivePoseIdx] = useState<number | null>(null);
  const seqRafRef = useRef<number | null>(null);

  const assignedSlotCount = useMemo(() => {
    let n = 0;
    if (armSlots.shoulder_mount) n++;
    if (armSlots.shoulder_gear) n++;
    if (armSlots.upper_arm) n++;
    if (armSlots.elbow_gear) n++;
    if (armSlots.forearm) n++;
    return n;
  }, [armSlots]);

  // Eje 4 Copilot state
  const [copilotPrompt, setCopilotPrompt] = useState("");
  const [copilotVerifyFea, setCopilotVerifyFea] = useState(true);
  const [copilotInfo, setCopilotInfo] = useState<string | null>(null);

  // Status & Loading
  const [busyLabel, setBusyLabel] = useState<string | null>(null);
  const [toast, setToast] = useState<{ msg: string; err?: boolean } | null>(null);
  const showToast = useCallback((msg: string, err = false) => {
    setToast({ msg, err });
    setTimeout(() => setToast(null), 4200);
  }, []);

  // Keyboard shortcuts Ctrl+Z / Ctrl+Y / Ctrl+Enter
  useEffect(() => {
    const onKey = (e: KeyboardEvent) => {
      if ((e.ctrlKey || e.metaKey) && e.key.toLowerCase() === "z" && !e.shiftKey) {
        e.preventDefault();
        undo();
      } else if ((e.ctrlKey || e.metaKey) && (e.key.toLowerCase() === "y" || (e.shiftKey && e.key.toLowerCase() === "z"))) {
        e.preventDefault();
        redo();
      }
    };
    window.addEventListener("keydown", onKey);
    return () => window.removeEventListener("keydown", onKey);
  }, [undo, redo]);

  // Bootstrap catalogs once
  useEffect(() => {
    Promise.all([
      api.materials(),
      api.printers(),
      api.linkPresets(),
      api.mountPresets(),
      api.gearPresets(),
      api.armPresets(),
      api.getMechatronicsCatalog().catch(() => null),
      api.getArmDatabase().catch(() => null),
    ])
      .then(([m, p, lp, mp, gp, ap, mechCat, armDb]) => {
        setMaterials(m.materials || m);
        setPrinters(p.printers || p);
        setPresets({
          links: lp.architectures || lp,
          mounts: mp.actuators || mp,
          gears: gp.presets || gp,
          arms: ap.presets || ap,
        });
        if (mechCat) {
          setMechatronicsCatalog(mechCat);
        }
        if (armDb) {
          setArmDatabase(armDb);
        }
      })
      .catch((err) => showToast(`Error cargando catálogos: ${err.message}`, true));
  }, [showToast]);

  // Debounced auto-generation (skips FK angle, SCARA Z & gripper-only changes so arm stays 60 FPS!)
  const structuralKey = useMemo(() => {
    if (state.family === "arm") {
      const {
        base_yaw_deg,
        shoulder_joint_deg,
        elbow_joint_deg,
        wrist_joint_deg,
        gripper_open_pct,
        z_axis_mm,
        ...rest
      } = state.arm;
      return JSON.stringify({ f: "arm", rest });
    }
    return JSON.stringify({
      f: state.family,
      p: state.family === "mounts" ? state.mount : state.family === "gears" ? state.gear : state.link,
    });
  }, [state]);

  const runGeneration = useCallback(async () => {
    setBusyLabel("Sintetizando CAD B-Rep & Física...");
    setViewingFea(false);
    setViewingGyroid(false);
    setFeaRes(null);
    setGyroidRes(null);
    setMeshOpacity(1.0);
    try {
      if (state.family === "arm") {
        const res = await api.generateArm(state.arm);
        setArmRes(res);
        setModularArmRes(null);
        setGenRes(null);
        setActiveVariant(null);
      } else if (state.family === "mounts") {
        const res = await api.generateMounts(state.mount);
        setGenRes(res);
        setArmRes(null);
        const rec = res.variants.find((v) => v.variant_id === res.recommended_variant) || res.variants[0];
        setActiveVariant(rec || null);
      } else if (state.family === "gears") {
        const res = await api.generateGears(state.gear);
        setGenRes(res);
        setArmRes(null);
        const rec = res.variants.find((v) => v.variant_id === res.recommended_variant) || res.variants[0];
        setActiveVariant(rec || null);
      } else {
        const res = await api.generateLinks(state.link);
        setGenRes(res);
        setArmRes(null);
        const rec = res.variants.find((v) => v.variant_id === res.recommended_variant) || res.variants[0];
        setActiveVariant(rec || null);
      }
    } catch (err: any) {
      showToast(`Error generando modelo: ${err.message}`, true);
    } finally {
      setBusyLabel(null);
    }
  }, [state, showToast]);

  const genTimer = useRef<number | null>(null);
  useEffect(() => {
    if (genTimer.current) window.clearTimeout(genTimer.current);
    genTimer.current = window.setTimeout(() => {
      runGeneration();
    }, 360);
    return () => {
      if (genTimer.current) window.clearTimeout(genTimer.current);
    };
  }, [structuralKey]);

  const runFea = async (overrideDir?: "y_inplane" | "z_outofplane", overrideDef?: number) => {
    if (!genRes || !activeVariant || state.family === "arm") return;
    const dir = overrideDir ?? loadDir;
    const def = overrideDef ?? deformScale;
    setBusyLabel("Resolviendo FEA 3D Ortotrópico + Modal + Pandeo...");
    try {
      const reqPayload = state.family === "mounts" ? state.mount : state.family === "gears" ? state.gear : state.link;
      const res = await api.runFea(state.family, {
        design_hash: genRes.design_hash,
        variant_id: activeVariant.variant_id,
        request: reqPayload,
        mesh_size_mm: state.family === "gears" ? 4.5 : state.family === "mounts" ? 5.5 : 7.0,
        load_direction: dir,
        deform_scale: def,
      });
      setFeaRes(res);
      setViewingFea(true);
      setViewingGyroid(false);
      showToast(`FEA 3D listo (${res.element_count.toLocaleString("es")} tets, f₁=${res.first_natural_freq_hz ?? "—"} Hz)`);
    } catch (err: any) {
      showToast(`Error FEA 3D: ${err.message}`, true);
    } finally {
      setBusyLabel(null);
    }
  };

  const runGyroid = async () => {
    if (state.family !== "links") return;
    setBusyLabel("Generando Celosía Implícita SDF Gyroid (TPMS)...");
    try {
      const res = await api.generateGyroid({
        request: state.link,
        cell_size_mm: 14,
        shell_thickness_mm: 2.4,
        min_density: 0.28,
        max_density: 0.68,
        voxel_size_mm: 1.6,
      });
      setGyroidRes(res);
      setViewingGyroid(true);
      setViewingFea(false);
      showToast(`Gyroid SDF listo (${res.mass_g} g, ${res.triangle_count.toLocaleString("es")} tris en ${res.solve_time_s}s)`);
    } catch (err: any) {
      showToast(`Error Gyroid SDF: ${err.message}`, true);
    } finally {
      setBusyLabel(null);
    }
  };

  const handleCopilot = async () => {
    if (!copilotPrompt.trim()) return;
    setBusyLabel(copilotVerifyFea ? "Copiloto IA + Verificación FEA 3D Real..." : "Copiloto IA Optimizando...");
    try {
      const res = await api.copilotOptimize({
        prompt: copilotPrompt.trim(),
        current_request: state.link,
        verify_with_fea: copilotVerifyFea,
      });
      setCopilotInfo(res.summary);
      pushState((s) => ({ ...s, family: "links", link: { ...s.link, ...res.optimized_request } }));
      if (res.fea_result) {
        setFeaRes(res.fea_result);
      }
      showToast(res.fea_verified ? "Copiloto convergido y validado con FEA 3D." : "Copiloto convergido.");
    } catch (err: any) {
      showToast(`Error Copiloto: ${err.message}`, true);
    } finally {
      setBusyLabel(null);
    }
  };

  const handleBlueprintUpload = async (file: File) => {
    setBusyLabel(`Analizando plano 2D (${file.name})...`);
    try {
      const res = await api.analyzeBlueprint(file, copilotPrompt, copilotVerifyFea);
      setCopilotInfo(`${res.notes.join(" ")} — ${res.copilot_result.summary}`);
      pushState((s) => ({
        ...s,
        family: "links",
        link: { ...s.link, ...res.copilot_result.optimized_request },
      }));
      showToast(`Plano 2D analizado: L/A=${res.detected_aspect_ratio}, ${res.detected_holes_count} perforaciones.`);
    } catch (err: any) {
      showToast(`Error leyendo plano 2D: ${err.message}`, true);
    } finally {
      setBusyLabel(null);
    }
  };

  const handleStepUpload = async (file: File) => {
    setBusyLabel(`Analizando referencia .STEP (${file.name})...`);
    try {
      const res = await api.analyzeStep(file);
      if (res.reference_glb_url) {
        setGhostUrl(res.reference_glb_url);
        setShowGhost(true);
      }
      pushState((s) => ({ ...s, family: "links", link: { ...s.link, ...res.suggested_request } }));
      showToast(`Referencia .STEP cargada (${res.volume_cm3.toFixed(1)} cm³).`);
    } catch (err: any) {
      showToast(`Error .STEP: ${err.message}`, true);
    } finally {
      setBusyLabel(null);
    }
  };

  const assignToArmSlot = useCallback(
    (slot: ArmSlotKey, payload: Record<string, any>, variantId: string, summary: string) => {
      const variantKey = `${slot}_variant` as keyof ModularArmSlots;
      setArmSlots((prev) => ({
        ...prev,
        [slot]: structuredClone(payload),
        [variantKey]: variantId,
      }));
      setSlotSummaries((prev) => ({ ...prev, [slot]: summary }));
      const slotNames: Record<ArmSlotKey, string> = {
        shoulder_mount: "Montura Hombro",
        shoulder_gear: "Transmisión Hombro",
        upper_arm: "Brazo Superior",
        elbow_gear: "Transmisión Codo",
        forearm: "Antebrazo",
      };
      showToast(`Asignado a ${slotNames[slot]} (${summary}) en Brazo Completo.`);
    },
    [showToast]
  );

  const clearArmSlot = useCallback((slot: ArmSlotKey) => {
    setArmSlots((prev) => ({ ...prev, [slot]: null }));
    setSlotSummaries((prev) => {
      const next = { ...prev };
      delete next[slot];
      return next;
    });
  }, []);

  const buildModularArmRequest = useCallback(
    (autoFix = false): ModularArmRequest => ({
      dof: state.arm.dof,
      payload_kg: state.arm.payload_kg,
      material_id: state.arm.material_id,
      printer_id: state.arm.printer_id,
      base_height_mm: state.arm.base_height_mm,
      wrist_mm: state.arm.wrist_mm,
      max_accel_rad_s2: state.arm.max_accel_rad_s2,
      joint_angles_deg: [
        state.arm.shoulder_joint_deg,
        state.arm.elbow_joint_deg,
        state.arm.wrist_joint_deg,
      ],
      slots: armSlots,
      auto_fix: autoFix,
      mechatronics: mechatronicsConfig,
      kinematics_type: state.arm.kinematics_type,
      z_stroke_mm: state.arm.z_stroke_mm,
      wrist_3dof: state.arm.wrist_3dof,
      cnc_plate_mode: state.arm.cnc_plate_mode,
      remote_base_motors: state.arm.remote_base_motors,
      four_bar_parallelogram: state.arm.four_bar_parallelogram,
      bevel_differential_wrist: state.arm.bevel_differential_wrist,
      vacuum_eoat: state.arm.vacuum_eoat,
    }),
    [state.arm, armSlots, mechatronicsConfig]
  );

  const applyArmPresetWithMechatronics = useCallback(
    (presetId: string, presetDefaults: Record<string, any> = {}) => {
      let kinType: KinematicsType = (presetDefaults.kinematics_type as KinematicsType) || "articulated";
      if (presetId === "pybot_scara") kinType = "scara";
      else if (presetId === "lumen_pnp") kinType = "cartesian_gantry";
      else if (presetId === "delta_flexpicker") kinType = "delta_parallel";
      else if (presetId === "surgical_continuum") kinType = "continuum_pcc";

      const isCnc =
        presetId === "ar4_industrial" ||
        presetId === "barrett_wam" ||
        presetId === "dobot_4bar" ||
        Boolean(presetDefaults.cnc_plate_mode);
      const isCapstan = presetId === "barrett_wam" || Boolean(presetDefaults.remote_base_motors);
      const isFourBar = presetId === "dobot_4bar" || Boolean(presetDefaults.four_bar_parallelogram);
      const isBevel = presetId === "thor_6dof" || Boolean(presetDefaults.bevel_differential_wrist);
      const isVacuum =
        presetId === "lumen_pnp" ||
        presetId === "delta_flexpicker" ||
        presetId === "dobot_4bar" ||
        Boolean(presetDefaults.vacuum_eoat);

      pushState((s) => ({
        ...s,
        family: "arm",
        arm: {
          ...s.arm,
          ...presetDefaults,
          preset: presetId,
          kinematics_type: kinType,
          cnc_plate_mode: isCnc,
          wrist_3dof: kinType === "articulated",
          remote_base_motors: isCapstan,
          four_bar_parallelogram: isFourBar,
          bevel_differential_wrist: isBevel,
          vacuum_eoat: isVacuum,
        },
      }));

      if (presetId === "fabri_arm_5") {
        setMechatronicsConfig({
          bus_architecture: "arduino_pca9685_i2c",
          system_voltage_v: 6.5,
          joint_actuators: {
            J1: { motor_id: "servo_td8120mg", reducer_id: "direct", target_rpm: 35, parallel_count: 2 },
            J2: { motor_id: "servo_td8120mg", reducer_id: "direct", target_rpm: 35, parallel_count: 2 },
            J3: { motor_id: "servo_td8120mg", reducer_id: "direct", target_rpm: 45, parallel_count: 1 },
          },
        });
        showToast("Preset Brazo Arduino 5.0 (8× Servos TD-8120MG · PCA9685 6.5V) cargado.");
      } else if (presetId === "pybot_scara") {
        setMechatronicsConfig({
          bus_architecture: "stm32_step_dir",
          system_voltage_v: 24,
          joint_actuators: {
            J1: { motor_id: "nema17_48mm", reducer_id: "gt2_belt_3", target_rpm: 60, parallel_count: 1 },
            J2: { motor_id: "nema17_48mm", reducer_id: "gt2_belt_3", target_rpm: 60, parallel_count: 1 },
            J3: { motor_id: "nema17_48mm", reducer_id: "direct", target_rpm: 45, parallel_count: 1 },
          },
        });
        showToast("Preset PyBot SCARA 4-DOF (Eje Z Prismático + Correas GT2) cargado.");
      } else if (presetId === "so_arm_lerobot") {
        setMechatronicsConfig({
          bus_architecture: "uart_serial_bus",
          system_voltage_v: 7.4,
          joint_actuators: {
            J1: { motor_id: "servo_sts3215", reducer_id: "direct", target_rpm: 40, parallel_count: 1 },
            J2: { motor_id: "servo_sts3215", reducer_id: "direct", target_rpm: 40, parallel_count: 1 },
            J3: { motor_id: "servo_sts3215", reducer_id: "direct", target_rpm: 45, parallel_count: 1 },
          },
        });
        showToast("Preset SO-ARM100 LeRobot (Bus UART STS3215 · 7.4V) cargado.");
      } else if (presetId === "openarm_7dof") {
        setMechatronicsConfig({
          bus_architecture: "can_fd_bus",
          system_voltage_v: 24,
          joint_actuators: {
            J1: { motor_id: "damiao_dm_j4340", reducer_id: "direct", target_rpm: 35, parallel_count: 1 },
            J2: { motor_id: "damiao_dm_j4340", reducer_id: "direct", target_rpm: 35, parallel_count: 1 },
            J3: { motor_id: "damiao_dm_j4310", reducer_id: "direct", target_rpm: 45, parallel_count: 1 },
          },
        });
        showToast("Preset OpenArm 7-DOF Redundante (Damiao QDD CAN-FD) cargado.");
      } else if (presetId === "arctos_6dof" || presetId === "cobot_hollow_ur") {
        setMechatronicsConfig({
          bus_architecture: "can_fd_bus",
          system_voltage_v: 24,
          joint_actuators: {
            J1: { motor_id: "nema23_mks_servo57d", reducer_id: "cycloidal_25", target_rpm: 30, parallel_count: 1 },
            J2: { motor_id: "nema17_mks_servo42d", reducer_id: "cycloidal_25", target_rpm: 35, parallel_count: 1 },
            J3: { motor_id: "nema17_mks_servo42d", reducer_id: "planetary_10", target_rpm: 45, parallel_count: 1 },
          },
        });
        showToast("Preset Cobot Cicloidal (Reductores Cicloidales 25:1 · CAN-FD) cargado.");
      } else if (presetId === "ar4_industrial") {
        setMechatronicsConfig({
          bus_architecture: "teensy_dm542",
          system_voltage_v: 24,
          joint_actuators: {
            J1: { motor_id: "nema23_mks_servo57d", reducer_id: "planetary_20", target_rpm: 28, parallel_count: 1 },
            J2: { motor_id: "nema23_mks_servo57d", reducer_id: "planetary_20", target_rpm: 28, parallel_count: 1 },
            J3: { motor_id: "nema17_mks_servo42d", reducer_id: "planetary_10", target_rpm: 40, parallel_count: 1 },
          },
        });
        showToast("Preset AR4-MK3 Industrial (Teensy 4.1 + DM542T + CNC Al 6061 .DXF) cargado.");
      } else if (presetId === "bcn3d_moveo") {
        setMechatronicsConfig({
          bus_architecture: "stm32_step_dir",
          system_voltage_v: 24,
          joint_actuators: {
            J1: { motor_id: "nema23_76mm", reducer_id: "gt2_belt_3", target_rpm: 30, parallel_count: 1 },
            J2: { motor_id: "nema17_48mm", reducer_id: "gt2_belt_3", target_rpm: 35, parallel_count: 1 },
            J3: { motor_id: "nema14_34mm", reducer_id: "gt2_belt_3", target_rpm: 45, parallel_count: 1 },
          },
        });
        showToast("Preset BCN3D Moveo (Correas GT2 + NEMA 23/17/14) cargado.");
      } else if (presetId === "thor_6dof") {
        setMechatronicsConfig({
          bus_architecture: "can_fd_bus",
          system_voltage_v: 24,
          joint_actuators: {
            J1: { motor_id: "nema17_mks_servo42d", reducer_id: "planetary_20", target_rpm: 32, parallel_count: 1 },
            J2: { motor_id: "nema17_mks_servo42d", reducer_id: "cycloidal_25", target_rpm: 32, parallel_count: 1 },
            J3: { motor_id: "nema14_34mm", reducer_id: "planetary_10", target_rpm: 45, parallel_count: 1 },
          },
        });
        showToast("Preset Thor 6-DOF / Niryo Ned2 (Muñeca Diferencial Bevel) cargado.");
      } else if (presetId === "dobot_4bar") {
        setMechatronicsConfig({
          bus_architecture: "stm32_step_dir",
          system_voltage_v: 24,
          joint_actuators: {
            J1: { motor_id: "nema17_48mm", reducer_id: "planetary_10", target_rpm: 40, parallel_count: 1 },
            J2: { motor_id: "nema17_48mm", reducer_id: "planetary_10", target_rpm: 40, parallel_count: 1 },
            J3: { motor_id: "nema14_34mm", reducer_id: "direct", target_rpm: 50, parallel_count: 1 },
          },
        });
        showToast("Preset Dobot Magician (Paralelogramo 4-Barras + Boquilla Vacío) cargado.");
      } else if (presetId === "lumen_pnp") {
        setMechatronicsConfig({
          bus_architecture: "stm32_step_dir",
          system_voltage_v: 24,
          joint_actuators: {
            J1: { motor_id: "nema17_48mm", reducer_id: "gt2_belt_3", target_rpm: 90, parallel_count: 1 },
            J2: { motor_id: "nema17_48mm", reducer_id: "gt2_belt_3", target_rpm: 90, parallel_count: 1 },
            J3: { motor_id: "nema14_34mm", reducer_id: "direct", target_rpm: 60, parallel_count: 1 },
          },
        });
        showToast("Preset LumenPnP (Pórtico Cartesiano X-Y-Z + Boquilla Vacío) cargado.");
      } else if (presetId === "delta_flexpicker") {
        setMechatronicsConfig({
          bus_architecture: "can_fd_bus",
          system_voltage_v: 48,
          joint_actuators: {
            J1: { motor_id: "bldc_qdd_8308", reducer_id: "planetary_10", target_rpm: 120, parallel_count: 1 },
            J2: { motor_id: "bldc_qdd_8308", reducer_id: "planetary_10", target_rpm: 120, parallel_count: 1 },
            J3: { motor_id: "bldc_qdd_8308", reducer_id: "planetary_10", target_rpm: 120, parallel_count: 1 },
          },
        });
        showToast("Preset ABB FlexPicker Delta 4-DOF (3 Brazos Paralelos + 48V) cargado.");
      } else if (presetId === "barrett_wam") {
        setMechatronicsConfig({
          bus_architecture: "can_fd_bus",
          system_voltage_v: 48,
          joint_actuators: {
            J1: { motor_id: "damiao_dm_j4340", reducer_id: "planetary_10", target_rpm: 40, parallel_count: 1 },
            J2: { motor_id: "damiao_dm_j4340", reducer_id: "planetary_10", target_rpm: 40, parallel_count: 1 },
            J3: { motor_id: "damiao_dm_j4310", reducer_id: "planetary_10", target_rpm: 50, parallel_count: 1 },
          },
        });
        showToast("Preset Barrett WAM (7-DOF · Motores en Base Capstan · 48V) cargado.");
      } else if (presetId === "surgical_continuum") {
        setMechatronicsConfig({
          bus_architecture: "can_fd_bus",
          system_voltage_v: 24,
          joint_actuators: {
            J1: { motor_id: "nema14_34mm", reducer_id: "direct", target_rpm: 30, parallel_count: 1 },
            J2: { motor_id: "nema14_34mm", reducer_id: "direct", target_rpm: 30, parallel_count: 1 },
            J3: { motor_id: "nema14_34mm", reducer_id: "direct", target_rpm: 30, parallel_count: 1 },
          },
        });
        showToast("Preset Robot Quirúrgico Continuo PCC (Nitinol + Discos PEEK) cargado.");
      }
    },
    [pushState, showToast]
  );

  const handleLoadArmFromDatabase = useCallback(
    (arm: ArmDatabaseEntry) => {
      const idMap: Record<string, string> = {
        fabri_creator_5_0: "fabri_arm_5",
        parol6_6dof: "parol6_desktop",
        annin_ar4_mk3: "ar4_industrial",
        pybot_jjrobots_scara: "pybot_scara",
        so_arm100_lerobot: "so_arm_lerobot",
        openarm_7dof: "openarm_7dof",
        arctos_6dof: "arctos_6dof",
        bcn3d_moveo: "bcn3d_moveo",
        thor_niryo_ned2: "thor_6dof",
        dobot_magician_4bar: "dobot_4bar",
        lumenpnp_cartesian: "lumen_pnp",
        abb_irb360_flexpicker_delta: "delta_flexpicker",
        barrett_wam_ambidex: "barrett_wam",
        continuum_tendon_surgical: "surgical_continuum",
      };
      const presetId = arm.roboforge_preset_id || idMap[arm.id] || "parol6_desktop";
      const presetObj = presets.arms?.[presetId];
      const baseDefaults = presetObj?.defaults || presetObj || {};
      const kt = (arm.kinematics_type || "").toLowerCase();
      let kinType: KinematicsType = "articulated";
      if (kt.includes("scara")) kinType = "scara";
      else if (kt.includes("cartesian")) kinType = "cartesian_gantry";
      else if (kt.includes("delta")) kinType = "delta_parallel";
      else if (kt.includes("continuum")) kinType = "continuum_pcc";

      applyArmPresetWithMechatronics(presetId, {
        ...baseDefaults,
        dof: Math.min(Math.max(arm.dof || 6, 4), 7),
        payload_kg: Math.max(0.1, arm.payload_kg || baseDefaults.payload_kg || 1.0),
        kinematics_type: kinType,
      });
    },
    [presets.arms, applyArmPresetWithMechatronics]
  );

  const handleAutoMatchArm = useCallback(async () => {
    setIsAssemblingModularArm(true);
    setBusyLabel("Auto-ajustando interfaces mecánicas del brazo...");
    try {
      const res = await api.autoMatchArm(buildModularArmRequest(true));
      setModularArmRes(res);
      if (res.slots_used) {
        setArmSlots(res.slots_used);
      }
      showToast(
        res.auto_fix_notes.length > 0
          ? `Interfaces ajustadas (${res.auto_fix_notes.length} cambios).`
          : "Todas las interfaces ya son compatibles."
      );
    } catch (err: any) {
      showToast(`Error auto-ajustando interfaces: ${err.message}`, true);
    } finally {
      setIsAssemblingModularArm(false);
      setBusyLabel(null);
    }
  }, [buildModularArmRequest, showToast]);

  const handleAssembleModularArm = useCallback(async () => {
    setIsAssemblingModularArm(true);
    setBusyLabel("Ensamblando Brazo Real (build123d + Kit .ZIP)...");
    try {
      const res = await api.assembleModularArm(buildModularArmRequest(false));
      setModularArmRes(res);
      setExplodeFactor(0);
      showToast(
        res.compatible
          ? `Brazo modular ensamblado (${(res.total_mass_g / 1000).toFixed(2)} kg, FS mín ${res.min_safety_factor.toFixed(2)}×).`
          : `Brazo ensamblado con ${res.compatibility_issues.length} alertas de compatibilidad.`,
        !res.compatible
      );
    } catch (err: any) {
      showToast(`Error ensamblando brazo real: ${err.message}`, true);
    } finally {
      setIsAssemblingModularArm(false);
      setBusyLabel(null);
    }
  }, [buildModularArmRequest, showToast]);

  // 60 FPS smooth Pose Sequencer playback loop
  useEffect(() => {
    if (!isPlayingSeq || recordedPoses.length < 2) {
      if (seqRafRef.current) cancelAnimationFrame(seqRafRef.current);
      seqRafRef.current = null;
      return;
    }

    const segDurationMs = 950;
    let startTs = performance.now();
    let segIdx = 0;

    const tick = (now: number) => {
      const elapsed = now - startTs;
      const rawT = Math.min(elapsed / segDurationMs, 1);
      // Cubic ease-in-out
      const ease = rawT < 0.5 ? 4 * rawT * rawT * rawT : 1 - Math.pow(-2 * rawT + 2, 3) / 2;

      const from = recordedPoses[segIdx];
      const to = recordedPoses[(segIdx + 1) % recordedPoses.length];
      setActivePoseIdx(segIdx);

      const b = Math.round(from.j0_base + (to.j0_base - from.j0_base) * ease);
      const s = Math.round(from.shoulder + (to.shoulder - from.shoulder) * ease);
      const e = Math.round(from.elbow + (to.elbow - from.elbow) * ease);
      const w = Math.round(from.wrist + (to.wrist - from.wrist) * ease);
      const g = Math.round(from.gripper_pct + (to.gripper_pct - from.gripper_pct) * ease);

      setHistory((prevHist) =>
        prevHist.map((item, idx) =>
          idx === histIdx
            ? {
                ...item,
                arm: {
                  ...item.arm,
                  base_yaw_deg: b,
                  shoulder_joint_deg: s,
                  elbow_joint_deg: e,
                  wrist_joint_deg: w,
                  gripper_open_pct: g,
                },
              }
            : item
        )
      );

      if (rawT >= 1) {
        segIdx = (segIdx + 1) % recordedPoses.length;
        startTs = now;
      }
      seqRafRef.current = requestAnimationFrame(tick);
    };

    seqRafRef.current = requestAnimationFrame(tick);
    return () => {
      if (seqRafRef.current) cancelAnimationFrame(seqRafRef.current);
      seqRafRef.current = null;
    };
  }, [isPlayingSeq, recordedPoses, histIdx]);

  const handleExportIno = useCallback(async () => {
    const posesPayload =
      recordedPoses.length > 0
        ? recordedPoses.map((p) => ({
            j0_base: p.j0_base,
            j1_shoulder: p.shoulder,
            j2_elbow: p.elbow,
            j3_wrist: p.wrist,
            gripper_pct: p.gripper_pct,
          }))
        : [
            {
              j0_base: state.arm.base_yaw_deg,
              j1_shoulder: state.arm.shoulder_joint_deg,
              j2_elbow: state.arm.elbow_joint_deg,
              j3_wrist: state.arm.wrist_joint_deg,
              gripper_pct: state.arm.gripper_open_pct,
            },
          ];

    try {
      const inoText = await api.exportFirmwareIno({
        arm_name: armRes?.preset_name || state.arm.preset || "Brazo Arduino 5.0",
        poses: posesPayload,
        speed_ms: 18,
      });
      const blob = new Blob([inoText], { type: "text/plain;charset=utf-8" });
      const url = URL.createObjectURL(blob);
      const a = document.createElement("a");
      a.href = url;
      a.download = "brazo_5_0_secuencia.ino";
      a.click();
      URL.revokeObjectURL(url);
      showToast(`Firmware Arduino descargado (${posesPayload.length} poses).`);
    } catch (err: any) {
      showToast(`Error exportando .INO: ${err.message}`, true);
    }
  }, [recordedPoses, state.arm, armRes, showToast]);

  // Live 60 FPS TCP readout for Brazo Completo
  const liveTcp = useMemo(() => {
    const a1 = (state.arm.shoulder_joint_deg * Math.PI) / 180;
    const a2 = ((state.arm.shoulder_joint_deg + state.arm.elbow_joint_deg) * Math.PI) / 180;
    const a3 = ((state.arm.shoulder_joint_deg + state.arm.elbow_joint_deg + state.arm.wrist_joint_deg) * Math.PI) / 180;
    const x = state.arm.upper_arm_mm * Math.cos(a1) + state.arm.forearm_mm * Math.cos(a2) + state.arm.wrist_mm * Math.cos(a3);
    const z = state.arm.base_height_mm + state.arm.upper_arm_mm * Math.sin(a1) + state.arm.forearm_mm * Math.sin(a2) + state.arm.wrist_mm * Math.sin(a3);
    return { x, z };
  }, [state.arm]);

  const activeGlbUrl =
    state.family === "arm"
      ? modularArmRes?.files?.glb || armRes?.glb_url || null
      : viewingGyroid && gyroidRes
        ? gyroidRes.glb_url
        : viewingFea && feaRes
          ? feaRes.fea_glb_url
          : activeVariant?.glb_url || null;

  const secondaryGlbUrl = useMemo(() => {
    if (!splitCompare || !genRes || genRes.variants.length < 2 || !activeVariant) return null;
    const base = genRes.variants[0];
    return (base.variant_id === activeVariant.variant_id ? genRes.variants[1] : base).glb_url;
  }, [splitCompare, genRes, activeVariant]);

  const saveProject = () => {
    const blob = new Blob([JSON.stringify({ format: "roboforge-project", version: "2.0", state }, null, 2)], {
      type: "application/json",
    });
    const url = URL.createObjectURL(blob);
    const a = document.createElement("a");
    a.href = url;
    a.download = `roboforge_${state.family}.rforge`;
    a.click();
    URL.revokeObjectURL(url);
    showToast("Proyecto guardado (.rforge)");
  };

  const openProject = async (file: File) => {
    try {
      const data = JSON.parse(await file.text());
      if (data.state) {
        pushState(() => data.state);
        showToast(`Proyecto ${file.name} restaurado.`);
      }
    } catch (err: any) {
      showToast(`Archivo .rforge inválido: ${err.message}`, true);
    }
  };

  return (
    <div className="app-shell">
      {/* HEADER */}
      <header className="top-header">
        <div className="header-left">
          <div className="brand">
            <span>RoboForge</span>
            <span className="brand-badge">PRO v6.0 · R3F</span>
          </div>
          <nav className="family-tabs">
            {(
              [
                ["links", "Eslabones Robóticos"],
                ["mounts", "Monturas Mecatrónicas"],
                ["gears", "Transmisión"],
                ["arm", "Brazo Completo"],
              ] as Array<[FamilyId, string]>
            ).map(([fid, label]) => (
              <button
                key={fid}
                type="button"
                className={`family-tab ${state.family === fid ? "active" : ""}`}
                onClick={() => pushState((s) => ({ ...s, family: fid }))}
              >
                {label}
                {fid === "arm" && assignedSlotCount > 0 && (
                  <span className="tab-count-badge" title={`${assignedSlotCount} piezas asignadas al ensamblador modular`}>
                    {assignedSlotCount}
                  </span>
                )}
              </button>
            ))}
          </nav>
        </div>

        <div className="header-right">
          <button type="button" className="btn-tool" disabled={!canUndo} onClick={undo} title="Deshacer (Ctrl+Z)">
            ↶ Undo
          </button>
          <button type="button" className="btn-tool" disabled={!canRedo} onClick={redo} title="Rehacer (Ctrl+Y)">
            ↷ Redo
          </button>
          <button type="button" className="btn-tool" onClick={saveProject}>
            Guardar .rforge
          </button>
          <label className="btn-tool" style={{ cursor: "pointer" }}>
            Abrir
            <input
              type="file"
              accept=".rforge,.json"
              hidden
              onChange={(e) => {
                const f = e.target.files?.[0];
                if (f) openProject(f);
                e.target.value = "";
              }}
            />
          </label>
          <button type="button" className="btn-primary" disabled={Boolean(busyLabel)} onClick={runGeneration}>
            {busyLabel ? "Calculando..." : "Regenerar"}
          </button>
        </div>
      </header>

      {/* 3-COLUMN STUDIO */}
      <div className="studio-grid">
        {/* LEFT INSPECTOR */}
        <aside className="sidebar">
          {state.family === "links" && (
            <>
              {/* EJE 4: COPILOTO IA MULTIMODAL + FEA 3D */}
              <div className="section-card">
                <div className="section-title">
                  <span>Copiloto IA (FEA 3D + Planos 2D)</span>
                </div>
                <input
                  type="text"
                  placeholder="Ej. Brazo 220mm, 35N, PA12-CF, FS >= 2.5"
                  value={copilotPrompt}
                  onChange={(e) => setCopilotPrompt(e.target.value)}
                />
                <label className="check-row">
                  <span>Verificar bucle con FEA 3D Real</span>
                  <input
                    type="checkbox"
                    checked={copilotVerifyFea}
                    onChange={(e) => setCopilotVerifyFea(e.target.checked)}
                  />
                </label>
                <div className="two-col">
                  <button type="button" className="btn-primary" onClick={handleCopilot}>
                    Optimizar IA
                  </button>
                  <label className="btn-tool" style={{ textAlign: "center", cursor: "pointer" }}>
                    Plano 2D / PDF
                    <input
                      type="file"
                      accept=".png,.jpg,.jpeg,.webp,.pdf"
                      hidden
                      onChange={(e) => {
                        const f = e.target.files?.[0];
                        if (f) handleBlueprintUpload(f);
                        e.target.value = "";
                      }}
                    />
                  </label>
                </div>
                {copilotInfo && <div className="alert-item info">{copilotInfo}</div>}
              </div>

              {/* .STEP REFERENCE */}
              <div className="section-card">
                <div className="section-title">
                  <span>Referencia .STEP + Fantasma</span>
                </div>
                <label className="btn-tool" style={{ textAlign: "center", cursor: "pointer" }}>
                  Subir .STEP de Referencia
                  <input
                    type="file"
                    accept=".step,.stp"
                    hidden
                    onChange={(e) => {
                      const f = e.target.files?.[0];
                      if (f) handleStepUpload(f);
                      e.target.value = "";
                    }}
                  />
                </label>
              </div>

              {/* LINK ARCHITECTURE */}
              <div className="section-card">
                <div className="section-title">
                  <span>01 · Arquitectura del Brazo</span>
                </div>
                <div className="preset-grid">
                  {Object.entries(presets.links).map(([id, p]: [string, any]) => (
                    <button
                      key={id}
                      type="button"
                      className={`preset-btn ${state.link.architecture === id ? "active" : ""}`}
                      onClick={() =>
                        pushState((s) => ({ ...s, link: { ...s.link, ...(p.defaults || p), architecture: id } }))
                      }
                    >
                      <strong>{p.name || id}</strong>
                      <small>{p.subtitle || ""}</small>
                    </button>
                  ))}
                </div>
              </div>

              {/* LINK GEOMETRY */}
              <div className="section-card">
                <div className="section-title">
                  <span>02 · Geometría y Perfil Orgánico</span>
                </div>
                {(
                  [
                    ["length_mm", "Longitud L", 60, 400, 5, "mm"],
                    ["root_width_mm", "Ancho Raíz", 18, 95, 1, "mm"],
                    ["tip_width_mm", "Ancho Punta", 14, 75, 1, "mm"],
                    ["thickness_mm", "Espesor", 8, 50, 1, "mm"],
                  ] as const
                ).map(([key, label, min, max, step, unit]) => (
                  <div key={key} className="field-row">
                    <div className="field-head">
                      <span>{label}</span>
                      <span>
                        {(state.link as any)[key]} {unit}
                      </span>
                    </div>
                    <input
                      type="range"
                      min={min}
                      max={max}
                      step={step}
                      value={(state.link as any)[key]}
                      onChange={(e) =>
                        pushState((s) => ({ ...s, link: { ...s.link, [key]: Number(e.target.value) } }))
                      }
                    />
                  </div>
                ))}
                <div className="field-row">
                  <div className="field-head">
                    <span>Nivel Orgánico</span>
                    <span>{Math.round(state.link.organic_taper * 100)}%</span>
                  </div>
                  <input
                    type="range"
                    min={0}
                    max={100}
                    step={5}
                    value={Math.round(state.link.organic_taper * 100)}
                    onChange={(e) =>
                      pushState((s) => ({
                        ...s,
                        link: { ...s.link, organic_taper: Number(e.target.value) / 100 },
                      }))
                    }
                  />
                </div>
                <div className="two-col">
                  <div className="field-row">
                    <div className="field-head">
                      <span>Carga (N)</span>
                    </div>
                    <input
                      type="number"
                      value={state.link.tip_load_n}
                      onChange={(e) =>
                        pushState((s) => ({ ...s, link: { ...s.link, tip_load_n: Number(e.target.value) || 10 } }))
                      }
                    />
                  </div>
                  <div className="field-row">
                    <div className="field-head">
                      <span>FS Objetivo</span>
                    </div>
                    <input
                      type="number"
                      step={0.1}
                      value={state.link.target_safety_factor}
                      onChange={(e) =>
                        pushState((s) => ({
                          ...s,
                          link: { ...s.link, target_safety_factor: Number(e.target.value) || 2.0 },
                        }))
                      }
                    />
                  </div>
                </div>
              </div>
            </>
          )}

          {state.family === "mounts" && (
            <div className="section-card">
              <div className="section-title">
                <span>Monturas Mecatrónicas</span>
              </div>
              <div className="preset-grid">
                {Object.entries(presets.mounts).map(([id, p]: [string, any]) => (
                  <button
                    key={id}
                    type="button"
                    className={`preset-btn ${state.mount.actuator_type === id ? "active" : ""}`}
                    onClick={() =>
                      pushState((s) => ({ ...s, mount: { ...s.mount, ...(p.defaults || p), actuator_type: id } }))
                    }
                  >
                    <strong>{p.name || id}</strong>
                    <small>{p.subtitle || ""}</small>
                  </button>
                ))}
              </div>
              <div className="field-row">
                <div className="field-head">
                  <span>Espesor Pared</span>
                  <span>{state.mount.wall_thickness_mm} mm</span>
                </div>
                <input
                  type="range"
                  min={3}
                  max={14}
                  step={0.5}
                  value={state.mount.wall_thickness_mm}
                  onChange={(e) =>
                    pushState((s) => ({ ...s, mount: { ...s.mount, wall_thickness_mm: Number(e.target.value) } }))
                  }
                />
              </div>
              <div className="field-row">
                <div className="field-head">
                  <span>Altura Eje</span>
                  <span>{state.mount.shaft_height_mm} mm</span>
                </div>
                <input
                  type="range"
                  min={22}
                  max={80}
                  step={1}
                  value={state.mount.shaft_height_mm}
                  onChange={(e) =>
                    pushState((s) => ({ ...s, mount: { ...s.mount, shaft_height_mm: Number(e.target.value) } }))
                  }
                />
              </div>
              <div className="two-col">
                <div className="field-row">
                  <div className="field-head"><span>Carga Radial (N)</span></div>
                  <input
                    type="number"
                    value={state.mount.radial_load_n}
                    onChange={(e) =>
                      pushState((s) => ({ ...s, mount: { ...s.mount, radial_load_n: Number(e.target.value) || 20 } }))
                    }
                  />
                </div>
                <div className="field-row">
                  <div className="field-head"><span>Temp. Motor (°C)</span></div>
                  <input
                    type="number"
                    value={state.mount.motor_temp_c}
                    onChange={(e) =>
                      pushState((s) => ({ ...s, mount: { ...s.mount, motor_temp_c: Number(e.target.value) || 45 } }))
                    }
                  />
                </div>
              </div>
            </div>
          )}

          {state.family === "gears" && (
            <div className="section-card">
              <div className="section-title">
                <span>Transmisión (Involuta & GT2)</span>
              </div>
              <div className="preset-grid">
                {Object.entries(presets.gears).map(([id, p]: [string, any]) => (
                  <button
                    key={id}
                    type="button"
                    className={`preset-btn ${state.gear.preset === id ? "active" : ""}`}
                    onClick={() =>
                      pushState((s) => ({ ...s, gear: { ...s.gear, ...(p.defaults || p), preset: id } }))
                    }
                  >
                    <strong>{p.name || id}</strong>
                    <small>{p.subtitle || ""}</small>
                  </button>
                ))}
              </div>
              <div className="field-row">
                <div className="field-head">
                  <span>Dientes (Z)</span>
                  <span>{state.gear.teeth}</span>
                </div>
                <input
                  type="range"
                  min={14}
                  max={60}
                  step={1}
                  value={state.gear.teeth}
                  onChange={(e) =>
                    pushState((s) => ({ ...s, gear: { ...s.gear, teeth: Number(e.target.value) } }))
                  }
                />
              </div>
              <div className="field-row">
                <div className="field-head">
                  <span>Módulo (m)</span>
                  <span>{state.gear.module_mm} mm</span>
                </div>
                <input
                  type="range"
                  min={1.0}
                  max={4.0}
                  step={0.25}
                  value={state.gear.module_mm}
                  onChange={(e) =>
                    pushState((s) => ({ ...s, gear: { ...s.gear, module_mm: Number(e.target.value) } }))
                  }
                />
              </div>
              <div className="two-col">
                <div className="field-row">
                  <div className="field-head"><span>Torque (N·m)</span></div>
                  <input
                    type="number"
                    step={0.1}
                    value={state.gear.torque_nm}
                    onChange={(e) =>
                      pushState((s) => ({ ...s, gear: { ...s.gear, torque_nm: Number(e.target.value) || 1 } }))
                    }
                  />
                </div>
                <div className="field-row">
                  <div className="field-head"><span>Backlash (mm)</span></div>
                  <input
                    type="number"
                    step={0.05}
                    value={state.gear.backlash_mm}
                    onChange={(e) =>
                      pushState((s) => ({ ...s, gear: { ...s.gear, backlash_mm: Number(e.target.value) || 0.15 } }))
                    }
                  />
                </div>
              </div>
            </div>
          )}

          {state.family === "arm" && (
            <>
              <div className="section-card">
                <div className="section-title">
                  <span>Brazo Completo (4–7 DOF · SCARA)</span>
                </div>
                <button
                  type="button"
                  className="btn-tool btn-arm-db"
                  onClick={() => setShowArmDbModal(true)}
                  title="Explorar los 14 brazos robóticos reales y cargar en 1 clic"
                >
                  📚 Base de Datos ({armDatabase?.arms?.length || 14} Brazos Reales)
                </button>
                <div className="preset-grid">
                  {Object.entries(presets.arms).map(([id, p]: [string, any]) => (
                    <button
                      key={id}
                      type="button"
                      className={`preset-btn ${state.arm.preset === id ? "active" : ""}`}
                      onClick={() => applyArmPresetWithMechatronics(id, p.defaults || p)}
                    >
                      <strong>{p.name || id}</strong>
                      <small>{p.subtitle || ""}</small>
                    </button>
                  ))}
                </div>
                <div className="two-col">
                  <div className="field-row">
                    <div className="field-head"><span>Cinemática</span></div>
                    <select
                      value={state.arm.kinematics_type}
                      onChange={(e) => {
                        const kt = e.target.value as KinematicsType;
                        pushState((s) => ({
                          ...s,
                          arm: {
                            ...s.arm,
                            kinematics_type: kt,
                            dof:
                              kt === "scara" || kt === "cartesian_gantry" || kt === "delta_parallel"
                                ? 4
                                : kt === "continuum_pcc"
                                  ? 7
                                  : s.arm.dof,
                          },
                        }));
                      }}
                    >
                      <option value="articulated">Articulado (4–7 DOF)</option>
                      <option value="scara">SCARA 4-DOF (Z Prismático)</option>
                      <option value="cartesian_gantry">Pórtico Cartesiano X-Y-Z</option>
                      <option value="delta_parallel">Delta Paralelo 4-DOF</option>
                      <option value="continuum_pcc">Continuo PCC (Quirúrgico)</option>
                    </select>
                  </div>
                  <div className="field-row">
                    <div className="field-head"><span>DOF</span></div>
                    <select
                      value={state.arm.dof}
                      onChange={(e) =>
                        pushState((s) => ({ ...s, arm: { ...s.arm, dof: Number(e.target.value) } }))
                      }
                    >
                      <option value={4}>4 DOF</option>
                      <option value={5}>5 DOF</option>
                      <option value={6}>6 DOF</option>
                      <option value={7}>7 DOF (Redundante)</option>
                    </select>
                  </div>
                </div>
                <label className="check-row">
                  <span>Motores en Base (Capstan)</span>
                  <input
                    type="checkbox"
                    checked={state.arm.remote_base_motors}
                    onChange={(e) =>
                      pushState((s) => ({ ...s, arm: { ...s.arm, remote_base_motors: e.target.checked } }))
                    }
                  />
                </label>
                <label className="check-row">
                  <span>Paralelogramo 4-Barras</span>
                  <input
                    type="checkbox"
                    checked={state.arm.four_bar_parallelogram}
                    onChange={(e) =>
                      pushState((s) => ({ ...s, arm: { ...s.arm, four_bar_parallelogram: e.target.checked } }))
                    }
                  />
                </label>
                <label className="check-row">
                  <span>Muñeca Diferencial Bevel</span>
                  <input
                    type="checkbox"
                    checked={state.arm.bevel_differential_wrist}
                    onChange={(e) =>
                      pushState((s) => ({ ...s, arm: { ...s.arm, bevel_differential_wrist: e.target.checked } }))
                    }
                  />
                </label>
                <label className="check-row">
                  <span>Boquilla Vacío (EOAT)</span>
                  <input
                    type="checkbox"
                    checked={state.arm.vacuum_eoat}
                    onChange={(e) =>
                      pushState((s) => ({ ...s, arm: { ...s.arm, vacuum_eoat: e.target.checked } }))
                    }
                  />
                </label>
                <label className="check-row">
                  <span>Placas CNC Al 6061 (.DXF)</span>
                  <input
                    type="checkbox"
                    checked={state.arm.cnc_plate_mode}
                    onChange={(e) =>
                      pushState((s) => ({ ...s, arm: { ...s.arm, cnc_plate_mode: e.target.checked } }))
                    }
                  />
                </label>
                <label className="check-row">
                  <span>Muñeca Esférica 3-DOF (J4/J5/J6)</span>
                  <input
                    type="checkbox"
                    checked={state.arm.wrist_3dof}
                    onChange={(e) =>
                      pushState((s) => ({ ...s, arm: { ...s.arm, wrist_3dof: e.target.checked } }))
                    }
                  />
                </label>
                {(
                  [
                    ["upper_arm_mm", "Brazo Superior (L1)", 120, 350, 10],
                    ["forearm_mm", "Antebrazo (L2)", 100, 300, 10],
                    ["wrist_mm", "Muñeca (L3)", 50, 140, 5],
                  ] as const
                ).map(([k, lbl, min, max, step]) => (
                  <div key={k} className="field-row">
                    <div className="field-head">
                      <span>{lbl}</span>
                      <span>{(state.arm as any)[k]} mm</span>
                    </div>
                    <input
                      type="range"
                      min={min}
                      max={max}
                      step={step}
                      value={(state.arm as any)[k]}
                      onChange={(e) =>
                        pushState((s) => ({ ...s, arm: { ...s.arm, [k]: Number(e.target.value) } }))
                      }
                    />
                  </div>
                ))}
              </div>

              <div className="section-card">
                <div className="section-title">
                  <span>Cinemática Directa 60 FPS (GPU)</span>
                </div>
                {(
                  [
                    ["base_yaw_deg", "J0 Base (Yaw)", -180, 180, "°"],
                    ["shoulder_joint_deg", "J1 Hombro", -60, 90, "°"],
                    ["elbow_joint_deg", "J2 Codo", -120, 120, "°"],
                    ["wrist_joint_deg", "J3 Muñeca", -90, 90, "°"],
                    ["gripper_open_pct", "Pinza (Apertura)", 0, 100, "%"],
                  ] as const
                ).map(([k, lbl, min, max, unit]) => (
                  <div key={k} className="field-row">
                    <div className="field-head">
                      <span>{lbl}</span>
                      <span>
                        {(state.arm as any)[k]}
                        {unit}
                      </span>
                    </div>
                    <input
                      type="range"
                      min={min}
                      max={max}
                      step={1}
                      value={(state.arm as any)[k]}
                      onChange={(e) => {
                        setIsPlayingSeq(false);
                        pushState((s) => ({ ...s, arm: { ...s.arm, [k]: Number(e.target.value) } }));
                      }}
                    />
                  </div>
                ))}

                {(state.arm.kinematics_type === "scara" ||
                  state.arm.kinematics_type === "cartesian_gantry" ||
                  state.arm.kinematics_type === "delta_parallel") && (
                  <div className="field-row">
                    <div className="field-head">
                      <span>Eje Z Lineal ({state.arm.kinematics_type === "delta_parallel" ? "Delta" : state.arm.kinematics_type === "cartesian_gantry" ? "Pórtico" : "SCARA"})</span>
                      <span>{state.arm.z_axis_mm} mm</span>
                    </div>
                    <input
                      type="range"
                      min={0}
                      max={state.arm.z_stroke_mm || 120}
                      step={1}
                      value={state.arm.z_axis_mm}
                      onChange={(e) =>
                        pushState((s) => ({ ...s, arm: { ...s.arm, z_axis_mm: Number(e.target.value) } }))
                      }
                    />
                  </div>
                )}

                {/* v4.1/v4.2 Mini-Secuenciador de Poses (5 ejes, 60 FPS + Exportar .INO) */}
                <div className="sequencer-box">
                  <div className="sequencer-head">
                    <span>Secuenciador de Poses (5 Ejes)</span>
                    <small>{recordedPoses.length} grabadas</small>
                  </div>
                  <div className="sequencer-actions four-btn">
                    <button
                      type="button"
                      className="btn-tool"
                      onClick={() => {
                        setRecordedPoses((prev) => [
                          ...prev,
                          {
                            id: Date.now(),
                            label: `P${prev.length + 1}`,
                            j0_base: state.arm.base_yaw_deg,
                            shoulder: state.arm.shoulder_joint_deg,
                            elbow: state.arm.elbow_joint_deg,
                            wrist: state.arm.wrist_joint_deg,
                            gripper_pct: state.arm.gripper_open_pct,
                          },
                        ]);
                      }}
                    >
                      + Pose
                    </button>
                    <button
                      type="button"
                      className={`btn-tool ${isPlayingSeq ? "active" : ""}`}
                      disabled={recordedPoses.length < 2}
                      onClick={() => setIsPlayingSeq((v) => !v)}
                    >
                      {isPlayingSeq ? "■ Stop" : "▶ Play"}
                    </button>
                    <button
                      type="button"
                      className="btn-tool btn-ino"
                      onClick={handleExportIno}
                      title="Exportar sketch Arduino UNO + PCA9685 (.INO)"
                    >
                      ⬇ .INO
                    </button>
                    <button
                      type="button"
                      className="btn-tool"
                      disabled={recordedPoses.length === 0}
                      onClick={() => {
                        setIsPlayingSeq(false);
                        setActivePoseIdx(null);
                        setRecordedPoses([]);
                      }}
                    >
                      ✕
                    </button>
                  </div>
                  {recordedPoses.length > 0 && (
                    <div className="sequencer-chips">
                      {recordedPoses.map((pose, idx) => (
                        <button
                          key={pose.id}
                          type="button"
                          className={`pose-chip ${activePoseIdx === idx && isPlayingSeq ? "active" : ""}`}
                          onClick={() => {
                            setIsPlayingSeq(false);
                            setActivePoseIdx(idx);
                            pushState((s) => ({
                              ...s,
                              arm: {
                                ...s.arm,
                                base_yaw_deg: pose.j0_base,
                                shoulder_joint_deg: pose.shoulder,
                                elbow_joint_deg: pose.elbow,
                                wrist_joint_deg: pose.wrist,
                                gripper_open_pct: pose.gripper_pct,
                              },
                            }));
                          }}
                          title="Ir a esta pose"
                        >
                          <strong>{pose.label}</strong> {pose.j0_base}°/{pose.shoulder}°/{pose.elbow}° · G{pose.gripper_pct}%
                        </button>
                      ))}
                    </div>
                  )}
                </div>
              </div>
            </>
          )}

          {/* SHARED MATERIAL & PRINTER */}
          <div className="section-card">
            <div className="section-title">
              <span>Material e Impresora 3D</span>
            </div>
            <div className="field-row">
              <div className="field-head"><span>Impresora 3D</span></div>
              <select
                value={
                  state.family === "arm"
                    ? state.arm.printer_id || "bambu_x1c"
                    : state.family === "mounts"
                      ? state.mount.printer_id || "bambu_x1c"
                      : state.family === "gears"
                        ? state.gear.printer_id || "bambu_x1c"
                        : state.link.printer_id || "bambu_x1c"
                }
                onChange={(e) => {
                  const pid = e.target.value;
                  pushState((s) => ({
                    ...s,
                    link: { ...s.link, printer_id: pid },
                    mount: { ...s.mount, printer_id: pid },
                    gear: { ...s.gear, printer_id: pid },
                    arm: { ...s.arm, printer_id: pid },
                  }));
                }}
              >
                {Object.entries(printers).map(([id, p]: [string, any]) => (
                  <option key={id} value={id}>
                    {p.name || id} ({p.enclosed ? "Cerrada" : "Abierta"})
                  </option>
                ))}
              </select>
            </div>
            <div className="field-row">
              <div className="field-head"><span>Material Anisotrópico</span></div>
              <select
                value={
                  state.family === "arm"
                    ? state.arm.material_id
                    : state.family === "mounts"
                      ? state.mount.material_id
                      : state.family === "gears"
                        ? state.gear.material_id
                        : state.link.material_id
                }
                onChange={(e) => {
                  const mid = e.target.value;
                  pushState((s) => ({
                    ...s,
                    link: { ...s.link, material_id: mid },
                    mount: { ...s.mount, material_id: mid },
                    gear: { ...s.gear, material_id: mid },
                    arm: { ...s.arm, material_id: mid },
                  }));
                }}
              >
                {Object.entries(materials).map(([id, m]: [string, any]) => (
                  <option key={id} value={id}>
                    {m.name || id}
                  </option>
                ))}
              </select>
            </div>
          </div>
        </aside>

        {/* CENTER R3F 3D VIEWPORT */}
        <main className="viewport-wrap">
          <div className="vp-toolbar">
            {(["iso", "top", "front", "side"] as const).map((v) => (
              <button
                key={v}
                type="button"
                className={`btn-tool ${camPreset.view === v ? "active" : ""}`}
                onClick={() => setCamPreset((c) => ({ view: v, nonce: c.nonce + 1 }))}
              >
                {v.toUpperCase()}
              </button>
            ))}
            {state.family !== "arm" && (
              <button
                type="button"
                className={`btn-tool btn-fea ${viewingFea ? "active" : ""}`}
                onClick={() => (viewingFea ? setViewingFea(false) : runFea())}
                title={viewingFea ? "Volver al modelo CAD 3D" : "Ejecutar análisis FEA 3D (Von Mises)"}
              >
                {viewingFea ? "FEA 3D ✓" : "FEA 3D"}
              </button>
            )}
            {state.family === "links" && (
              <button type="button" className="btn-tool" onClick={runGyroid}>
                Gyroid SDF
              </button>
            )}
            {ghostUrl && (
              <button
                type="button"
                className={`btn-tool ${showGhost ? "active" : ""}`}
                onClick={() => setShowGhost((g) => !g)}
              >
                Fantasma .STEP
              </button>
            )}
            <button
              type="button"
              className={`btn-tool ${showLoads ? "active" : ""}`}
              onClick={() => setShowLoads((v) => !v)}
            >
              Cargas 3D
            </button>
            {state.family !== "arm" && (
              <button
                type="button"
                className={`btn-tool ${splitCompare ? "active" : ""}`}
                onClick={() => setSplitCompare((v) => !v)}
              >
                Comparar 2
              </button>
            )}
            <button
              type="button"
              className={`btn-tool ${wireframe ? "active" : ""}`}
              onClick={() => setWireframe((v) => !v)}
            >
              Wire
            </button>
            <button
              type="button"
              className={`btn-tool ${autoRotate ? "active" : ""}`}
              onClick={() => setAutoRotate((v) => !v)}
            >
              Rotar
            </button>
          </div>

          <div className="vp-hud">
            <strong>
              {state.family === "arm"
                ? `${armRes?.preset_name || "Brazo Completo"} (${state.arm.dof}-DOF)`
                : viewingGyroid && gyroidRes
                  ? "Celosía Implícita SDF Gyroid (TPMS)"
                  : activeVariant?.name || "RoboForge Studio Pro"}
            </strong>
            <small>
              {state.family === "arm"
                ? `TCP (FK): X ${liveTcp.x.toFixed(1)} · Y 0.0 · Z ${liveTcp.z.toFixed(1)} mm`
                : viewingGyroid && gyroidRes
                  ? `${gyroidRes.mass_g} g · ${gyroidRes.voxel_grid.join("×")} vóxeles · ${gyroidRes.triangle_count.toLocaleString("es")} tris`
                  : activeVariant
                    ? `${activeVariant.metrics.mass_g.toFixed(1)} g · FS ${activeVariant.metrics.safety_factor.toFixed(2)}× · Deflex. ${activeVariant.metrics.tip_deflection_mm.toFixed(2)} mm`
                    : "Listo"}
            </small>
          </div>

          <ViewportR3F
            glbUrl={activeGlbUrl}
            secondaryGlbUrl={secondaryGlbUrl}
            ghostGlbUrl={showGhost ? ghostUrl : null}
            isFeaMesh={viewingFea}
            wireframe={wireframe}
            autoRotate={autoRotate}
            showLoads={showLoads}
            meshOpacity={meshOpacity}
            family={state.family}
            loadDirection={loadDir}
            armAngles={{
              baseYaw: state.arm.base_yaw_deg,
              shoulder: state.arm.shoulder_joint_deg,
              elbow: state.arm.elbow_joint_deg,
              wrist: state.arm.wrist_joint_deg,
              gripperOpenPct: state.arm.gripper_open_pct,
              kinematicsType: state.arm.kinematics_type,
              zAxisMm: state.arm.z_axis_mm,
            }}
            armReachMm={state.arm.upper_arm_mm + state.arm.forearm_mm + state.arm.wrist_mm}
            armBaseHeightMm={state.arm.base_height_mm}
            selectedArmNode={selectedArmNode}
            onSelectArmNode={setSelectedArmNode}
            explodeVectors={modularArmRes?.explode_vectors || null}
            explodeFactor={explodeFactor}
            onExplodeFactorChange={setExplodeFactor}
            hoveredArmPart={hoveredArmPart}
            onHoverArmPart={setHoveredArmPart}
            cameraPreset={camPreset}
          />

          {busyLabel && (
            <div className="vp-loader">
              <div className="spinner" />
              <strong>{busyLabel}</strong>
            </div>
          )}
        </main>

        {/* RIGHT PANEL: PARETO + FEA + VARIANTS OR ASSEMBLY TREE */}
        <aside className="right-panel">
          {feaRes && state.family !== "arm" && (
            <div className="fea-card">
              <div className="pareto-head">
                <strong>FEA 3D Ortotrópico + Modal + Pandeo</strong>
                <span className={`badge ${feaRes.passes_fea ? "good" : "bad"}`}>
                  {feaRes.passes_fea ? "✓ Aprobado FEA" : "✕ Revisar"}
                </span>
              </div>
              <div className="metrics-grid">
                <div className="metric-box">
                  <span>FS FEA</span>
                  <strong className={feaRes.passes_fea ? "good" : "bad"}>{feaRes.fea_safety_factor}×</strong>
                </div>
                <div className="metric-box">
                  <span>Von Mises P95</span>
                  <strong>{feaRes.p95_von_mises_mpa} MPa</strong>
                </div>
                <div className="metric-box">
                  <span>Modal f₁</span>
                  <strong className={(feaRes.first_natural_freq_hz || 0) >= 45 ? "good" : "warn"}>
                    {feaRes.first_natural_freq_hz ?? "—"} Hz
                  </strong>
                </div>
                <div className="metric-box">
                  <span>Pandeo Pcr</span>
                  <strong className="good">{feaRes.buckling_critical_load_n ?? "—"} N</strong>
                </div>
                <div className="metric-box">
                  <span>Desplaz.</span>
                  <strong>{feaRes.max_displacement_mm.toFixed(2)} mm</strong>
                </div>
                <div className="metric-box">
                  <span>Tetraedros</span>
                  <strong>{feaRes.element_count.toLocaleString("es")}</strong>
                </div>
              </div>
              <div className="two-col">
                <select
                  value={loadDir}
                  onChange={(e) => {
                    const next = e.target.value as "y_inplane" | "z_outofplane";
                    setLoadDir(next);
                    runFea(next, deformScale);
                  }}
                >
                  <option value="y_inplane">Carga Y (plano)</option>
                  <option value="z_outofplane">Carga Z (fuera plano)</option>
                </select>
                <select
                  value={deformScale}
                  onChange={(e) => {
                    const next = Number(e.target.value);
                    setDeformScale(next);
                    runFea(loadDir, next);
                  }}
                >
                  <option value={0}>Deform. 0×</option>
                  <option value={10}>Deform. 10×</option>
                  <option value={25}>Deform. 25×</option>
                </select>
              </div>
              <div className="field-row">
                <div className="field-head">
                  <span>Opacidad Malla FEA</span>
                  <span>{Math.round(meshOpacity * 100)}%</span>
                </div>
                <input
                  type="range"
                  min={25}
                  max={100}
                  step={5}
                  value={Math.round(meshOpacity * 100)}
                  onChange={(e) => setMeshOpacity(Number(e.target.value) / 100)}
                />
              </div>
              <button type="button" className="btn-tool" onClick={() => setViewingFea((v) => !v)}>
                {viewingFea ? "Ver Modelo CAD" : "Ver Malla FEA 3D"}
              </button>
            </div>
          )}

          {gyroidRes && viewingGyroid && (
            <div className="fea-card">
              <div className="pareto-head">
                <strong>Celosía Implícita SDF Gyroid</strong>
                <span className="badge good">{gyroidRes.mass_g} g</span>
              </div>
              <div className="dl-row">
                <a className="dl-link" href={gyroidRes.stl_url} download>.STL</a>
                <a className="dl-link" href={gyroidRes.threemf_url} download>.3MF</a>
              </div>
            </div>
          )}

          {genRes && state.family !== "arm" && (
            <>
              <ParetoChart
                variants={genRes.variants}
                activeVariantId={activeVariant?.variant_id || null}
                onSelect={(v) => {
                  setActiveVariant(v);
                  setViewingFea(false);
                  setViewingGyroid(false);
                }}
              />

              {genRes.global_alerts.map((a, i) => (
                <div key={i} className={`alert-item ${a.level}`}>
                  <strong>{a.code}:</strong> {a.message}
                </div>
              ))}

              {genRes.variants.map((v) => (
                <div
                  key={v.variant_id}
                  className={`variant-card ${activeVariant?.variant_id === v.variant_id ? "active" : ""}`}
                  onClick={() => {
                    setActiveVariant(v);
                    setViewingFea(false);
                    setViewingGyroid(false);
                  }}
                >
                  <div className="pareto-head">
                    <strong>{v.name}</strong>
                    <div className="badge-row">
                      {genRes.recommended_variant === v.variant_id && <span className="badge good">★ Rec.</span>}
                      <span className={`badge ${v.metrics.passes_requirements ? "good" : "bad"}`}>
                        {v.metrics.passes_requirements ? "✓ Cumple" : "✕ Revisar"}
                      </span>
                    </div>
                  </div>
                  <div className="metrics-grid">
                    <div className="metric-box">
                      <span>Masa</span>
                      <strong>{v.metrics.mass_g.toFixed(1)} g</strong>
                    </div>
                    <div className="metric-box">
                      <span>FS</span>
                      <strong className={v.metrics.passes_requirements ? "good" : "warn"}>
                        {v.metrics.safety_factor.toFixed(1)}×
                      </strong>
                    </div>
                    <div className="metric-box">
                      <span>Deflex.</span>
                      <strong>{v.metrics.tip_deflection_mm.toFixed(2)} mm</strong>
                    </div>
                  </div>
                  <div className="dl-row" onClick={(e) => e.stopPropagation()}>
                    <a className="dl-link" href={v.step_url} download>.STEP</a>
                    <a className="dl-link" href={v.stl_url} download>.STL</a>
                    {v.threemf_url && (
                      <a className="dl-link" href={v.threemf_url} download>.3MF</a>
                    )}
                  </div>
                  <div className="assign-row" onClick={(e) => e.stopPropagation()}>
                    {state.family === "links" && (
                      <>
                        <button
                          type="button"
                          className="btn-assign"
                          onClick={() =>
                            assignToArmSlot(
                              "upper_arm",
                              state.link,
                              v.variant_id,
                              `${v.name} · ${state.link.length_mm}mm`
                            )
                          }
                        >
                          → Brazo Sup.
                        </button>
                        <button
                          type="button"
                          className="btn-assign"
                          onClick={() =>
                            assignToArmSlot(
                              "forearm",
                              state.link,
                              v.variant_id,
                              `${v.name} · ${state.link.length_mm}mm`
                            )
                          }
                        >
                          → Antebrazo
                        </button>
                      </>
                    )}
                    {state.family === "mounts" && (
                      <button
                        type="button"
                        className="btn-assign"
                        onClick={() =>
                          assignToArmSlot(
                            "shoulder_mount",
                            state.mount,
                            v.variant_id,
                            `${v.name} · ${state.mount.actuator_type.toUpperCase()}`
                          )
                        }
                      >
                        → Montura Hombro
                      </button>
                    )}
                    {state.family === "gears" && (
                      <>
                        <button
                          type="button"
                          className="btn-assign"
                          onClick={() =>
                            assignToArmSlot(
                              "shoulder_gear",
                              state.gear,
                              v.variant_id,
                              `${v.name} · ${state.gear.teeth}T`
                            )
                          }
                        >
                          → Transm. Hombro
                        </button>
                        <button
                          type="button"
                          className="btn-assign"
                          onClick={() =>
                            assignToArmSlot(
                              "elbow_gear",
                              state.gear,
                              v.variant_id,
                              `${v.name} · ${state.gear.teeth}T`
                            )
                          }
                        >
                          → Transm. Codo
                        </button>
                      </>
                    )}
                  </div>
                </div>
              ))}
            </>
          )}

          {state.family === "arm" && (
            <>
              <ModularArmPanel
                slots={armSlots}
                slotSummaries={slotSummaries}
                result={modularArmRes}
                isBusy={isAssemblingModularArm || Boolean(busyLabel)}
                onClearSlot={clearArmSlot}
                onAutoMatch={handleAutoMatchArm}
                onAssemble={handleAssembleModularArm}
              />

              <MechatronicsPanel
                config={mechatronicsConfig}
                catalog={mechatronicsCatalog}
                mechatronics={modularArmRes?.mechatronics}
                files={modularArmRes?.files}
                onChangeConfig={setMechatronicsConfig}
              />

              {armRes && (
                <>
                  <div className="fea-card">
                    <div className="pareto-head">
                      <strong>{armRes.preset_name} ({armRes.dof}-DOF)</strong>
                      <span className="badge good">${armRes.estimated_total_cost_usd.toFixed(0)} USD</span>
                    </div>
                    <div className="metrics-grid">
                      <div className="metric-box"><span>Alcance</span><strong>{Math.round(armRes.max_reach_mm)} mm</strong></div>
                      <div className="metric-box"><span>Masa Total</span><strong>{armRes.total_arm_mass_kg.toFixed(2)} kg</strong></div>
                      <div className="metric-box"><span>Impreso 3D</span><strong>{armRes.printed_mass_g.toFixed(0)} g</strong></div>
                    </div>
                    <div className="dl-row">
                      <a className="dl-link" href={armRes.step_url} download>.STEP</a>
                      <a className="dl-link" href={armRes.stl_url} download>.STL</a>
                      <a className="dl-link" href={armRes.urdf_url} download>.URDF</a>
                      <a className="dl-link" href={armRes.mujoco_url} download>.XML</a>
                      <a className="dl-link" href={armRes.bom_csv_url} download>BOM .CSV</a>
                    </div>
                  </div>

                  <AssemblyTree
                    arm={armRes}
                    selectedNode={selectedArmNode}
                    onSelectNode={setSelectedArmNode}
                    onInspectSegmentInLinks={(lengthMm, loadN, label) => {
                      pushState((s) => ({
                        ...s,
                        family: "links",
                        link: { ...s.link, length_mm: Math.max(80, Math.min(400, lengthMm)), tip_load_n: loadN },
                      }));
                      showToast(`Editando ${label} (${lengthMm} mm, ${loadN} N) en Eslabones Robóticos.`);
                    }}
                  />
                </>
              )}
            </>
          )}
        </aside>
      </div>

      <ArmDatabaseModal
        open={showArmDbModal}
        arms={armDatabase?.arms || []}
        averageReplicabilityPct={armDatabase?.average_replicability_pct}
        onClose={() => setShowArmDbModal(false)}
        onLoadArm={handleLoadArmFromDatabase}
      />

      {toast && <div className={`toast ${toast.err ? "err" : ""}`}>{toast.msg}</div>}
    </div>
  );
}
