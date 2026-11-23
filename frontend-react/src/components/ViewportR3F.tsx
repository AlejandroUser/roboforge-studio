import React, { useEffect, useMemo, useRef, useState } from "react";
import { Canvas, useThree } from "@react-three/fiber";
import { ContactShadows, Environment, Grid, Lightformer, OrbitControls } from "@react-three/drei";
import * as THREE from "three";
import { GLTFLoader } from "three/examples/jsm/loaders/GLTFLoader.js";
import type { FamilyId, KinematicsType } from "../types";
import { countSceneTriangles, createCadPhysicalMaterial } from "../utils/cadMaterials";

const MAX_EDGE_TRIANGLES = 90_000;
const CAD_EDGE_THRESHOLD_DEG = 28;

const SEMANTIC_PART_KEYS = [
  "base_pedestal",
  "shoulder_mount",
  "shoulder_gear",
  "upper_arm_link",
  "elbow_gear",
  "forearm_link",
  "wrist_tube",
  "tcp_flange",
  "motor_j1_shoulder",
  "motor_j2_elbow",
  "motor_j3_wrist",
  "bearing_j1",
  "bearing_j2",
  "bearing_j3",
  "fasteners_kit",
  "controller_box",
  "horn_25t_j1",
  "horn_25t_j2",
  "horn_25t_j3",
  "clevis_shoulder",
  "clevis_elbow",
  "gripper_assembly",
  "gripper_base",
  "gripper_finger_left",
  "gripper_finger_right",
  "cycloidal_j1",
  "cycloidal_j2",
  "planetary_j1",
  "planetary_j2",
  "belt_scara",
  "linear_rail_z",
  "z_carriage",
  "scara_arm_1",
  "scara_arm_2",
  "z_quill",
  "upper_arm_roll_j3",
  "wrist_j4_roll",
  "wrist_j5_pitch",
  "wrist_j6_flange",
  "homing_sensor_j1",
  "gantry_frame_2020",
  "gantry_y_bridge",
  "gantry_x_carriage",
  "gantry_z_head",
  "vacuum_nozzle",
  "motor_x",
  "motor_y",
  "motor_z",
  "delta_top_frame",
  "delta_motor_1",
  "delta_motor_2",
  "delta_motor_3",
  "delta_bicep_1",
  "delta_bicep_2",
  "delta_bicep_3",
  "delta_forearms_1",
  "delta_forearms_2",
  "delta_forearms_3",
  "delta_platform",
  "delta_z_shaft",
  "continuum_actuator_base",
  "continuum_backbone",
  "continuum_spacer_disks",
  "continuum_tendons",
  "micro_forceps",
  "bevel_wrist_diff",
  "parallelogram_rod",
  "capstan_drum_j1",
  "capstan_drum_j2",
  "tendon_cables",
  "bimanual_torso",
  "stewart_base",
  "stewart_platform",
  "stewart_leg_1",
  "stewart_leg_2",
  "stewart_leg_3",
  "stewart_leg_4",
  "stewart_leg_5",
  "stewart_leg_6",
  "bellows_chamber_1",
  "bellows_chamber_2",
  "bellows_chamber_3",
  "bellows_end_cap",
  "pneumatic_manifold",
] as const;

const FALLBACK_EXPLODE_VECTORS: Record<string, [number, number, number]> = {
  base_pedestal: [0, 0, -45],
  controller_box: [-75, 0, -25],
  shoulder_mount: [0, -35, 0],
  clevis_shoulder: [0, 0, 20],
  shoulder_gear: [0, -60, 15],
  cycloidal_j1: [0, -65, 18],
  planetary_j1: [0, -62, 16],
  motor_j1_shoulder: [0, -90, 10],
  bearing_j1: [0, -45, 25],
  horn_25t_j1: [0, -72, 15],
  homing_sensor_j1: [-35, -35, 10],
  upper_arm_link: [35, 0, 35],
  upper_arm_roll_j3: [48, 0, 42],
  clevis_elbow: [55, 0, 48],
  elbow_gear: [55, 45, 50],
  cycloidal_j2: [55, 55, 52],
  planetary_j2: [55, 52, 50],
  motor_j2_elbow: [55, 80, 50],
  bearing_j2: [55, 30, 50],
  horn_25t_j2: [55, 62, 50],
  forearm_link: [90, 0, 65],
  wrist_tube: [130, 0, 80],
  wrist_j4_roll: [122, 0, 76],
  wrist_j5_pitch: [142, 0, 84],
  wrist_j6_flange: [160, 0, 92],
  motor_j3_wrist: [130, 65, 80],
  bearing_j3: [130, 30, 80],
  horn_25t_j3: [130, 48, 80],
  tcp_flange: [165, 0, 95],
  gripper_assembly: [185, 0, 100],
  gripper_base: [175, 0, 95],
  gripper_finger_left: [195, -24, 100],
  gripper_finger_right: [195, 24, 100],
  linear_rail_z: [0, -45, 20],
  z_carriage: [25, 0, 35],
  scara_arm_1: [55, 0, 45],
  belt_scara: [75, 0, 62],
  scara_arm_2: [110, 0, 55],
  z_quill: [145, 0, -25],
  fasteners_kit: [0, 75, 15],
  gantry_frame_2020: [0, 0, -30],
  gantry_y_bridge: [0, 0, 35],
  gantry_x_carriage: [0, -35, 55],
  gantry_z_head: [0, -55, 70],
  vacuum_nozzle: [0, 0, -55],
  motor_x: [-60, 0, 40],
  motor_y: [0, 65, 15],
  motor_z: [0, -55, 110],
  delta_top_frame: [0, 0, 55],
  delta_motor_1: [55, 0, 65],
  delta_motor_2: [-28, 48, 65],
  delta_motor_3: [-28, -48, 65],
  delta_bicep_1: [70, 0, 20],
  delta_bicep_2: [-35, 60, 20],
  delta_bicep_3: [-35, -60, 20],
  delta_forearms_1: [85, 0, -20],
  delta_forearms_2: [-42, 72, -20],
  delta_forearms_3: [-42, -72, -20],
  delta_platform: [0, 0, -65],
  delta_z_shaft: [0, 0, -35],
  continuum_actuator_base: [0, 0, -45],
  continuum_backbone: [0, 0, 15],
  continuum_spacer_disks: [0, 35, 35],
  continuum_tendons: [0, -35, 35],
  micro_forceps: [0, 0, 75],
  bevel_wrist_diff: [152, 0, 88],
  parallelogram_rod: [42, -38, 45],
  capstan_drum_j1: [0, -58, 12],
  capstan_drum_j2: [50, -52, 46],
  tendon_cables: [40, -65, 36],
  bimanual_torso: [0, 0, -45],
  stewart_base: [0, 0, -45],
  stewart_platform: [0, 0, 75],
  stewart_leg_1: [45, 25, 15],
  stewart_leg_2: [45, -25, 15],
  stewart_leg_3: [0, 50, 15],
  stewart_leg_4: [-45, 25, 15],
  stewart_leg_5: [-45, -25, 15],
  stewart_leg_6: [0, -50, 15],
  bellows_chamber_1: [35, 0, 20],
  bellows_chamber_2: [-18, 30, 20],
  bellows_chamber_3: [-18, -30, 20],
  bellows_end_cap: [0, 0, 70],
  pneumatic_manifold: [0, 0, -45],
};

const PART_LABELS_ES: Record<string, string> = {
  base_pedestal: "Pedestal Base",
  shoulder_mount: "Montura Hombro",
  clevis_shoulder: "Horquilla Hombro",
  clevis_elbow: "Horquilla Codo",
  shoulder_gear: "Transmisión Hombro",
  cycloidal_j1: "Reductor Cicloidal J1",
  cycloidal_j2: "Reductor Cicloidal J2",
  planetary_j1: "Reductor Planetario J1",
  planetary_j2: "Reductor Planetario J2",
  upper_arm_link: "Brazo Superior",
  upper_arm_roll_j3: "Eje Redundante Roll J3 (7-DOF)",
  elbow_gear: "Transmisión Codo",
  forearm_link: "Antebrazo",
  wrist_tube: "Muñeca",
  wrist_j4_roll: "Muñeca J4 Roll",
  wrist_j5_pitch: "Muñeca J5 Pitch",
  wrist_j6_flange: "Muñeca J6 Brida",
  tcp_flange: "Brida TCP",
  motor_j1_shoulder: "Motor J1 Hombro",
  motor_j2_elbow: "Motor J2 Codo",
  motor_j3_wrist: "Motor J3 Muñeca",
  bearing_j1: "Rodamiento J1",
  bearing_j2: "Rodamiento J2",
  bearing_j3: "Rodamiento J3",
  fasteners_kit: "Kit Tornillería ISO 4762",
  controller_box: "Caja Controlador STM32 / CAN-FD",
  horn_25t_j1: "Horn Aluminio 25T J1",
  horn_25t_j2: "Horn Aluminio 25T J2",
  horn_25t_j3: "Horn Aluminio 25T J3",
  gripper_assembly: "Pinza / Garra Robótica",
  gripper_base: "Base de Pinza",
  gripper_finger_left: "Dedo Pinza Izq.",
  gripper_finger_right: "Dedo Pinza Der.",
  linear_rail_z: "Guía Lineal Prismática Z (MGN12)",
  z_carriage: "Carro Vertical Z",
  scara_arm_1: "Brazo SCARA 1 (Horizontal)",
  belt_scara: "Correa Cerrada GT2 SCARA",
  scara_arm_2: "Antebrazo SCARA 2",
  z_quill: "Husillo / Quill Z",
  homing_sensor_j1: "Sensor Inductivo Homing J1",
  gantry_frame_2020: "Estructura Pórtico Aluminio 2020",
  gantry_y_bridge: "Puente Móvil Y",
  gantry_x_carriage: "Carro Lineal X",
  gantry_z_head: "Cabezal Vertical Z",
  vacuum_nozzle: "Boquilla de Vacío (EOAT)",
  motor_x: "Motor Eje X",
  motor_y: "Motor Eje Y",
  motor_z: "Motor Eje Z",
  delta_top_frame: "Chasis Superior Delta",
  delta_motor_1: "Motor Delta 1",
  delta_motor_2: "Motor Delta 2",
  delta_motor_3: "Motor Delta 3",
  delta_bicep_1: "Brazo Superior Delta 1",
  delta_bicep_2: "Brazo Superior Delta 2",
  delta_bicep_3: "Brazo Superior Delta 3",
  delta_forearms_1: "Varillas Carbono Delta 1",
  delta_forearms_2: "Varillas Carbono Delta 2",
  delta_forearms_3: "Varillas Carbono Delta 3",
  delta_platform: "Plataforma Móvil Delta",
  delta_z_shaft: "Eje Cardán Telescópico Z",
  continuum_actuator_base: "Base de Actuación Tendones",
  continuum_backbone: "Espina Flexible Nitinol (PCC)",
  continuum_spacer_disks: "Discos Espaciadores PEEK",
  continuum_tendons: "Micro-Tendones (120°)",
  micro_forceps: "Micro-Fórceps Quirúrgico",
  bevel_wrist_diff: "Muñeca Diferencial Cónica (Bevel)",
  parallelogram_rod: "Varilla Paralelogramo 4-Barras",
  capstan_drum_j1: "Tambor Capstan J1",
  capstan_drum_j2: "Tambor Capstan J2",
  tendon_cables: "Cables Tensores Capstan",
  bimanual_torso: "Torso Dual-Arm Bimanual",
  stewart_base: "Base Hexápodo Stewart",
  stewart_platform: "Plataforma Móvil 6-DOF Stewart",
  stewart_leg_1: "Actuador Lineal Stewart 1",
  stewart_leg_2: "Actuador Lineal Stewart 2",
  stewart_leg_3: "Actuador Lineal Stewart 3",
  stewart_leg_4: "Actuador Lineal Stewart 4",
  stewart_leg_5: "Actuador Lineal Stewart 5",
  stewart_leg_6: "Actuador Lineal Stewart 6",
  bellows_chamber_1: "Fuelle Neumático TPU 1",
  bellows_chamber_2: "Fuelle Neumático TPU 2",
  bellows_chamber_3: "Fuelle Neumático TPU 3",
  bellows_end_cap: "Tapa Efectora Fuelle",
  pneumatic_manifold: "Colector Neumático / Electroválvulas",
};

function resolveSemanticKey(obj: THREE.Object3D | null): string | null {
  let cur: THREE.Object3D | null = obj;
  for (let depth = 0; depth < 4 && cur; depth++) {
    const lower = (cur.name || "").toLowerCase();
    for (const key of SEMANTIC_PART_KEYS) {
      if (lower === key || lower.includes(key)) return key;
    }
    if (lower.startsWith("joint_hw")) return cur.name;
    cur = cur.parent;
  }
  return obj?.name || null;
}

export interface ViewportR3FProps {
  glbUrl: string | null;
  secondaryGlbUrl?: string | null;
  ghostGlbUrl?: string | null;
  isFeaMesh?: boolean;
  wireframe: boolean;
  autoRotate: boolean;
  showLoads: boolean;
  meshOpacity: number;
  family: FamilyId;
  materialId?: string;
  loadDirection: "y_inplane" | "z_outofplane";
  armAngles?: {
    baseYaw?: number;
    shoulder: number;
    elbow: number;
    wrist: number;
    gripperOpenPct?: number;
    kinematicsType?: KinematicsType;
    zAxisMm?: number;
  };
  armReachMm?: number;
  armBaseHeightMm?: number;
  selectedArmNode?: string | null;
  onSelectArmNode?: (nodeName: string) => void;
  explodeVectors?: Record<string, number[]> | null;
  explodeFactor?: number;
  onExplodeFactorChange?: (factor: number) => void;
  hoveredArmPart?: string | null;
  onHoverArmPart?: (partName: string | null) => void;
  cameraPreset: { view: "iso" | "top" | "front" | "side"; nonce: number };
}

/**
 * 100% offline procedural IBL studio environment (zero remote HDRIs / presets).
 */
function StudioEnvironment() {
  return (
    <Environment resolution={256} frames={1}>
      {/* Overhead diffused softbox */}
      <Lightformer
        form="rect"
        intensity={2.4}
        color="#f8fafc"
        scale={[14, 14, 1]}
        position={[0, 10, 0]}
        rotation={[Math.PI / 2, 0, 0]}
      />
      {/* Cool cyan rim light (left-back) */}
      <Lightformer
        form="rect"
        intensity={2.1}
        color="#7dd3fc"
        scale={[10, 5, 1]}
        position={[-9, 4, -7]}
        target={[0, 0, 0]}
      />
      {/* Warm industrial key-rim (right-front) */}
      <Lightformer
        form="rect"
        intensity={1.7}
        color="#fde68a"
        scale={[8, 6, 1]}
        position={[9, 5, 6]}
        target={[0, 0, 0]}
      />
      {/* Soft neutral front-side fill for dark anodized / carbon vertical faces */}
      <Lightformer
        form="ring"
        intensity={1.4}
        color="#e2e8f0"
        scale={8}
        position={[0, 3, 10]}
        target={[0, 0, 0]}
      />
      {/* Subtle dark-slate floor bounce */}
      <Lightformer
        form="rect"
        intensity={0.45}
        color="#1e293b"
        scale={[18, 18, 1]}
        position={[0, -6, 0]}
        rotation={[-Math.PI / 2, 0, 0]}
      />
    </Environment>
  );
}

function disposeCadEdges(root: THREE.Object3D) {
  root.traverse((obj: any) => {
    if (!obj.isMesh || !obj.userData?.cadEdges) return;
    const edgeLines = obj.userData.cadEdges as THREE.LineSegments;
    obj.remove(edgeLines);
    edgeLines.geometry?.dispose?.();
    if (Array.isArray(edgeLines.material)) {
      edgeLines.material.forEach((m) => m.dispose());
    } else {
      edgeLines.material?.dispose?.();
    }
    delete obj.userData.cadEdges;
  });
}

function LoadedModel({
  url,
  family,
  materialId,
  isFeaMesh,
  wireframe,
  showEdges,
  meshOpacity,
  armAngles,
  selectedArmNode,
  hoveredArmPart,
  explodeVectors,
  explodeFactor,
  onSelectArmNode,
  onHoverArmPart,
  onBounds,
  onModelMeta,
}: {
  url: string;
  family: FamilyId;
  materialId?: string;
  isFeaMesh?: boolean;
  wireframe: boolean;
  showEdges: boolean;
  meshOpacity: number;
  armAngles?: {
    baseYaw?: number;
    shoulder: number;
    elbow: number;
    wrist: number;
    gripperOpenPct?: number;
    kinematicsType?: KinematicsType;
    zAxisMm?: number;
  };
  selectedArmNode?: string | null;
  hoveredArmPart?: string | null;
  explodeVectors?: Record<string, number[]> | null;
  explodeFactor: number;
  onSelectArmNode?: (name: string) => void;
  onHoverArmPart?: (name: string | null) => void;
  onBounds?: (box: THREE.Box3) => void;
  onModelMeta?: (meta: { triangles: number; hasVertexColors: boolean }) => void;
}) {
  const [sceneObj, setSceneObj] = useState<THREE.Group | null>(null);
  const metaRef = useRef<{ triangles: number; hasVertexColors: boolean }>({
    triangles: 0,
    hasVertexColors: false,
  });

  useEffect(() => {
    let cancelled = false;
    let loadedWrapper: THREE.Group | null = null;
    const loader = new GLTFLoader();
    loader.load(
      url,
      (gltf) => {
        if (cancelled) return;
        const root = gltf.scene;

        // In arm mode, group all mobile nodes under a vertical-axis turret pivot so J0 Base Yaw
        // rotates the entire arm at 60 FPS while base_pedestal & controller_box remain fixed.
        if (family === "arm") {
          const turretPivot = new THREE.Group();
          turretPivot.name = "__turret_yaw_pivot";
          const children = [...root.children];
          for (const child of children) {
            const lower = (child.name || "").toLowerCase();
            const isStaticBase =
              lower.includes("base_pedestal") ||
              lower.includes("controller_box") ||
              lower.includes("gantry_") ||
              lower.includes("delta_") ||
              lower.includes("continuum_") ||
              lower.includes("stewart_") ||
              lower.includes("bimanual_") ||
              lower.includes("bellows_") ||
              lower.includes("pneumatic_");
            if (!isStaticBase) {
              turretPivot.add(child);
            }
          }
          if (turretPivot.children.length > 0) {
            root.add(turretPivot);
          }
        }

        root.rotation.x = -Math.PI / 2;
        root.updateMatrixWorld(true);

        const box = new THREE.Box3().setFromObject(root);
        const center = new THREE.Vector3();
        box.getCenter(center);
        root.position.x -= center.x;
        root.position.z -= center.z;
        root.position.y -= box.min.y;
        root.updateMatrixWorld(true);

        let hasVertexColors = false;
        root.traverse((obj: any) => {
          if (obj !== root && obj.name) {
            obj.userData.basePosition = obj.position.clone();
          }
          if (!obj.isMesh) return;
          if (obj.geometry && !obj.geometry.attributes?.normal) {
            obj.geometry.computeVertexNormals();
          }
          if (isFeaMesh || obj.geometry?.attributes?.color) {
            hasVertexColors = true;
            obj.geometry?.computeVertexNormals?.();
            obj.material = new THREE.MeshBasicMaterial({
              color: 0xffffff,
              vertexColors: true,
              side: THREE.DoubleSide,
            });
          } else {
            // Apply calibrated industrial CAD physical material by semantic node role
            obj.material = createCadPhysicalMaterial(obj as THREE.Mesh, family, materialId);
          }
        });

        const triangles = countSceneTriangles(root);
        metaRef.current = { triangles, hasVertexColors };
        onModelMeta?.({ triangles, hasVertexColors });

        const wrapper = new THREE.Group();
        wrapper.add(root);
        loadedWrapper = wrapper;
        const finalBox = new THREE.Box3().setFromObject(wrapper);
        onBounds?.(finalBox);
        setSceneObj(wrapper);
      },
      undefined,
      (err) => console.error("GLB load error:", err)
    );
    return () => {
      cancelled = true;
      if (loadedWrapper) {
        disposeCadEdges(loadedWrapper);
      }
    };
  }, [url, isFeaMesh, family, materialId]);

  // Apply wireframe, opacity, selection & hover highlight live
  useEffect(() => {
    if (!sceneObj) return;
    sceneObj.traverse((obj: any) => {
      if (!obj.isMesh || !obj.material) return;
      const semantic = resolveSemanticKey(obj);
      const isSelected = Boolean(
        selectedArmNode &&
          ((obj.name && obj.name.includes(selectedArmNode)) ||
            (semantic && semantic.includes(selectedArmNode)))
      );
      const isHovered = Boolean(
        !isSelected &&
          hoveredArmPart &&
          ((obj.name && obj.name === hoveredArmPart) || semantic === hoveredArmPart)
      );
      const mats = Array.isArray(obj.material) ? obj.material : [obj.material];
      mats.forEach((m: any) => {
        m.wireframe = wireframe;
        m.transparent = meshOpacity < 0.99;
        m.opacity = meshOpacity;
        if (isSelected) {
          m.emissive?.setHex?.(0x38bdf8);
          m.emissiveIntensity = 0.38;
        } else if (isHovered) {
          m.emissive?.setHex?.(0x38bdf8);
          m.emissiveIntensity = 0.2;
        } else if (m.emissive) {
          m.emissive.setHex(0x000000);
          m.emissiveIntensity = 0;
        }
        m.needsUpdate = true;
      });
    });
  }, [sceneObj, wireframe, meshOpacity, selectedArmNode, hoveredArmPart]);

  // 60 FPS client-side Exploded View (Despiece 3D) + Gripper Open/Close (0-100%)
  useEffect(() => {
    if (!sceneObj || family !== "arm") return;
    const vecMap: Record<string, number[]> = {
      ...FALLBACK_EXPLODE_VECTORS,
      ...(explodeVectors || {}),
    };
    const gripOffsetMm = ((armAngles?.gripperOpenPct ?? 50) / 100) * 14.0;

    const displaced = new Set<THREE.Object3D>();
    sceneObj.traverse((obj: any) => {
      if (!obj.name || obj.name === "__turret_yaw_pivot" || !obj.userData?.basePosition) return;
      // Avoid double-offsetting a child whose ancestor was already exploded
      let anc: THREE.Object3D | null = obj.parent;
      while (anc) {
        if (displaced.has(anc)) return;
        anc = anc.parent;
      }

      const lower = obj.name.toLowerCase();
      let vec: number[] | undefined = vecMap[obj.name] || vecMap[lower];
      if (!vec) {
        for (const key of SEMANTIC_PART_KEYS) {
          if (lower.includes(key)) {
            vec = vecMap[key];
            break;
          }
        }
      }

      const base = obj.userData.basePosition as THREE.Vector3;
      const ex = vec && vec.length >= 3 ? vec[0] * explodeFactor : 0;
      const ey = vec && vec.length >= 3 ? vec[1] * explodeFactor : 0;
      const ez = vec && vec.length >= 3 ? vec[2] * explodeFactor : 0;

      let gripY = 0;
      if (lower.includes("gripper_finger_left")) {
        gripY = -gripOffsetMm;
      } else if (lower.includes("gripper_finger_right")) {
        gripY = gripOffsetMm;
      }

      // v5.0 & v6.0 60 FPS GPU Linear Kinematics: SCARA Z, Cartesian Gantry X/Y/Z, Delta Platform Z
      let linX = 0;
      let linY = 0;
      let linZ = 0;
      const kt = armAngles?.kinematicsType || "articulated";
      if (kt === "scara" && (lower.includes("z_quill") || lower.includes("z_carriage"))) {
        linZ = -(armAngles?.zAxisMm ?? 0);
      } else if (kt === "cartesian_gantry") {
        const xShift = (armAngles?.shoulder ?? 0) * 0.9;
        const yShift = (armAngles?.elbow ?? 0) * 0.9;
        const zShift = -(armAngles?.zAxisMm ?? 0);
        if (lower.includes("gantry_y_bridge")) {
          linY = yShift;
        } else if (lower.includes("gantry_x_carriage")) {
          linX = xShift;
          linY = yShift;
        } else if (lower.includes("gantry_z_head") || lower.includes("vacuum_nozzle") || lower.includes("motor_z")) {
          linX = xShift;
          linY = yShift;
          linZ = zShift;
        }
      } else if (kt === "delta_parallel") {
        if (
          lower.includes("delta_platform") ||
          lower.includes("delta_z_shaft") ||
          lower.includes("vacuum_nozzle")
        ) {
          linZ = -(armAngles?.zAxisMm ?? 0);
        }
      } else if (kt === "stewart_hexapod") {
        if (lower.includes("stewart_platform") || lower.includes("tcp_flange")) {
          linZ = (armAngles?.zAxisMm ?? 0) * 0.65;
        }
      }

      if (vec || gripY !== 0 || linX !== 0 || linY !== 0 || linZ !== 0) {
        obj.position.set(base.x + ex + linX, base.y + ey + gripY + linY, base.z + ez + linZ);
        displaced.add(obj);
      }
    });
    sceneObj.updateMatrixWorld(true);
  }, [
    sceneObj,
    family,
    explodeFactor,
    explodeVectors,
    armAngles?.gripperOpenPct,
    armAngles?.kinematicsType,
    armAngles?.zAxisMm,
    armAngles?.shoulder,
    armAngles?.elbow,
  ]);

  // Apply / remove technical CAD Edges (28 deg threshold), skipped for FEA or >40k triangles
  useEffect(() => {
    if (!sceneObj) return;
    disposeCadEdges(sceneObj);

    const { triangles, hasVertexColors } = metaRef.current;
    const allowEdges =
      showEdges &&
      !wireframe &&
      !isFeaMesh &&
      !hasVertexColors &&
      triangles <= MAX_EDGE_TRIANGLES;

    if (!allowEdges) return;

    sceneObj.traverse((obj: any) => {
      if (!obj.isMesh || !obj.geometry) return;
      const edgeGeom = new THREE.EdgesGeometry(obj.geometry, CAD_EDGE_THRESHOLD_DEG);
      const edgeMat = new THREE.LineBasicMaterial({
        color: 0x0f172a,
        transparent: true,
        opacity: Math.min(0.55, meshOpacity * 0.6),
        depthTest: true,
      });
      const lines = new THREE.LineSegments(edgeGeom, edgeMat);
      lines.raycast = () => {};
      obj.add(lines);
      obj.userData.cadEdges = lines;
    });

    return () => {
      disposeCadEdges(sceneObj);
    };
  }, [sceneObj, showEdges, wireframe, isFeaMesh, meshOpacity]);

  // 60 FPS real-time GPU FK for J0 Base Yaw, hierarchical articulated J1/J2/J3, and SCARA horizontal arms
  useEffect(() => {
    if (!sceneObj || !armAngles) return;
    const turret = sceneObj.getObjectByName("__turret_yaw_pivot");
    if (turret) {
      turret.rotation.set(0, 0, THREE.MathUtils.degToRad(armAngles.baseYaw || 0));
    }
    if (armAngles.kinematicsType === "scara") {
      const s1 = sceneObj.getObjectByName("scara_arm_1");
      const s2 = sceneObj.getObjectByName("scara_arm_2");
      if (s1) s1.rotation.set(0, 0, THREE.MathUtils.degToRad(armAngles.shoulder || 0));
      if (s2) s2.rotation.set(0, 0, THREE.MathUtils.degToRad(armAngles.elbow || 0));
    } else {
      const j1 = sceneObj.getObjectByName("joint_shoulder");
      const j2 = sceneObj.getObjectByName("joint_elbow");
      const j3 = sceneObj.getObjectByName("joint_wrist");
      if (j1 && j2 && j3) {
        j1.rotation.set(0, -THREE.MathUtils.degToRad(armAngles.shoulder), 0);
        j2.rotation.set(0, -THREE.MathUtils.degToRad(armAngles.elbow), 0);
        j3.rotation.set(0, -THREE.MathUtils.degToRad(armAngles.wrist), 0);
      }
    }
    sceneObj.updateMatrixWorld(true);
  }, [
    sceneObj,
    armAngles?.kinematicsType,
    armAngles?.baseYaw,
    armAngles?.shoulder,
    armAngles?.elbow,
    armAngles?.wrist,
  ]);

  if (!sceneObj) return null;
  return (
    <primitive
      object={sceneObj}
      onClick={(e: any) => {
        if (!onSelectArmNode) return;
        e.stopPropagation();
        const name = resolveSemanticKey(e.object) || e.object?.name || "";
        if (name) onSelectArmNode(name);
      }}
      onPointerOver={(e: any) => {
        if (family !== "arm" || !onHoverArmPart) return;
        e.stopPropagation();
        const name = resolveSemanticKey(e.object);
        if (name) onHoverArmPart(name);
      }}
      onPointerOut={(e: any) => {
        if (family !== "arm" || !onHoverArmPart) return;
        e.stopPropagation();
        onHoverArmPart(null);
      }}
    />
  );
}

function SecondaryGhostModel({
  url,
  offsetZ,
  colorHex,
  opacity,
}: {
  url: string;
  offsetZ: number;
  colorHex: number;
  opacity: number;
}) {
  const [sceneObj, setSceneObj] = useState<THREE.Group | null>(null);

  useEffect(() => {
    let cancelled = false;
    const loader = new GLTFLoader();
    loader.load(url, (gltf) => {
      if (cancelled) return;
      const root = gltf.scene;
      root.rotation.x = -Math.PI / 2;
      root.updateMatrixWorld(true);
      const box = new THREE.Box3().setFromObject(root);
      const center = new THREE.Vector3();
      box.getCenter(center);
      root.position.x -= center.x;
      root.position.z -= center.z;
      root.position.y -= box.min.y;
      const mat = new THREE.MeshStandardMaterial({
        color: colorHex,
        emissive: colorHex,
        emissiveIntensity: 0.22,
        metalness: 0.15,
        roughness: 0.55,
        transparent: true,
        opacity,
        depthWrite: opacity > 0.5,
        side: THREE.DoubleSide,
      });
      root.traverse((obj: any) => {
        if (!obj.isMesh) return;
        obj.geometry?.computeVertexNormals?.();
        obj.material = mat;
      });
      const wrap = new THREE.Group();
      wrap.position.z = offsetZ;
      wrap.add(root);
      setSceneObj(wrap);
    });
    return () => {
      cancelled = true;
    };
  }, [url, offsetZ, colorHex, opacity]);

  if (!sceneObj) return null;
  return <primitive object={sceneObj} />;
}

function CameraController({
  bounds,
  preset,
  autoRotate,
  family,
  explodeFactor,
}: {
  bounds: THREE.Box3 | null;
  preset: { view: "iso" | "top" | "front" | "side"; nonce: number };
  autoRotate: boolean;
  family: FamilyId;
  explodeFactor: number;
}) {
  const { camera } = useThree();
  const controlsRef = useRef<any>(null);

  useEffect(() => {
    if (!bounds || bounds.isEmpty()) return;
    const sphere = new THREE.Sphere();
    bounds.getBoundingSphere(sphere);
    const r = Math.max(sphere.radius, 30);
    const target = sphere.center.clone();
    const isExplodedArm = family === "arm" && explodeFactor > 0.01;
    if (isExplodedArm) {
      target.x += explodeFactor * r * 0.22;
    }
    const dist = r * (2.55 + (isExplodedArm ? explodeFactor * 0.85 : 0));

    let dir = new THREE.Vector3(0.72, 0.55, 0.78);
    if (preset.view === "top") dir = new THREE.Vector3(0.001, 1, 0.001);
    else if (preset.view === "front") dir = new THREE.Vector3(0, 0.1, 1);
    else if (preset.view === "side") dir = new THREE.Vector3(1, 0.12, 0);
    dir.normalize();

    camera.position.copy(target).addScaledVector(dir, dist);
    camera.near = Math.max(0.5, r * 0.01);
    camera.far = Math.max(4000, r * 30);
    camera.updateProjectionMatrix();
    if (controlsRef.current) {
      controlsRef.current.target.copy(target);
      controlsRef.current.update();
    }
  }, [bounds, preset.view, preset.nonce, family, explodeFactor, camera]);

  return <OrbitControls ref={controlsRef} makeDefault enableDamping autoRotate={autoRotate} autoRotateSpeed={1.6} />;
}

function EngineeringOverlays({
  bounds,
  family,
  loadDirection,
  armReachMm,
  armBaseHeightMm,
}: {
  bounds: THREE.Box3 | null;
  family: FamilyId;
  loadDirection: "y_inplane" | "z_outofplane";
  armReachMm: number;
  armBaseHeightMm: number;
}) {
  const arrowObj = useMemo(() => {
    if (!bounds || bounds.isEmpty()) return null;
    const size = new THREE.Vector3();
    const center = new THREE.Vector3();
    bounds.getSize(size);
    bounds.getCenter(center);

    const len = Math.max(24, Math.max(size.x, size.y) * 0.22);
    const dir =
      family === "arm" || loadDirection === "z_outofplane"
        ? new THREE.Vector3(0, -1, 0)
        : new THREE.Vector3(0, 0, 1);
    const origin =
      family === "arm"
        ? new THREE.Vector3(bounds.max.x, bounds.max.y * 0.8 + len, center.z)
        : new THREE.Vector3(
            bounds.max.x - 8,
            bounds.max.y + (loadDirection === "z_outofplane" ? len + 4 : 0),
            center.z - (loadDirection === "z_outofplane" ? 0 : len + 6)
          );
    return new THREE.ArrowHelper(dir, origin, len, 0xef4444, len * 0.32, len * 0.22);
  }, [bounds, family, loadDirection]);

  if (!bounds || bounds.isEmpty()) return null;
  const center = new THREE.Vector3();
  const size = new THREE.Vector3();
  bounds.getCenter(center);
  bounds.getSize(size);

  return (
    <group>
      {arrowObj && <primitive object={arrowObj} />}
      <mesh
        rotation={[-Math.PI / 2, 0, 0]}
        position={[bounds.min.x + Math.min(28, size.x * 0.2), bounds.min.y + 0.8, center.z]}
      >
        <ringGeometry args={[12, 16, 32]} />
        <meshBasicMaterial color="#10b981" transparent opacity={0.75} side={THREE.DoubleSide} />
      </mesh>
      {family === "arm" && (
        <mesh position={[center.x - size.x * 0.22, armBaseHeightMm, center.z]}>
          <ringGeometry args={[armReachMm - 2.5, armReachMm, 64, 1, -Math.PI * 0.25, Math.PI * 1.15]} />
          <meshBasicMaterial color="#38bdf8" transparent opacity={0.25} side={THREE.DoubleSide} depthWrite={false} />
        </mesh>
      )}
    </group>
  );
}

export const ViewportR3F: React.FC<ViewportR3FProps> = (props) => {
  const [bounds, setBounds] = useState<THREE.Box3 | null>(null);
  const [showEdges, setShowEdges] = useState<boolean>(true);
  const [internalExplode, setInternalExplode] = useState<number>(0);
  const [internalHoverPart, setInternalHoverPart] = useState<string | null>(null);
  const [modelMeta, setModelMeta] = useState<{ triangles: number; hasVertexColors: boolean }>({
    triangles: 0,
    hasVertexColors: false,
  });

  const explodeFactor = props.explodeFactor ?? internalExplode;
  const setExplodeFactor = props.onExplodeFactorChange ?? setInternalExplode;
  const hoveredPart = props.hoveredArmPart !== undefined ? props.hoveredArmPart : internalHoverPart;
  const setHoveredPart = props.onHoverArmPart ?? setInternalHoverPart;

  const { splitOffsetZ, shadowScale, shadowFar } = useMemo(() => {
    if (!bounds || bounds.isEmpty()) {
      return { splitOffsetZ: 70, shadowScale: 420, shadowFar: 260 };
    }
    const s = new THREE.Vector3();
    bounds.getSize(s);
    const maxSpan = Math.max(s.x, s.y, s.z, 120);
    return {
      splitOffsetZ: Math.max(45, s.z + 24),
      shadowScale: Math.min(Math.max(maxSpan * 2.4, 240), 1400),
      shadowFar: Math.max(s.y * 2.2, 220),
    };
  }, [bounds]);

  const edgesBlockedReason = useMemo(() => {
    if (props.isFeaMesh || modelMeta.hasVertexColors) {
      return "Aristas omitidas en malla FEA";
    }
    if (modelMeta.triangles > MAX_EDGE_TRIANGLES) {
      return `Omitido (>90k triángulos: ${modelMeta.triangles.toLocaleString("es")})`;
    }
    return "Alternar aristas técnicas CAD (umbral 28°)";
  }, [props.isFeaMesh, modelMeta]);

  const edgesDisabled =
    Boolean(props.isFeaMesh) || modelMeta.hasVertexColors || modelMeta.triangles > MAX_EDGE_TRIANGLES;

  return (
    <div style={{ position: "relative", width: "100%", height: "100%" }}>
      <Canvas
        dpr={[1, 2]}
        gl={{
          antialias: true,
          toneMapping: THREE.ACESFilmicToneMapping,
          toneMappingExposure: 1.08,
        }}
        camera={{ position: [180, 140, 210], fov: 42, near: 0.5, far: 8000 }}
        style={{
          width: "100%",
          height: "100%",
          background: "radial-gradient(circle at 50% 42%, #131c2b 0%, #090d14 78%)",
        }}
      >
        {/* 100% offline procedural IBL + studio key/fill + front fill for dark faces */}
        <StudioEnvironment />
        <ambientLight intensity={0.42} />
        <hemisphereLight args={["#e0f2fe", "#1e293b", 0.52]} />
        <directionalLight position={[180, 280, 180]} intensity={1.15} color="#ffffff" />
        <directionalLight position={[40, 95, 260]} intensity={0.55} color="#e2e8f0" />
        <directionalLight position={[-170, 120, -150]} intensity={0.48} color="#7dd3fc" />

        <ContactShadows
          key={`shadow-${props.glbUrl || "none"}-${props.family}`}
          position={[0, -0.05, 0]}
          frames={1}
          opacity={0.45}
          scale={shadowScale}
          blur={2.2}
          far={shadowFar}
          resolution={512}
          color="#020617"
        />

        <Grid
          args={[700, 700]}
          cellSize={10}
          cellThickness={0.5}
          cellColor="#1e293b"
          sectionSize={50}
          sectionThickness={1}
          sectionColor="#334155"
          fadeDistance={650}
          fadeStrength={2.2}
          position={[0, -0.2, 0]}
        />

        {props.glbUrl && (
          <LoadedModel
            url={props.glbUrl}
            family={props.family}
            materialId={props.materialId}
            isFeaMesh={props.isFeaMesh}
            wireframe={props.wireframe}
            showEdges={showEdges}
            meshOpacity={props.meshOpacity}
            armAngles={props.family === "arm" ? props.armAngles : undefined}
            selectedArmNode={props.selectedArmNode}
            hoveredArmPart={hoveredPart}
            explodeVectors={props.explodeVectors}
            explodeFactor={explodeFactor}
            onSelectArmNode={props.onSelectArmNode}
            onHoverArmPart={setHoveredPart}
            onBounds={setBounds}
            onModelMeta={setModelMeta}
          />
        )}

        {props.secondaryGlbUrl && (
          <SecondaryGhostModel url={props.secondaryGlbUrl} offsetZ={splitOffsetZ} colorHex={0x94a3b8} opacity={0.7} />
        )}

        {props.ghostGlbUrl && (
          <SecondaryGhostModel url={props.ghostGlbUrl} offsetZ={0} colorHex={0x38bdf8} opacity={0.28} />
        )}

        {props.showLoads && (
          <EngineeringOverlays
            bounds={bounds}
            family={props.family}
            loadDirection={props.loadDirection}
            armReachMm={props.armReachMm || 445}
            armBaseHeightMm={props.armBaseHeightMm || 95}
          />
        )}

        <CameraController
          bounds={bounds}
          preset={props.cameraPreset}
          autoRotate={props.autoRotate}
          family={props.family}
          explodeFactor={explodeFactor}
        />
      </Canvas>

      {/* Floating hovered/selected part label in Arm mode */}
      {props.family === "arm" && (hoveredPart || props.selectedArmNode) && (
        <div
          style={{
            position: "absolute",
            left: 14,
            bottom: 14,
            zIndex: 8,
            padding: "5px 11px",
            borderRadius: 999,
            background: "rgba(14, 19, 27, 0.86)",
            border: "1px solid rgba(56, 189, 248, 0.45)",
            backdropFilter: "blur(8px)",
            color: "#e2e8f0",
            fontSize: 11.5,
            fontWeight: 600,
            pointerEvents: "none",
          }}
        >
          <span style={{ color: "#38bdf8", marginRight: 6 }}>●</span>
          {PART_LABELS_ES[hoveredPart || props.selectedArmNode || ""] || hoveredPart || props.selectedArmNode}
        </div>
      )}

      {/* Floating CAD Studio HUD pills: Despiece 3D (Arm) + Aristas CAD */}
      <div
        style={{
          position: "absolute",
          right: 14,
          bottom: 14,
          zIndex: 8,
          display: "flex",
          alignItems: "center",
          gap: 8,
          pointerEvents: "auto",
        }}
      >
        {props.family === "arm" && (
          <div
            style={{
              display: "inline-flex",
              alignItems: "center",
              gap: 8,
              padding: "5px 12px",
              borderRadius: 999,
              border: `1px solid ${
                explodeFactor > 0 ? "rgba(56, 189, 248, 0.55)" : "rgba(148, 163, 184, 0.22)"
              }`,
              background: "rgba(14, 19, 27, 0.85)",
              backdropFilter: "blur(8px)",
              color: "#e2e8f0",
              fontSize: 11.5,
              fontWeight: 600,
            }}
          >
            <span style={{ color: explodeFactor > 0 ? "#38bdf8" : "#94a3b8" }}>Despiece 3D</span>
            <input
              type="range"
              min={0}
              max={100}
              step={1}
              value={Math.round(explodeFactor * 100)}
              onChange={(e) => setExplodeFactor(Number(e.target.value) / 100)}
              style={{ width: 96, accentColor: "#38bdf8", cursor: "pointer" }}
              title="Despiece interactivo 3D (60 FPS)"
            />
            <span style={{ fontFamily: "monospace", minWidth: 32, textAlign: "right", color: "#7dd3fc" }}>
              {Math.round(explodeFactor * 100)}%
            </span>
          </div>
        )}

        <button
          type="button"
          onClick={() => !edgesDisabled && setShowEdges((v) => !v)}
          disabled={edgesDisabled}
          title={edgesBlockedReason}
          style={{
            display: "inline-flex",
            alignItems: "center",
            gap: 6,
            padding: "6px 11px",
            borderRadius: 999,
            border: `1px solid ${
              showEdges && !edgesDisabled ? "rgba(56, 189, 248, 0.55)" : "rgba(148, 163, 184, 0.2)"
            }`,
            background:
              showEdges && !edgesDisabled ? "rgba(56, 189, 248, 0.16)" : "rgba(14, 19, 27, 0.82)",
            backdropFilter: "blur(8px)",
            color: edgesDisabled ? "#64748b" : showEdges ? "#7dd3fc" : "#cbd5e1",
            fontSize: 11.5,
            fontWeight: 600,
            cursor: edgesDisabled ? "not-allowed" : "pointer",
            opacity: edgesDisabled ? 0.65 : 1,
            transition: "all 0.15s ease",
          }}
        >
          <svg width="14" height="14" viewBox="0 0 18 18" fill="none" aria-hidden="true">
            <path d="M9 2L16 6V12L9 16L2 12V6L9 2Z" stroke="currentColor" strokeWidth="1.5" />
            <path d="M9 9L16 6M9 9L2 6M9 9V16" stroke="currentColor" strokeWidth="1.3" />
          </svg>
          <span>Aristas CAD</span>
        </button>
      </div>
    </div>
  );
};
