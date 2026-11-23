import * as THREE from "three";
import type { FamilyId } from "../types";

export type CadMaterialRole =
  | "carbon_fiber"
  | "anodized_base"
  | "anodized_mount"
  | "anodized_tcp"
  | "machined_gear"
  | "joint_hardware"
  | "matte_polymer"
  | "single_part_enhanced";

let cachedCarbonColorTex: THREE.CanvasTexture | null = null;
let cachedCarbonRoughTex: THREE.CanvasTexture | null = null;

/**
 * Generates a procedural 2x2 twill carbon-fiber weave texture offline via HTMLCanvasElement.
 */
function getCarbonFiberTextures(): {
  map: THREE.CanvasTexture;
  roughnessMap: THREE.CanvasTexture;
} {
  if (cachedCarbonColorTex && cachedCarbonRoughTex) {
    return { map: cachedCarbonColorTex, roughnessMap: cachedCarbonRoughTex };
  }

  const size = 128;
  const colorCanvas = document.createElement("canvas");
  colorCanvas.width = size;
  colorCanvas.height = size;
  const cCtx = colorCanvas.getContext("2d");

  const roughCanvas = document.createElement("canvas");
  roughCanvas.width = size;
  roughCanvas.height = size;
  const rCtx = roughCanvas.getContext("2d");

  if (cCtx && rCtx) {
    // Base dark carbon matrix
    cCtx.fillStyle = "#161b22";
    cCtx.fillRect(0, 0, size, size);

    rCtx.fillStyle = "#6e6e6e";
    rCtx.fillRect(0, 0, size, size);

    const cell = 16;
    for (let y = 0; y < size; y += cell) {
      for (let x = 0; x < size; x += cell) {
        const ix = x / cell;
        const iy = y / cell;
        const phase = (ix + iy) % 4;
        const isWarp = phase < 2;

        // Subtle anisotropic sheen gradient per tow
        const grad = isWarp
          ? cCtx.createLinearGradient(x, y, x + cell, y)
          : cCtx.createLinearGradient(x, y, x, y + cell);

        if (isWarp) {
          grad.addColorStop(0, "#1c222b");
          grad.addColorStop(0.5, "#333d4b");
          grad.addColorStop(1, "#181d25");
        } else {
          grad.addColorStop(0, "#12161d");
          grad.addColorStop(0.5, "#262e3a");
          grad.addColorStop(1, "#10141a");
        }

        cCtx.fillStyle = grad;
        cCtx.fillRect(x + 0.5, y + 0.5, cell - 1, cell - 1);

        // Alternating roughness for woven reflection catchlights
        rCtx.fillStyle = isWarp ? "#4c4c4c" : "#808080";
        rCtx.fillRect(x + 0.5, y + 0.5, cell - 1, cell - 1);
      }
    }
  }

  const map = new THREE.CanvasTexture(colorCanvas);
  map.wrapS = THREE.RepeatWrapping;
  map.wrapT = THREE.RepeatWrapping;
  map.repeat.set(6, 6);
  map.anisotropy = 4;
  map.colorSpace = THREE.SRGBColorSpace;
  map.needsUpdate = true;

  const roughnessMap = new THREE.CanvasTexture(roughCanvas);
  roughnessMap.wrapS = THREE.RepeatWrapping;
  roughnessMap.wrapT = THREE.RepeatWrapping;
  roughnessMap.repeat.set(6, 6);
  roughnessMap.anisotropy = 4;
  roughnessMap.needsUpdate = true;

  cachedCarbonColorTex = map;
  cachedCarbonRoughTex = roughnessMap;
  return { map, roughnessMap };
}

/**
 * Inspects a mesh and its immediate ancestors to find the semantic GLB node name.
 */
export function getSemanticNodeName(obj: THREE.Object3D): string {
  const candidates: string[] = [];
  let cur: THREE.Object3D | null = obj;
  for (let depth = 0; depth < 3 && cur; depth++) {
    if (cur.name) candidates.push(cur.name.toLowerCase());
    cur = cur.parent;
  }
  return candidates.join(" ");
}

/**
 * Resolves the PBR role from the Shared Contract semantic node names.
 */
export function detectCadMaterialRole(obj: THREE.Object3D, family: FamilyId): CadMaterialRole {
  const name = getSemanticNodeName(obj);

  if (name.includes("base_pedestal") || name.includes("pedestal")) {
    return "anodized_base";
  }
  if (name.includes("shoulder_mount") || name.includes("mount")) {
    return "anodized_mount";
  }
  if (name.includes("tcp_flange") || name.includes("flange") || name.includes("tcp")) {
    return "anodized_tcp";
  }
  if (name.includes("shoulder_gear") || name.includes("elbow_gear") || name.includes("gear")) {
    return "machined_gear";
  }
  if (name.includes("joint_hw") || name.includes("joint_housing") || name.includes("joint_")) {
    return "joint_hardware";
  }
  if (
    name.includes("upper_arm_link") ||
    name.includes("forearm_link") ||
    name.includes("wrist_tube") ||
    name.includes("upper_arm") ||
    name.includes("forearm") ||
    name.includes("wrist")
  ) {
    return "carbon_fiber";
  }

  if (family === "arm") {
    return "matte_polymer";
  }
  return "single_part_enhanced";
}

function extractBaseColor(existingMat: THREE.Material | THREE.Material[] | undefined): THREE.Color {
  const m = Array.isArray(existingMat) ? existingMat[0] : existingMat;
  if (m && "color" in m && (m as any).color instanceof THREE.Color) {
    return (m as any).color.clone();
  }
  return new THREE.Color("#64748b");
}

/**
 * Creates a calibrated industrial CAD MeshPhysicalMaterial for a mesh.
 */
export function createCadPhysicalMaterial(
  mesh: THREE.Mesh,
  family: FamilyId
): THREE.MeshPhysicalMaterial {
  const role = detectCadMaterialRole(mesh, family);
  const origColor = extractBaseColor(mesh.material);

  switch (role) {
    case "carbon_fiber": {
      const { map, roughnessMap } = getCarbonFiberTextures();
      return new THREE.MeshPhysicalMaterial({
        color: new THREE.Color("#e2e8f0"),
        map,
        roughnessMap,
        metalness: 0.2,
        roughness: 0.36,
        clearcoat: 0.75,
        clearcoatRoughness: 0.18,
        envMapIntensity: 1.25,
      });
    }

    case "anodized_base":
      return new THREE.MeshPhysicalMaterial({
        color: new THREE.Color("#334155"),
        metalness: 0.85,
        roughness: 0.28,
        clearcoat: 0.25,
        clearcoatRoughness: 0.3,
        envMapIntensity: 1.35,
      });

    case "anodized_mount":
      return new THREE.MeshPhysicalMaterial({
        color: new THREE.Color("#3b648c"),
        metalness: 0.85,
        roughness: 0.28,
        clearcoat: 0.3,
        clearcoatRoughness: 0.25,
        envMapIntensity: 1.4,
      });

    case "anodized_tcp":
      return new THREE.MeshPhysicalMaterial({
        color: new THREE.Color("#e09f24"),
        metalness: 0.86,
        roughness: 0.26,
        clearcoat: 0.35,
        clearcoatRoughness: 0.2,
        envMapIntensity: 1.45,
      });

    case "machined_gear":
      return new THREE.MeshPhysicalMaterial({
        color: new THREE.Color("#cbd5e1"),
        metalness: 0.9,
        roughness: 0.22,
        clearcoat: 0.15,
        clearcoatRoughness: 0.25,
        envMapIntensity: 1.5,
      });

    case "joint_hardware":
      return new THREE.MeshPhysicalMaterial({
        color: new THREE.Color("#475569"),
        metalness: 0.82,
        roughness: 0.3,
        clearcoat: 0.2,
        envMapIntensity: 1.3,
      });

    case "matte_polymer":
      return new THREE.MeshPhysicalMaterial({
        color: origColor,
        metalness: 0.08,
        roughness: 0.52,
        clearcoat: 0.12,
        clearcoatRoughness: 0.42,
        envMapIntensity: 0.95,
      });

    case "single_part_enhanced":
    default: {
      const isGear = family === "gears";
      const isMount = family === "mounts";
      return new THREE.MeshPhysicalMaterial({
        color: origColor,
        metalness: isGear ? 0.65 : isMount ? 0.55 : 0.35,
        roughness: isGear ? 0.28 : isMount ? 0.34 : 0.38,
        clearcoat: isGear ? 0.2 : 0.35,
        clearcoatRoughness: 0.24,
        envMapIntensity: 1.25,
      });
    }
  }
}

/**
 * Counts total triangles in a Three.js object hierarchy (used to gate EdgesGeometry at 40k tris).
 */
export function countSceneTriangles(root: THREE.Object3D): number {
  let tris = 0;
  root.traverse((obj) => {
    if (!(obj instanceof THREE.Mesh) || !obj.geometry) return;
    const geom = obj.geometry as THREE.BufferGeometry;
    if (geom.index) {
      tris += geom.index.count / 3;
    } else if (geom.attributes?.position) {
      tris += geom.attributes.position.count / 3;
    }
  });
  return Math.round(tris);
}
