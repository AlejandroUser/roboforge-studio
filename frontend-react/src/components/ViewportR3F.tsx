import React, { useEffect, useMemo, useRef, useState } from "react";
import { Canvas, useThree } from "@react-three/fiber";
import { Grid, OrbitControls } from "@react-three/drei";
import * as THREE from "three";
import { GLTFLoader } from "three/examples/jsm/loaders/GLTFLoader.js";
import type { FamilyId } from "../types";

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
  loadDirection: "y_inplane" | "z_outofplane";
  armAngles?: { shoulder: number; elbow: number; wrist: number };
  armReachMm?: number;
  armBaseHeightMm?: number;
  selectedArmNode?: string | null;
  onSelectArmNode?: (nodeName: string) => void;
  cameraPreset: { view: "iso" | "top" | "front" | "side"; nonce: number };
}

function LoadedModel({
  url,
  isFeaMesh,
  wireframe,
  meshOpacity,
  armAngles,
  selectedArmNode,
  onSelectArmNode,
  onBounds,
}: {
  url: string;
  isFeaMesh?: boolean;
  wireframe: boolean;
  meshOpacity: number;
  armAngles?: { shoulder: number; elbow: number; wrist: number };
  selectedArmNode?: string | null;
  onSelectArmNode?: (name: string) => void;
  onBounds?: (box: THREE.Box3) => void;
}) {
  const [sceneObj, setSceneObj] = useState<THREE.Group | null>(null);

  useEffect(() => {
    let cancelled = false;
    const loader = new GLTFLoader();
    loader.load(
      url,
      (gltf) => {
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
        root.updateMatrixWorld(true);

        root.traverse((obj: any) => {
          if (!obj.isMesh) return;
          if (isFeaMesh || obj.geometry?.attributes?.color) {
            obj.material = new THREE.MeshStandardMaterial({
              color: 0xffffff,
              vertexColors: true,
              roughness: 0.42,
              metalness: 0.12,
              side: THREE.DoubleSide,
            });
          }
        });

        const wrapper = new THREE.Group();
        wrapper.add(root);
        const finalBox = new THREE.Box3().setFromObject(wrapper);
        onBounds?.(finalBox);
        setSceneObj(wrapper);
      },
      undefined,
      (err) => console.error("GLB load error:", err)
    );
    return () => {
      cancelled = true;
    };
  }, [url, isFeaMesh]);

  // Apply wireframe & opacity live
  useEffect(() => {
    if (!sceneObj) return;
    sceneObj.traverse((obj: any) => {
      if (!obj.isMesh || !obj.material) return;
      const mats = Array.isArray(obj.material) ? obj.material : [obj.material];
      mats.forEach((m: any) => {
        m.wireframe = wireframe;
        m.transparent = meshOpacity < 0.99;
        m.opacity = meshOpacity;
        if (selectedArmNode && obj.name && obj.name.includes(selectedArmNode)) {
          m.emissive?.setHex?.(0x38bdf8);
          m.emissiveIntensity = 0.35;
        } else if (m.emissive) {
          m.emissive.setHex(0x000000);
          m.emissiveIntensity = 0;
        }
        m.needsUpdate = true;
      });
    });
  }, [sceneObj, wireframe, meshOpacity, selectedArmNode]);

  // 60 FPS real-time GPU FK for hierarchical arm_assembly.glb
  useEffect(() => {
    if (!sceneObj || !armAngles) return;
    const j1 = sceneObj.getObjectByName("joint_shoulder");
    const j2 = sceneObj.getObjectByName("joint_elbow");
    const j3 = sceneObj.getObjectByName("joint_wrist");
    if (j1 && j2 && j3) {
      j1.rotation.set(0, -THREE.MathUtils.degToRad(armAngles.shoulder), 0);
      j2.rotation.set(0, -THREE.MathUtils.degToRad(armAngles.elbow), 0);
      j3.rotation.set(0, -THREE.MathUtils.degToRad(armAngles.wrist), 0);
      sceneObj.updateMatrixWorld(true);
    }
  }, [sceneObj, armAngles?.shoulder, armAngles?.elbow, armAngles?.wrist]);

  if (!sceneObj) return null;
  return (
    <primitive
      object={sceneObj}
      onClick={(e: any) => {
        if (!onSelectArmNode) return;
        e.stopPropagation();
        const name = e.object?.name || "";
        if (name) onSelectArmNode(name);
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
        metalness: 0.25,
        roughness: 0.45,
        transparent: true,
        opacity,
        depthWrite: opacity > 0.5,
        side: THREE.DoubleSide,
      });
      root.traverse((obj: any) => {
        if (obj.isMesh) obj.material = mat;
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
}: {
  bounds: THREE.Box3 | null;
  preset: { view: "iso" | "top" | "front" | "side"; nonce: number };
  autoRotate: boolean;
}) {
  const { camera } = useThree();
  const controlsRef = useRef<any>(null);

  useEffect(() => {
    if (!bounds || bounds.isEmpty()) return;
    const sphere = new THREE.Sphere();
    bounds.getBoundingSphere(sphere);
    const r = Math.max(sphere.radius, 30);
    const target = sphere.center.clone();
    const dist = r * 2.55;

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
  }, [bounds, preset.view, preset.nonce, camera]);

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
        : new THREE.Vector3(bounds.max.x - 8, bounds.max.y + (loadDirection === "z_outofplane" ? len + 4 : 0), center.z - (loadDirection === "z_outofplane" ? 0 : len + 6));
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
      <mesh rotation={[-Math.PI / 2, 0, 0]} position={[bounds.min.x + Math.min(28, size.x * 0.2), bounds.min.y + 0.8, center.z]}>
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
  const splitOffsetZ = useMemo(() => {
    if (!bounds || bounds.isEmpty()) return 70;
    const s = new THREE.Vector3();
    bounds.getSize(s);
    return Math.max(45, s.z + 24);
  }, [bounds]);

  return (
    <Canvas
      camera={{ position: [180, 140, 210], fov: 42, near: 0.5, far: 8000 }}
      style={{ width: "100%", height: "100%", background: "radial-gradient(circle at 50% 42%, #131c2b 0%, #090d14 78%)" }}
    >
      <ambientLight intensity={0.65} />
      <hemisphereLight args={["#bae6fd", "#0f172a", 0.6]} />
      <directionalLight position={[180, 260, 160]} intensity={1.35} />
      <directionalLight position={[-160, 120, -140]} intensity={0.6} color="#7dd3fc" />

      <Grid
        args={[600, 600]}
        cellSize={10}
        cellThickness={0.5}
        cellColor="#1e293b"
        sectionSize={50}
        sectionThickness={1}
        sectionColor="#334155"
        fadeDistance={700}
        position={[0, -0.2, 0]}
      />

      {props.glbUrl && (
        <LoadedModel
          url={props.glbUrl}
          isFeaMesh={props.isFeaMesh}
          wireframe={props.wireframe}
          meshOpacity={props.meshOpacity}
          armAngles={props.family === "arm" ? props.armAngles : undefined}
          selectedArmNode={props.selectedArmNode}
          onSelectArmNode={props.onSelectArmNode}
          onBounds={setBounds}
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

      <CameraController bounds={bounds} preset={props.cameraPreset} autoRotate={props.autoRotate} />
    </Canvas>
  );
};
