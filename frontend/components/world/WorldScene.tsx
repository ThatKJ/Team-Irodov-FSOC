"use client";

import { useEffect, useMemo, useRef } from "react";
import { Canvas, useFrame, useThree } from "@react-three/fiber";
import { OrbitControls } from "@react-three/drei";
import * as THREE from "three";

import { CAMERA } from "@/lib/baseline/constants";
import { useSimulation } from "@/lib/simulation/SimulationProvider";
import type { DemoSnapshot, Vec3 } from "@/lib/telemetry/types";

export type WorldView = "WORLD" | "CAMERA" | "TOP" | "SIDE";

const SCALE = 0.06;
const DEG = Math.PI / 180;

/** world frame (+X fwd, +Y right, +Z up) -> three.js scene (Y up, look down -Z) */
function toScene(p: Vec3): [number, number, number] {
  return [p.y * SCALE, p.z * SCALE, -p.x * SCALE];
}

function lineGeom(points: THREE.Vector3[]): THREE.BufferGeometry {
  return new THREE.BufferGeometry().setFromPoints(points);
}

function TrajectoryPath({ frames }: { frames: DemoSnapshot[] }) {
  const geom = useMemo(() => {
    const stride = Math.max(1, Math.floor(frames.length / 300));
    const pts: THREE.Vector3[] = [];
    for (let i = 0; i < frames.length; i += stride) {
      pts.push(new THREE.Vector3(...toScene(frames[i].target.position)));
    }
    return lineGeom(pts);
  }, [frames]);
  return (
    <primitive object={new THREE.Line(geom, new THREE.LineBasicMaterial({ color: "#3c4947" }))} />
  );
}

function CameraTerminal({ panDeg, tiltDeg }: { panDeg: number; tiltDeg: number }) {
  const yaw = useRef<THREE.Group>(null);
  const pitch = useRef<THREE.Group>(null);
  useFrame(() => {
    // world pan (about +Z) -> scene yaw about +Y ; world tilt -> scene pitch about +X
    if (yaw.current) yaw.current.rotation.y = THREE.MathUtils.lerp(yaw.current.rotation.y, -panDeg * DEG, 0.25);
    if (pitch.current) pitch.current.rotation.x = THREE.MathUtils.lerp(pitch.current.rotation.x, tiltDeg * DEG, 0.25);
  });
  return (
    <group>
      <mesh>
        <octahedronGeometry args={[0.55, 0]} />
        <meshStandardMaterial color="#252b2a" metalness={0.6} roughness={0.4} wireframe={false} />
      </mesh>
      <lineSegments>
        <edgesGeometry args={[new THREE.OctahedronGeometry(0.55, 0)]} />
        <lineBasicMaterial color="#869491" />
      </lineSegments>
      <group ref={yaw}>
        <group ref={pitch}>
          {/* boresight barrel points -Z (into scene) */}
          <mesh position={[0, 0, -0.9]} rotation={[Math.PI / 2, 0, 0]}>
            <cylinderGeometry args={[0.14, 0.2, 1.4, 12]} />
            <meshStandardMaterial color="#6feee1" emissive="#0b3b37" metalness={0.5} roughness={0.3} />
          </mesh>
        </group>
      </group>
    </group>
  );
}

function LineOfSight({ target }: { target: Vec3 }) {
  const geom = useMemo(
    () => lineGeom([new THREE.Vector3(0, 0, 0), new THREE.Vector3(...toScene(target))]),
    [target],
  );
  return <primitive object={new THREE.Line(geom, new THREE.LineBasicMaterial({ color: "#6feee1" }))} />;
}

function FovFrustum({ panDeg, tiltDeg }: { panDeg: number; tiltDeg: number }) {
  const group = useRef<THREE.Group>(null);
  useFrame(() => {
    if (!group.current) return;
    group.current.rotation.y = THREE.MathUtils.lerp(group.current.rotation.y, -panDeg * DEG, 0.25);
    group.current.rotation.x = THREE.MathUtils.lerp(group.current.rotation.x, tiltDeg * DEG, 0.25);
  });
  const geom = useMemo(() => {
    const d = 9; // frustum length in scene units
    const hw = Math.tan((CAMERA.horizontalFovDeg / 2) * DEG) * d;
    const hh = Math.tan((CAMERA.verticalFovDeg / 2) * DEG) * d;
    const o = new THREE.Vector3(0, 0, 0);
    const corners = [
      new THREE.Vector3(-hw, -hh, -d),
      new THREE.Vector3(hw, -hh, -d),
      new THREE.Vector3(hw, hh, -d),
      new THREE.Vector3(-hw, hh, -d),
    ];
    const pts: THREE.Vector3[] = [];
    corners.forEach((c) => {
      pts.push(o.clone(), c.clone());
    });
    for (let i = 0; i < 4; i++) pts.push(corners[i].clone(), corners[(i + 1) % 4].clone());
    return lineGeom(pts);
  }, []);
  return (
    <group ref={group}>
      <primitive
        object={new THREE.LineSegments(geom, new THREE.LineBasicMaterial({ color: "#8ecdff", transparent: true, opacity: 0.5 }))}
      />
    </group>
  );
}

function TargetMarker({ position, lost }: { position: Vec3; lost: boolean }) {
  const ref = useRef<THREE.Mesh>(null);
  const p = toScene(position);
  useFrame((_, dt) => {
    if (ref.current) ref.current.rotation.y += dt * 0.6;
  });
  return (
    <group position={p}>
      <mesh ref={ref}>
        <icosahedronGeometry args={[0.32, 0]} />
        <meshStandardMaterial
          color={lost ? "#ffb4ab" : "#6feee1"}
          emissive={lost ? "#5a0b0b" : "#0b3b37"}
          emissiveIntensity={1.4}
        />
      </mesh>
      <mesh>
        <ringGeometry args={[0.5, 0.54, 32]} />
        <meshBasicMaterial color={lost ? "#ffb4ab" : "#ffd2a2"} side={THREE.DoubleSide} transparent opacity={0.7} />
      </mesh>
    </group>
  );
}

function Axes() {
  const x = useMemo(() => lineGeom([new THREE.Vector3(0, 0, 0), new THREE.Vector3(0, 0, -3)]), []);
  const y = useMemo(() => lineGeom([new THREE.Vector3(0, 0, 0), new THREE.Vector3(3, 0, 0)]), []);
  const z = useMemo(() => lineGeom([new THREE.Vector3(0, 0, 0), new THREE.Vector3(0, 3, 0)]), []);
  return (
    <group>
      <primitive object={new THREE.Line(x, new THREE.LineBasicMaterial({ color: "#ffb4ab" }))} />
      <primitive object={new THREE.Line(y, new THREE.LineBasicMaterial({ color: "#6feee1" }))} />
      <primitive object={new THREE.Line(z, new THREE.LineBasicMaterial({ color: "#8ecdff" }))} />
    </group>
  );
}

const VIEW_POS: Record<WorldView, [number, number, number]> = {
  WORLD: [10, 7, 10],
  CAMERA: [0.2, 0.6, 6],
  TOP: [0.01, 16, 0.01],
  SIDE: [16, 1, 0.01],
};

function CameraRig({ view }: { view: WorldView }) {
  const { camera } = useThree();
  const target = useRef(new THREE.Vector3(...VIEW_POS[view]));
  useEffect(() => {
    target.current.set(...VIEW_POS[view]);
  }, [view]);
  useFrame(() => {
    camera.position.lerp(target.current, 0.08);
    camera.lookAt(0, 1, -4);
  });
  return null;
}

export default function WorldScene({ view }: { view: WorldView }) {
  const { current, frames } = useSimulation();
  const lost = current.trackingState === "TARGET_LOST";

  return (
    <Canvas
      camera={{ position: VIEW_POS.WORLD, fov: 42, near: 0.1, far: 400 }}
      gl={{ antialias: true }}
      dpr={[1, 1.75]}
    >
      <color attach="background" args={["#0e1514"]} />
      <fog attach="fog" args={["#0e1514", 18, 46]} />
      <ambientLight intensity={0.5} />
      <directionalLight position={[6, 10, 4]} intensity={0.9} />
      <directionalLight position={[-8, 4, -6]} intensity={0.3} color="#8ecdff" />

      <gridHelper args={[60, 60, "#3c4947", "#1b2120"]} position={[0, -0.01, 0]} />
      <Axes />
      <CameraTerminal panDeg={current.camera.panDeg} tiltDeg={current.camera.tiltDeg} />
      <FovFrustum panDeg={current.camera.panDeg} tiltDeg={current.camera.tiltDeg} />
      {!lost && <LineOfSight target={current.target.position} />}
      <TargetMarker position={current.target.position} lost={lost} />
      {frames.length > 0 && <TrajectoryPath frames={frames} />}

      <CameraRig view={view} />
      <OrbitControls
        makeDefault
        enablePan
        enableZoom
        target={[0, 1, -4]}
        minDistance={3}
        maxDistance={40}
      />
    </Canvas>
  );
}
