import React, { useEffect, useRef } from "react";
import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import { GLTFLoader } from "three/examples/jsm/loaders/GLTFLoader.js";
import URDFLoader from "urdf-loader";
import { OBJLoader } from "three/examples/jsm/loaders/OBJLoader.js";
import { connectToRosBridge, subscribeTopic } from "../utils/rosbridge.js";

export default function EarthWithStationView() {
  const mountRef = useRef(null);
  const stationRef = useRef(null);
  const latestPos = useRef(new THREE.Vector3(0, 0, 0));
  const orbitTrailRef = useRef(null);
  const orbitPoints = useRef([]);
  const initializedRef = useRef(false);

  useEffect(() => {
    const container = mountRef.current;
    if (!container) return;

    // --- Scene setup ---
    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x000000);

    // --- Camera ---
    const camera = new THREE.PerspectiveCamera(
      60,
      container.clientWidth / container.clientHeight,
      0.1,
      1000000
    );
    camera.position.set(0, 3000, 8000);

    // --- Renderer ---
    const renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(container.clientWidth, container.clientHeight);
    renderer.setPixelRatio(window.devicePixelRatio);
    renderer.outputColorSpace = THREE.SRGBColorSpace;
    container.appendChild(renderer.domElement);

    // --- Lights ---
    scene.add(new THREE.AmbientLight(0xffffff, 0.5));
    const sun = new THREE.DirectionalLight(0xffffff, 2);
    sun.position.set(5000, 2000, 5000);
    scene.add(sun);

    // --- Starfield background ---
    const starCount = 9000;
    const stars = new Float32Array(starCount * 3);
    for (let i = 0; i < starCount * 3; i++) stars[i] = (Math.random() - 0.5) * 200000;
    const starGeom = new THREE.BufferGeometry();
    starGeom.setAttribute("position", new THREE.BufferAttribute(stars, 3));
    const starMat = new THREE.PointsMaterial({ color: 0xffffff, size: 1.5 });
    const starField = new THREE.Points(starGeom, starMat);
    scene.add(starField);

    // --- Earth (GLTF) ---
    const gltfLoader = new GLTFLoader();
    let earth = null;
    gltfLoader.load(
      "/models/earth.glb",
      (gltf) => {
        earth = gltf.scene;
        earth.scale.set(300, 300, 300); // Adjusted scale
        earth.position.set(0, -500, 0); // Slight lift to prevent orbit intersection
        scene.add(earth);
        console.log("✅ Earth loaded");
      },
      undefined,
      (err) => console.error("❌ Earth load error:", err)
    );

    // --- ISS URDF model ---
    const urdfLoader = new URDFLoader();
    urdfLoader.workingPath = "/models/iss/";
    urdfLoader.loadMeshFunc = (path, manager, onComplete) => {
      const objLoader = new OBJLoader(manager);
      objLoader.load(path, onComplete, undefined, () => onComplete(null));
    };

    urdfLoader.load("/models/iss/space_data.urdf", (robot) => {
      robot.traverse((child) => {
        if (child.isMesh) {
          child.material = new THREE.MeshStandardMaterial({
            color: 0xcccccc,
            metalness: 0.5,
            roughness: 0.5,
          });
        }
      });
      robot.scale.setScalar(5);
      stationRef.current = robot;
      scene.add(robot);
      console.log("🛰️ Station loaded");
    });

    // --- Orbit trail ---
    const trailGeometry = new THREE.BufferGeometry();
    const trailMaterial = new THREE.LineBasicMaterial({ color: 0xff0000 });
    const trailLine = new THREE.Line(trailGeometry, trailMaterial);
    orbitTrailRef.current = trailLine;
    scene.add(trailLine);

    // --- Controls ---
    const controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.autoRotate = false; // we’ll manually rotate Earth
    controls.minDistance = 100;
    controls.maxDistance = 300000; // allow full zoom-out

    // --- Connect to ROSBridge ---
    connectToRosBridge("ws://localhost:9090");

    // --- Subscribe to /gnc/pos_eci ---
    const topic = subscribeTopic("/gnc/pos_eci", "geometry_msgs/Vector3", (msg) => {
      const scale = 1000; // Match with Earth scale
      const newPos = new THREE.Vector3(msg.x / scale, msg.z / scale, msg.y / scale);

      if (!initializedRef.current && stationRef.current) {
        stationRef.current.position.copy(newPos);
        initializedRef.current = true;
      }

      latestPos.current.copy(newPos);
      orbitPoints.current.push(newPos.clone());
      if (orbitPoints.current.length > 500) orbitPoints.current.shift();

      const positions = new Float32Array(orbitPoints.current.length * 3);
      orbitPoints.current.forEach((p, i) => {
        positions[i * 3] = p.x;
        positions[i * 3 + 1] = p.y;
        positions[i * 3 + 2] = p.z;
      });
      orbitTrailRef.current.geometry.setAttribute(
        "position",
        new THREE.BufferAttribute(positions, 3)
      );
      orbitTrailRef.current.geometry.computeBoundingSphere();
    });

    // --- Animation loop ---
    const animate = () => {
      requestAnimationFrame(animate);

      // Earth rotation
      if (earth) earth.rotation.y += 0.0003;

      // Station position
      if (stationRef.current && initializedRef.current) {
        stationRef.current.position.lerp(latestPos.current, 0.05);
        if (earth) stationRef.current.lookAt(earth.position);
      }

      controls.update();
      renderer.render(scene, camera);
    };
    animate();

    // --- Resize handler ---
    const handleResize = () => {
      camera.aspect = container.clientWidth / container.clientHeight;
      camera.updateProjectionMatrix();
      renderer.setSize(container.clientWidth, container.clientHeight);
    };
    window.addEventListener("resize", handleResize);

    // --- Cleanup ---
    return () => {
      window.removeEventListener("resize", handleResize);
      controls.dispose();
      topic.unsubscribe?.();
      container.removeChild(renderer.domElement);
    };
  }, []);

  return (
    <div
      ref={mountRef}
      style={{
        position: "absolute",
        top: 0,
        left: 0,
        width: "100%",
        height: "100%",
        overflow: "hidden",
        backgroundColor: "#000",
        cursor: "grab",
      }}
    />
  );
}
