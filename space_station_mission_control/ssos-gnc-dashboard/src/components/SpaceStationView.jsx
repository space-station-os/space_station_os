import React, { useEffect, useRef } from "react";
import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import URDFLoader from "urdf-loader";
import { OBJLoader } from "three/examples/jsm/loaders/OBJLoader.js";
import { STLLoader } from "three/examples/jsm/loaders/STLLoader.js";
import { ColladaLoader } from "three/examples/jsm/loaders/ColladaLoader.js";

export default function SpaceStationView() {
  const mountRef = useRef(null);
  const rafRef = useRef(0);

  useEffect(() => {
    const container = mountRef.current;
    if (!container) return;

 
    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x0b1224);

    const camera = new THREE.PerspectiveCamera(
      45,
      container.clientWidth / container.clientHeight,
      0.01,
      100000
    );
    camera.position.set(0, 200, 400);

    const renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(container.clientWidth, container.clientHeight);
    renderer.outputColorSpace = THREE.SRGBColorSpace;
    container.appendChild(renderer.domElement);


    const ambient = new THREE.AmbientLight(0xffffff, 0.5);
    const directional = new THREE.DirectionalLight(0xffffff, 1);
    directional.position.set(200, 400, 200);
    scene.add(ambient, directional);

    
    const starsGeometry = new THREE.BufferGeometry();
    const starPositions = new Float32Array(3000 * 3);
    for (let i = 0; i < starPositions.length; i++) {
      starPositions[i] = THREE.MathUtils.randFloatSpread(6000);
    }
    starsGeometry.setAttribute(
      "position",
      new THREE.BufferAttribute(starPositions, 3)
    );
    const starsMaterial = new THREE.PointsMaterial({ color: 0xffffff, size: 2 });
    const starField = new THREE.Points(starsGeometry, starsMaterial);
    scene.add(starField);

    
    const controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.05;
    controls.enableZoom = true;
    controls.minDistance = 1;
    controls.maxDistance = 20000;

    const loader = new URDFLoader();
    loader.packages = { iss: "/models/iss/" };

    
    loader.loadMeshCb = (path, manager, onComplete) => {
      const ext = path.split(".").pop().toLowerCase();

      try {
        if (ext === "stl") {
          const stlLoader = new STLLoader(manager);
          stlLoader.load(
            path,
            (geometry) => {
              const material = new THREE.MeshStandardMaterial({
                color: 0xffffff,
                metalness: 0.3,
                roughness: 0.7,
              });
              const mesh = new THREE.Mesh(geometry, material);
              mesh.castShadow = true;
              mesh.receiveShadow = true;
              onComplete(mesh);
            },
            undefined,
            (err) => {
              console.error(` STL load error: ${path}`, err);
              onComplete(null, err);
            }
          );
        } else if (ext === "dae") {
          const daeLoader = new ColladaLoader(manager);
          daeLoader.load(
            path,
            (collada) => {
              const dae = collada.scene || collada;
              dae.traverse((child) => {
                if (child.isMesh) {
                  child.castShadow = true;
                  child.receiveShadow = true;
                }
              });
              onComplete(dae);
            },
            undefined,
            (err) => {
              console.error(` DAE load error: ${path}`, err);
              onComplete(null, err);
            }
          );
        } else {
          // OBJ fallback
          const objLoader = new OBJLoader(manager);
          objLoader.load(
            path,
            (obj) => {
              obj.traverse((child) => {
                if (child.isMesh) {
                  child.material =
                    child.material && !child.material.transparent
                      ? child.material
                      : new THREE.MeshStandardMaterial({
                          color: 0xffffff,
                          metalness: 0.3,
                          roughness: 0.8,
                        });
                  child.castShadow = true;
                  child.receiveShadow = true;
                }
              });
              onComplete(obj);
            },
            undefined,
            (err) => {
              console.error(` OBJ load error: ${path}`, err);
              onComplete(null, err);
            }
          );
        }
      } catch (err) {
        console.error(` Mesh load failed: ${path}`, err);
        onComplete(null, err);
      }
    };

   
    const urdfUrl = "/models/iss/ISS.urdf";

    const addRobot = (robot) => {
      let meshCount = 0;
      robot.traverse((child) => {
        if (child.isMesh) meshCount++;
      });

      console.log(`🛰️ ${meshCount} meshes found in URDF`);
      scene.add(robot);

      // Auto-scale & center
      const box = new THREE.Box3().setFromObject(robot);
      const size = box.getSize(new THREE.Vector3());
      const center = box.getCenter(new THREE.Vector3());
      console.log("Bounding box:", size, "center:", center);

      if (!isFinite(size.length()) || size.length() < 0.001) {
        console.warn(" Model has no visible geometry or wrong paths.");
        return;
      }

      const scale = 500 / size.length();
      robot.scale.setScalar(scale);
      robot.position.sub(center.multiplyScalar(scale));

      const distance = size.length() * scale * 1.2;
      camera.position.set(distance, distance * 0.6, distance);
      controls.target.set(0, 0, 0);
      controls.update();

      console.log(" Visible model added, scale:", scale);
    };

    loader.load(
      urdfUrl,
      (robot) => addRobot(robot),
      undefined,
      (err) => console.error(" URDF load error:", err)
    );

   
    const animate = () => {
      controls.update();
      renderer.render(scene, camera);
      rafRef.current = requestAnimationFrame(animate);
    };
    animate();

   
    const handleResize = () => {
      const w = container.clientWidth;
      const h = container.clientHeight;
      camera.aspect = w / h;
      camera.updateProjectionMatrix();
      renderer.setSize(w, h);
    };
    window.addEventListener("resize", handleResize);

    return () => {
      cancelAnimationFrame(rafRef.current);
      window.removeEventListener("resize", handleResize);
      controls.dispose();
      renderer.dispose();
      container.removeChild(renderer.domElement);
    };
  }, []);

  
  return (
    <div
      ref={mountRef}
      style={{
        position: "absolute",
        inset: 0,
        overflow: "hidden",
        backgroundColor: "#0b1224",
      }}
    />
  );
}
