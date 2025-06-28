import React, { useRef, useEffect } from 'react';
import {
  WebGLRenderer,
  PerspectiveCamera,
  Scene,
  Mesh,
  PlaneGeometry,
  ShadowMaterial,
  DirectionalLight,
  PCFSoftShadowMap,
  Color,
  AmbientLight,
  Box3,
  LoadingManager,
  MathUtils,
} from 'three';

import * as THREE from 'three';
import Stats from 'stats.js';


import URDFLoader from './urdf-loader/URDFLoader';

export default function RobotViewer() {
  const mountRef = useRef(null);

  useEffect(() => {
    if (!mountRef.current) return;

    const stats = new Stats();
    stats.showPanel(0);
    mountRef.current.appendChild(stats.dom);

    // Scene setup
    const scene = new Scene();
    scene.background = new Color(0xb0b0b0);

    const camera = new PerspectiveCamera(50, window.innerWidth / window.innerHeight, 0.1, 100);
    camera.position.set(1, 1, 2);
    camera.lookAt(0, 0, 0);

    const renderer = new WebGLRenderer({ antialias: true });
    renderer.outputEncoding = THREE.LinearEncoding;
    renderer.shadowMap.enabled = true;
    renderer.shadowMap.type = PCFSoftShadowMap;
    const width = mountRef.current.clientWidth;
    const height = mountRef.current.clientHeight;
    renderer.setSize(width, height);
    camera.aspect = width / height;
    camera.updateProjectionMatrix();
    mountRef.current.appendChild(renderer.domElement);

    // Lights
    const dirLight = new DirectionalLight(0xffffff, 10);
    dirLight.position.set(5, 30, 5);
    dirLight.castShadow = false;
    dirLight.shadow.mapSize.setScalar(512);
    scene.add(dirLight);

    const ambientLight = new AmbientLight(0xffffff, 0.1);
    scene.add(ambientLight);

    // Ground plane
    const ground = new Mesh(new PlaneGeometry(), new ShadowMaterial({ opacity: 0.25 }));
    ground.rotation.x = -Math.PI /2;
    ground.scale.setScalar(30);
    ground.receiveShadow = true;
    scene.add(ground);

    // URDF loader to load robot model
    const manager = new LoadingManager();
    const loader = new URDFLoader(manager);

    let robot = null;

    loader.load('/robot-model/ur5.urdf', result => {
      robot = result;
    });

    manager.onLoad = () => {
      robot.rotation.x = Math.PI / 2;
      robot.traverse(child => child.castShadow = true);

      // Set inital configuration of robot
      robot.joints['shoulder_pan_joint'].setJointValue(MathUtils.degToRad(0));
      robot.joints['shoulder_lift_joint'].setJointValue(MathUtils.degToRad(-70));
      robot.joints['elbow_joint'].setJointValue(MathUtils.degToRad(25));
      robot.joints['wrist_1_joint'].setJointValue(MathUtils.degToRad(0));
      robot.joints['wrist_2_joint'].setJointValue(MathUtils.degToRad(0));
      robot.joints['wrist_3_joint'].setJointValue(MathUtils.degToRad(0));

      robot.updateMatrixWorld(true);

      // rotate robot to appear upright
      robot.rotation.x = THREE.MathUtils.degToRad(270);

      scene.add(robot);
    };

// Animate
const animate = () => {
  requestAnimationFrame(animate);
  stats.begin();
  renderer.render(scene, camera);
  stats.end();
};
    animate();

    // Responsive
    const onResize = () => {
      const width = mountRef.current.clientWidth;
      const height = mountRef.current.clientHeight;
      renderer.setSize(width, height);
      camera.aspect = width / height;
      camera.updateProjectionMatrix();
      camera.aspect = width / height;
      camera.updateProjectionMatrix();
    };

    window.addEventListener('resize', onResize);
    onResize();

    // Clean up
    return () => {
      window.removeEventListener('resize', onResize);
      mountRef.current?.removeChild(renderer.domElement);
      renderer.dispose();
    };
  }, []);

return (
  <div
    ref={mountRef}
    style={{
        pointerEvents: 'auto',
      width: '50vw',
      height: '80vh',
      position: 'fixed',
          top: '10%',
          left: '2%',
      overflow: 'hidden',
    }}
  />
);

  
}