import { useRef, useEffect } from 'react';
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
  LoadingManager,
  MathUtils,
} from 'three';

import * as THREE from 'three';
import Stats from 'stats.js';

import URDFLoader from './urdf-loader/URDFLoader';

export default function RobotViewer() {
  const mountRef = useRef(null);
  const socketRef = useRef(null);

  // Function to update joint angles of 3D model
  const setJoints = (robot,angles) => {
    const joints_names = [
      'shoulder_pan_joint',
      'shoulder_lift_joint',
      'elbow_joint',
      'wrist_1_joint',
      'wrist_2_joint',
      'wrist_3_joint'
    ];

    joints_names.forEach((name, i) => {
    robot?.joints[name]?.setJointValue(angles[i]);
    robot.updateMatrixWorld(true);
    })
    };

  useEffect(() => {

    // Display FPS in top corner
    const stats = new Stats();
    stats.showPanel(0);

    mountRef.current.appendChild(stats.dom);

    // Scene setup
    const scene = new Scene();
    scene.background = new Color(0xb0b0b0);

    // Camera setup
    const camera = new PerspectiveCamera(50, window.innerWidth / window.innerHeight, 0.1, 100);
    camera.position.set(2, 2, 2);
    camera.lookAt(0, 0, 0);

    // Renderer setup
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

    loader.load('/robot-model/ur5/ur5.urdf', result => {
      robot = result;
    });


    manager.onLoad = () => {
      robot.traverse(child => child.castShadow = true);

      // Set inital configuration of robot after it was loaded
      const initial_angles = [0, 0, 0, 0, 0, 0];
      setJoints(robot, initial_angles);
      scene.add(robot);
      // Rotate robot to appear upright
      robot.rotation.x = MathUtils.degToRad(-90);
      

      // Connect WebSocket endpoint after robot was loaded
      socketRef.current = new WebSocket('ws://localhost:8000/joints');
      console.log("socket initialized")

      // Retrive desired angles from websocket data and update the 3D model's joint angles
      socketRef.current.onmessage = (event) => {
        const data = JSON.parse(event.data);
        const angles = [
        data.shoulder_pan_joint,
        data.shoulder_lift_joint,
        data.elbow_joint,
        data.wrist_1_joint,
        data.wrist_2_joint,
        data.wrist_3_joint
        ];
        setJoints(robot, angles);
        robot.updateMatrixWorld(true);
      };
    };

// Animate
const animate = () => {
  requestAnimationFrame(animate);
  stats.begin();
  renderer.render(scene, camera);
  stats.end();
};
    animate();

    const onResize = () => {
      const width = mountRef.current.clientWidth;
      const height = mountRef.current.clientHeight;
      renderer.setSize(width, height);
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
      socketRef.current?.close();
      scene.remove(robot);
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