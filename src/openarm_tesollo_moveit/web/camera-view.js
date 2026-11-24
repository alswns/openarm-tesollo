import * as THREE from "three";
import URDFLoader from "urdf-loader";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";

const canvas = document.getElementById("cameraCanvas");
const message = document.getElementById("cameraStatus");
const view = document.getElementById("cameraView");
let latest = null;
document.addEventListener("ros-state", event => { latest = event.detail; });

async function initializeCamera() {
  const response = await fetch("/api/scene");
  if (!response.ok) throw new Error("카메라 모델을 읽지 못했습니다");
  const description = await response.json();
  const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  const scene = new THREE.Scene();
  scene.background = new THREE.Color("#19283a");
  scene.add(new THREE.HemisphereLight(0xffffff, 0x607080, 2.2));
  const light = new THREE.DirectionalLight(0xffffff, 2.5);
  light.position.set(2, -2, 3);
  scene.add(light);
  const ground = new THREE.Mesh(
    new THREE.PlaneGeometry(6, 6),
    new THREE.MeshStandardMaterial({ color: 0x344354, side: THREE.DoubleSide }),
  );
  ground.position.z = -0.005;
  scene.add(ground);
  const grid = new THREE.GridHelper(4, 40, 0x728397, 0x435569);
  grid.rotation.x = Math.PI / 2;
  scene.add(grid);
  const camera = new THREE.PerspectiveCamera(58, 16 / 9, 0.015, 10);
  const overview = new THREE.PerspectiveCamera(50, 16 / 9, 0.015, 10);
  overview.up.set(0, 0, 1);
  overview.position.set(1.4, -1.4, 1.1);
  const controls = new OrbitControls(overview, canvas);
  controls.target.set(0.15, 0, 0.4);
  controls.update();
  controls.enabled = false;
  const manager = new THREE.LoadingManager();
  const loaded = new Promise((resolve, reject) => {
    manager.onLoad = resolve;
    manager.onError = url => reject(new Error(`메시 로딩 실패: ${url}`));
  });
  const loader = new URDFLoader(manager);
  const robot = loader.parse(description.urdf);
  scene.add(robot);
  const opticalFrame = robot.links[description.camera_frame];
  if (!opticalFrame) throw new Error("ZED optical frame을 찾지 못했습니다");
  const cameraHousing = robot.links[description.camera_link];
  if (!cameraHousing) throw new Error("ZED camera link를 찾지 못했습니다");
  const opticalToOpenGL = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), Math.PI);
  let frames = 0;
  window.cameraViewDebug = { robot, camera, description, ready: false, frames: 0 };
  const resize = () => {
    const width = Math.max(1, canvas.clientWidth), height = Math.max(1, canvas.clientHeight);
    renderer.setSize(width, height, false);
    for (const lens of [camera, overview]) {
      lens.aspect = width / height;
      lens.updateProjectionMatrix();
    }
  };
  new ResizeObserver(resize).observe(canvas);
  resize();
  view.addEventListener("change", () => { controls.enabled = view.value === "overview"; });
  renderer.setAnimationLoop(() => {
    if (document.hidden) return;
    if (latest?.joints) robot.setJointValues(latest.joints);
    robot.updateMatrixWorld(true);
    opticalFrame.getWorldPosition(camera.position);
    opticalFrame.getWorldQuaternion(camera.quaternion);
    camera.quaternion.multiply(opticalToOpenGL);
    cameraHousing.visible = view.value === "overview";
    controls.update();
    renderer.render(scene, view.value === "overview" ? overview : camera);
    window.cameraViewDebug.frames = ++frames;
  });
  await loaded;
  window.cameraViewDebug.ready = true;
  message.textContent = `가상 카메라 · ${description.camera_frame} · 실제 센서 영상 아님`;
}

initializeCamera().catch(error => {
  message.textContent = error.message;
  message.style.color = "#ff9292";
});
