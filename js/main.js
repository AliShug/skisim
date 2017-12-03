var scene = new THREE.Scene();
var camera = new THREE.PerspectiveCamera(60, window.innerWidth / window.innerHeight, 0.1, 5000);
var orbitControls = new THREE.OrbitControls(camera);
orbitControls.enableDamping = true;
orbitControls.rotateSpeed = 0.7;

var renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setSize(window.innerWidth, window.innerHeight);
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
renderer.shadowMap.renderReverseSided = false;
renderer.physicallyCorrectLights = true;
renderer.toneMapping = THREE.Uncharted2ToneMapping;

document.body.appendChild(renderer.domElement);

function onResize() {
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight);
}

window.addEventListener('resize', onResize, false);

var geometry = new THREE.BoxGeometry(1, 1, 1);
var material = new THREE.MeshStandardMaterial({ color: 0xffa0a0 });
var ambient = new THREE.AmbientLight(0xffffff, 1.2);
var sun = new THREE.DirectionalLight(0xffffff, 5.0);
sun.position.set(40,35,-45);
sun.castShadow = true;
var shadowMapRadius = 50;
sun.shadow.camera.left = -shadowMapRadius;
sun.shadow.camera.right = shadowMapRadius;
sun.shadow.camera.top = shadowMapRadius;
sun.shadow.camera.bottom = -shadowMapRadius;
sun.shadow.camera.far = 150;
sun.shadow.camera.near = 20;
sun.shadow.mapSize.width = 8192;
sun.shadow.mapSize.height = 8192;
sun.shadow.radius = 1;
sun.shadow.bias = -0.001;

scene.add(sun);
scene.add(ambient);

var testLight = new THREE.PointLight(0x3030ff, 8.0, 10, 2);
testLight.position.set(0,2,4);
scene.add(testLight);

camera.position.set(20, 20, 20);
orbitControls.update();

var cube = new THREE.Mesh(geometry, material);
cube.position.set(0, 6, 0);
cube.castShadow = true;
cube.receiveShadow = true;
scene.add(cube);

// Terrain Mesh
var loader = new THREE.JSONLoader();
loader.load(
  'assets/terrain.json',
  function (geometry, materials) {
    var material = new THREE.MeshStandardMaterial({ color: 0xffffff, roughness: 0.9 });
    var terrain = new THREE.Mesh(geometry, material);
    terrain.castShadow = true;
    terrain.receiveShadow = true;
    scene.add(terrain);

    physicsWorker.postMessage({terrain: geometry});
  }
);

// Main loop
function animate() {
  requestAnimationFrame(animate);

  cube.rotation.z += 0.01;
  cube.rotation.y += 0.01;
  orbitControls.update();

  renderer.render(scene, camera);
}

// Physics setup
var physicsWorker = new Worker('js/physics-worker.js');
var running = false;
physicsWorker.onmessage = function(event) {
  var data = event.data;
  if (data.debug) {
    console.log(data.debug);
    return;
  }
  if (data.objects.length != 1) return;
  var ammoObj = data.objects[0];
  var threeObj = cube;
  threeObj.position.set(ammoObj[0], ammoObj[1], ammoObj[2]);
  threeObj.quaternion.set(ammoObj[3], ammoObj[4], ammoObj[5], ammoObj[6]);
  // Start animating when the physics thread is ready
  if (!running) {
    running = true;
    animate();
  }
};
