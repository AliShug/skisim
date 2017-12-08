/*jshint esversion:6*/

class PhysicsDragController {
  constructor(domElement, cameraControls, strength=5.0) {
    this.mouse = new THREE.Vector2();
    this.targetObjectId = null;
    this.raycaster = new THREE.Raycaster();
    this.domElement = domElement;
    this.cameraControls = cameraControls;
    this.dragStarted = false;
    this.dragStartPosition = new THREE.Vector3(0, 0, 0);
    this.dragLocalAnchor = new THREE.Vector3(0, 0, 0);
    this.screenOffset = new THREE.Vector2();
    this.strength = strength;

    domElement.addEventListener("mousedown", e => this.onClick(e));
    document.addEventListener("mouseup", e => this.onRelease(e));
    document.addEventListener("mousemove", e => this.onMouseMove(e));

    var geometry = new THREE.SphereGeometry(0.03, 32, 32);
    var material = new THREE.MeshStandardMaterial({ color: 0xffff00 });
    this.debugObject = new THREE.Mesh(geometry, material);
    this.debugObject.castShadow = true;
    this.debugObject.receiveShadow = true;
    this.debugObject.visible = false;
    scene.add(this.debugObject);
  }

  startDrag(object, point) {
    for (var id in shapes) {
      if (shapes[id] === object) {
        this.targetObjectId = id;
      }
    }
    // console.log(`Hit ${this.targetObjectId}`);
    this.dragStartPosition.copy(point);
    var invMat = new THREE.Matrix4();
    invMat.getInverse(object.matrix);
    this.debugObject.position.copy(point);
    this.dragLocalAnchor.copy(point.applyMatrix4(invMat));
    this.dragStarted = true;
    this.debugObject.visible = true;
  }

  updateDrag() {
    if (!this.dragStarted || this.targetObjectId === null) return;
    this.raycaster.setFromCamera(this.mouse, camera);
    // ray-plane intersection
    var l = this.raycaster.ray.direction;
    var l0 = this.raycaster.ray.origin;
    var x = new THREE.Vector3();
    var y = new THREE.Vector3();
    var z = new THREE.Vector3();
    camera.matrix.extractBasis(x, y, z);
    var p0l0 = this.dragStartPosition.clone();
    p0l0.sub(l0);
    var t = p0l0.dot(z) / z.dot(l);
    var point = l0.clone();
    point.addScaledVector(l, t);
    this.debugObject.position.copy(point);
    physicsWorker.postMessage({
      type: "drag-force",
      object: this.targetObjectId,
      anchor: this.dragLocalAnchor,
      position: point,
      strength: this.strength
    });
  }

  endDrag() {
    this.dragStarted = false;
    physicsWorker.postMessage({
      type: "drag-force",
      object: null
    });
    this.debugObject.visible = false;
  }

  onClick(e) {
    if (e.shiftKey && e.button === 0) {
      var x = e.clientX;
      var y = e.clientY;
      this.cameraControls.enabled = false;
      var screen = renderer.getSize();
      this.mouse.x = (x / screen.width) * 2 - 1;
      this.mouse.y = (-y / screen.height) * 2 + 1;
      this.screenOffset.set(e.screenX - x, e.screenY -y);
      this.raycaster.setFromCamera(this.mouse, camera);
      var objects = Object.values(shapes);
      var intersects = this.raycaster.intersectObjects(objects);
      if (intersects.length > 0) {
        var intersection = intersects[0];
        this.startDrag(intersection.object, intersection.point);
      }
    }
  }

  onMouseMove(e) {
    if (this.dragStarted) {
      var x = e.screenX - this.screenOffset.x;
      var y = e.screenY - this.screenOffset.y;
      var screen = renderer.getSize();
      this.mouse.x = (x / screen.width) * 2 - 1;
      this.mouse.y = (-y / screen.height) * 2 + 1;
    }
  }

  onRelease(e) {
    if (this.dragStarted) {
      this.endDrag();
    }
    this.cameraControls.enabled = true;
  }
}

var scene = new THREE.Scene();
var camera = new THREE.PerspectiveCamera(60, window.innerWidth / window.innerHeight, 0.1, 5000);

var renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(window.devicePixelRatio);
renderer.setSize(window.innerWidth, window.innerHeight);
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
renderer.shadowMap.renderReverseSided = false;
renderer.physicallyCorrectLights = true;
renderer.toneMapping = THREE.Uncharted2ToneMapping;

document.body.appendChild(renderer.domElement);

var orbitControls = new THREE.OrbitControls(camera, renderer.domElement);
orbitControls.enableDamping = true;
orbitControls.rotateSpeed = 0.7;

var dragControls = new PhysicsDragController(renderer.domElement, orbitControls);

function onResize() {
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight);
}

window.addEventListener('resize', onResize, false);

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

camera.position.set(0.5, 7, -3);
orbitControls.target.set(0, 6, 0);
orbitControls.update();

// User interface
var controlData = {
  reset: function () {
    physicsWorker.postMessage({type: "reset"});
  },
  toggle: function () {
    physicsWorker.postMessage({type: "toggle"});
  },
  "Drag Strength": 5.0,
  l_shoulder_x: 1.0,
  l_shoulder_y: 1.0,
  l_shoulder_z: 1.0,
  l_arm: 1.0,
};
var gui = new dat.GUI();
// gui.remember(controlData);
gui.add(dragControls, 'strength', 0.0, 50.0);
gui.add(controlData, 'l_shoulder_x', -Math.PI, Math.PI);
gui.add(controlData, 'l_shoulder_y', -Math.PI, Math.PI);
gui.add(controlData, 'l_shoulder_z', -Math.PI, Math.PI);
gui.add(controlData, 'l_arm', -0.9*Math.PI, 0.9*Math.PI).onChange(function () {
  physicsWorker.postMessage({
    type: "control-update",
    controls: {l_arm: controlData.l_arm}
  });
});
gui.add(controlData, 'reset');
gui.add(controlData, 'toggle');

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

    physicsWorker.postMessage({type: "start-up", terrain: geometry});
  }
);

// Main loop
function animate() {
  requestAnimationFrame(animate);

  orbitControls.update();
  dragControls.updateDrag();

  renderer.render(scene, camera);
}

// Physics setup
// Physics objects are generated in the worker thread!
var shapes = {};
var physicsWorker = new Worker('js/physics-worker.js');
var running = false;
physicsWorker.onmessage = function(event) {
  var data = event.data;
  if (data.type == "scene-description") {
    // Sync up with the physics world
    console.log(data.bodies);
    for (var i = 0; i < data.bodies.length; i++) {
      var {id, x, y, z, w, d, h, ox, oy, oz, ow} = data.bodies[i];
      // Create unique objects, update even if not unique
      var shape = null;
      if (!(id in shapes)) {
        var geometry = new THREE.BoxGeometry(w, h, d);
        var material = new THREE.MeshStandardMaterial({ color: 0xffa0a0 });
        shape = new THREE.Mesh(geometry, material);
        scene.add(shape);
      }
      else {
        shape = shapes[id];
      }
      shape.position.set(x, y, z);
      shape.quaternion.set(ox, oy, oz, ow);
      shape.castShadow = true;
      shape.receiveShadow = true;
      shapes[id] = shape;
    }
  }
  else {
    if (data.debug) {
      console.log(data.debug);
      return;
    }
    for (var id2 in data.objects) {
      var ammoObj = data.objects[id2];
      var threeObj = shapes[id2];
      threeObj.position.set(ammoObj[0], ammoObj[1], ammoObj[2]);
      threeObj.quaternion.set(ammoObj[3], ammoObj[4], ammoObj[5], ammoObj[6]);
    }
    // Start animating when the physics thread is ready
    if (!running) {
      running = true;
      animate();
    }
  }
};
