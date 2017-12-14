/*jshint esversion:6*/

var terrainFile = 'terrain_smooth_v1.json';

class PhysicsDragController {
  constructor(domElement, cameraControls, strength=250.0) {
    this.mouse = new THREE.Vector2();
    this.targetObjectId = null;
    this.raycaster = new THREE.Raycaster();
    this.domElement = domElement;
    this.cameraControls = cameraControls;
    this.dragStarted = false;
    this.dragCameraOffsetPosition = new THREE.Vector3(0, 0, 0);
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
    this.dragCameraOffsetPosition.copy(point);
    this.dragCameraOffsetPosition.sub(camera.position);
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
    var p0l0 = this.dragCameraOffsetPosition.clone();
    p0l0.add(camera.position);
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
    if (e.button === 0) {
      var x = e.clientX;
      var y = e.clientY;
      var screen = renderer.getSize();
      this.mouse.x = (x / screen.width) * 2 - 1;
      this.mouse.y = (-y / screen.height) * 2 + 1;
      this.screenOffset.set(e.screenX - x, e.screenY -y);
      this.raycaster.setFromCamera(this.mouse, camera);
      var objects = Object.values(shapes);
      var intersects = this.raycaster.intersectObjects(objects);
      if (intersects.length > 0) {
        this.cameraControls.enabled = false;
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
scene.background = new THREE.Color(0xffffff);

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
var followCamera = true;

///// User interface
var controlData = {
  reset: function () {
    physicsWorker.postMessage({type: "reset", controls: getControlData()});
  },
  "Follow-camera": function () {
    followCamera = !followCamera;
  },
  "Drag strength": 250,
  "Forward lean": 0.2,
  "Stand": 0.2,
  l_shoulder_x: 0.18,
  r_shoulder_x: 0.18,
  l_shoulder_y: 0.8,
  r_shoulder_y: 0.8,
  l_elbow: 0.1,
  r_elbow: 0.1,
  l_hip_x: 0.5,
  r_hip_x: 0.5,
  l_hip_y: 0.5,
  r_hip_y: 0.5,
  l_knee: 0.4,
  r_knee: 0.4,
  l_ankle: 0.37,
  r_ankle: 0.37,
  neck: 0.5,
  spine: 0.5,
};
var gui = new dat.GUI();
// gui.remember(controlData);
gui.add(controlData, 'Drag strength', 0.0, 1000.0).onChange(function () {
  dragControls.strength = controlData['Drag strength'];
});
gui.add(controlData, 'Forward lean', 0.0, 1.0).onChange(postControlUpdate);
gui.add(controlData, 'Stand', 0.0, 1.0).onChange(postControlUpdate);
gui.add(controlData, 'l_shoulder_x', 0.0, 1.0).onChange(postControlUpdate);
gui.add(controlData, 'l_shoulder_y', 0.0, 1.0).onChange(postControlUpdate);
gui.add(controlData, 'r_shoulder_x', 0.0, 1.0).onChange(postControlUpdate);
gui.add(controlData, 'r_shoulder_y', 0.0, 1.0).onChange(postControlUpdate);
gui.add(controlData, 'l_elbow', 0.0, 1.0).onChange(postControlUpdate);
gui.add(controlData, 'r_elbow', 0.0, 1.0).onChange(postControlUpdate);
gui.add(controlData, 'neck', 0, 1.0).onChange(postControlUpdate);
gui.add(controlData, 'spine', 0, 1.0).onChange(postControlUpdate);
gui.add(controlData, 'reset');
gui.add(controlData, 'Follow-camera');

// output gui
// <div style="position:absolute;z-index:1000;left: 0;top: 0;margin-left: 10px;padding: 5px;font-size: small;">Hello world
//</div>
var debugReadout = document.createElement("div");
document.body.appendChild(debugReadout);
debugReadout.setAttribute(
  'style',
  'position:absolute;z-index:1000;left: 0;top: 0;margin-left: 10px;padding: 5px;font-size: small;'
);

function getControlData() {
  return {
    l_elbow: controlData.l_elbow,
    r_elbow: controlData.r_elbow,
    l_shoulder_x: controlData.l_shoulder_x,
    l_shoulder_y: controlData.l_shoulder_y,
    r_shoulder_x: controlData.r_shoulder_x,
    r_shoulder_y: controlData.r_shoulder_y,
    l_hip_x: controlData.l_hip_x,
    l_hip_y: controlData.l_hip_y,
    r_hip_x: controlData.r_hip_x,
    r_hip_y: controlData.r_hip_y,
    l_knee: controlData.l_knee,
    r_knee: controlData.r_knee,
    l_ankle: controlData.l_ankle,
    r_ankle: controlData.r_ankle,
    neck: controlData.neck,
    spine: controlData.spine,
  };
}

function postControlUpdate() {
  physicsWorker.postMessage({
    type: "control-update",
    controls: getControlData(),
  });
}

// Terrain Mesh
var loader = new THREE.JSONLoader();
loader.load(
  'assets/' + terrainFile,
  function (geometry, materials) {
    var material = new THREE.MeshStandardMaterial({ color: 0xf0f0f0, roughness: 0.9 });
    var terrain = new THREE.Mesh(geometry, material);
    terrain.castShadow = true;
    terrain.receiveShadow = true;
    scene.add(terrain);

    physicsWorker.postMessage({type: "start-up", terrain: geometry, controls: getControlData()});
    postControlUpdate();
  }
);

var chestLastPos = new THREE.Vector3();
var cameraDelta = new THREE.Vector3();
function updateCamera() {
  if (followCamera) {
    cameraDelta.subVectors(shapes.chest.position, chestLastPos);
    orbitControls.target.copy(shapes.chest.position);
    camera.position.add(cameraDelta);
  }
  chestLastPos.copy(shapes.chest.position);
}

// Main loop
function animate() {
  requestAnimationFrame(animate);

  orbitControls.update();
  dragControls.updateDrag();
  updateCamera();

  renderer.render(scene, camera);
}

function syntaxHighlight(json) {
  if (typeof json != 'string') {
    json = JSON.stringify(json, undefined, 2);
  }
  json = json.replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;');
  json = json.replace(/("(\\u[a-zA-Z0-9]{4}|\\[^u]|[^\\"])*"(\s*:)?|\b(true|false|null)\b|-?\d+(?:\.\d*)?(?:[eE][+\-]?\d+)?)/g, function(match) {
    var cls = 'number';
    if (/^"/.test(match)) {
      if (/:$/.test(match)) {
        cls = 'key';
      } else {
        cls = 'string';
      }
    } else if (/true|false/.test(match)) {
      cls = 'boolean';
    } else if (/null/.test(match)) {
      cls = 'null';
    }
    return '<span class="' + cls + '">' + match + '</span>';
  });
  json = json.replace(/ /g, '&nbsp;');
  return json.replace(/\n/g, '<br/>');
}

// Physics setup
// Physics objects are generated in the worker thread!
var shapes = {};
var physicsWorker = new Worker('js/physics-worker.js');
var running = false;
physicsWorker.onmessage = function(event) {
  var data = event.data;
  if (data.type === "scene-description") {
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
  else if (data.type === "info-readout") {
    var json = syntaxHighlight(data.content);
    debugReadout.innerHTML = json;
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
