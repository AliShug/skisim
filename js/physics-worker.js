/*jshint esversion: 6*/

importScripts('ammo.js');

var physicsDeltaTime = 1/1500;
var physicsStepsPerUpdate = 20;
// var physicsDeltaTime = 1/500;
// var physicsStepsPerUpdate = 7;
var gravity = -9.81;

// Ammo must be loaded for any of the rest of this to make sense
Ammo().then(function(Ammo) {
  importScripts('ammo-extensions.js');
  importScripts('skiier.js');

  var mouseDragTransform = new Ammo.btTransform();
  class MouseDrag {
    constructor() {
      this.dragPoint = null;
      this.dragAnchor = null;
      this.target = null;
      this.strength = 0;
    }

    set(target, point=null, anchor=null, strength=1) {
      this.target = target;
      this.dragAnchor = anchor;
      this.point = point;
      this.strength = strength;
    }

    apply() {
      if (this.target == null) return;
      // calculate the force to apply
      // local->global position
      this.target.getMotionState().getWorldTransform(mouseDragTransform);
      var anchorPosition = mouseDragTransform.xform(this.dragAnchor);
      var offsetPosition = anchorPosition.clone();
      offsetPosition.op_sub(mouseDragTransform.getOrigin());
      var force = this.point.clone();
      force.op_sub(anchorPosition);
      force.op_mul(this.strength);
      this.target.applyForce(force, offsetPosition);
    }
  }

  // Bullet code
  var collisionConfig = null;
  var dispatcher = null;
  var overlappingPairCache = null;
  var solver = null;
  var dynamicsWorld = null;
  var bodies = {};
  var dynamicBodies = {};
  var mouseDrag = new MouseDrag();
  var skiier = null;
  var savedTerrainMesh = null;

  function resetPhysics() {
    if (dynamicsWorld !== null) {
      for (var id in dynamicBodies) {
        Ammo.destroy(dynamicBodies[id]);
      }
      Ammo.destroy(dynamicsWorld);
      Ammo.destroy(solver);
      Ammo.destroy(overlappingPairCache);
      Ammo.destroy(dispatcher);
      Ammo.destroy(collisionConfig);
    }
    collisionConfig = new Ammo.btDefaultCollisionConfiguration();
    dispatcher = new Ammo.btCollisionDispatcher(collisionConfig);
    overlappingPairCache = new Ammo.btDbvtBroadphase();
    solver = new Ammo.btSequentialImpulseConstraintSolver();
    solver.setRandSeed(0);
    dynamicsWorld = new Ammo.btDiscreteDynamicsWorld(
      dispatcher,
      overlappingPairCache,
      solver,
      collisionConfig);
    dynamicsWorld.setGravity(new Ammo.btVector3(0, gravity, 0));
    dynamicsWorld.getSolverInfo().set_m_numIterations(20);

    bodies = {};
    dynamicBodies = {};
    joints = {};
  }

  function loadTerrain(terrainMesh = null) {
    if (terrainMesh !== null) {
      savedTerrainMesh = new Ammo.btTriangleMesh();
      for (var i = 0; i < terrainMesh.faces.length; i++) {
        var face = terrainMesh.faces[i];
        var v1 = terrainMesh.vertices[face.a];
        var a = new Ammo.btVector3(v1.x, v1.y, v1.z);
        var v2 = terrainMesh.vertices[face.b];
        var b = new Ammo.btVector3(v2.x, v2.y, v2.z);
        var v3 = terrainMesh.vertices[face.c];
        var c = new Ammo.btVector3(v3.x, v3.y, v3.z);
        savedTerrainMesh.addTriangle(a, b, c, true);
      }
    }
    var groundShape = new Ammo.btBvhTriangleMeshShape(savedTerrainMesh, true);
    var groundTransform = new Ammo.btTransform();
    groundTransform.setIdentity();
    var mass = 0;
    var localInertia = new Ammo.btVector3(0,0,0);
    var myMotionState = new Ammo.btDefaultMotionState(groundTransform);
    var rbInfo = new Ammo.btRigidBodyConstructionInfo(0, myMotionState, groundShape, localInertia);
    var body = new Ammo.btRigidBody(rbInfo);
    dynamicsWorld.addRigidBody(body);
    bodies.ground = body;
  }

  function startUp(terrainMesh = null) {
    // Register the terrain's geometry with the physics engine
    resetPhysics();
    loadTerrain(terrainMesh);

    skiier = new Skiier(dynamicsWorld);
    skiier.initSkiier(defaultSkiier, true);

    // Register physics objects with the renderer
    postMessage({
      type: "scene-description",
      bodies: skiier.visibles
    });
  }

  var transform = new Ammo.btTransform();

  function readBulletObject(body, output) {
    body.getMotionState().getWorldTransform(transform);
    var origin = transform.getOrigin();
    output[0] = origin.x();
    output[1] = origin.y();
    output[2] = origin.z();
    var rotation = transform.getRotation();
    output[3] = rotation.x();
    output[4] = rotation.y();
    output[5] = rotation.z();
    output[6] = rotation.w();
  }

  function simulate(dt) {
    dynamicsWorld.stepSimulation(dt, 1, dt);
  }

  function updateView() {
    var data = {objects: {}};
    var props = [];
    var id;
    for (id in dynamicBodies) {
      props = [];
      readBulletObject(dynamicBodies[id], props);
      data.objects[id] = props;
    }
    for (id in skiier.bodies) {
      props = [];
      readBulletObject(skiier.bodies[id], props);
      data.objects[id] = props;
    }
    postMessage(data);
  }

  function mainLoop() {
    for (var i = 0; i < physicsStepsPerUpdate; i++) {
      simulate(physicsDeltaTime);
      controlUpdate(physicsDeltaTime);
    }
    updateView();
    updateReadout();
    // clearInterval(interval); // UNCOMMENT TO PAUSE ON START
  }

  function controlUpdate(dt) {
    skiier.update(dt);
    mouseDrag.apply();
  }

  function updateReadout() {
    var content = {
      l_elbow: skiier.jointControllers.l_elbow.lastState,
      r_elbow: skiier.jointControllers.r_elbow.lastState,
      l_shoulder_x: skiier.jointControllers.l_shoulder_x.lastState,
      l_shoulder_y: skiier.jointControllers.l_shoulder_y.lastState,
      r_shoulder_x: skiier.jointControllers.r_shoulder_x.lastState,
      r_shoulder_y: skiier.jointControllers.r_shoulder_y.lastState,
      neck: skiier.jointControllers.neck.lastState,
      spine: skiier.jointControllers.spine.lastState,
      hinge_x: skiier.jointControllers.l_shoulder_x.lastState,
      hinge_y: skiier.jointControllers.l_shoulder_y.lastState,
      // hinge_z: skiier.jointControllers.l_shoulder_z.lastState,
      // x_i: skiier.jointControllers.l_shoulder_x.pid.integral,
      // y_i: skiier.jointControllers.l_shoulder_y.pid.integral,
      // z_i: skiier.jointControllers.l_shoulder_z.pid.integral,
    };
    postMessage({type: 'info-readout', content});
  }

  var interval = null;

  onmessage = function(event) {
    var data = event.data;
    if (data.type === "start-up" || data.type === "reset") {
      if ("terrain" in event.data) {
        startUp(event.data.terrain);
      }
      else {
        startUp();
      }

      if (interval) clearInterval(interval);
      interval = setInterval(mainLoop, 1000/60);
    }
    else if (data.type === "control-update") {
      skiier.inputControls(data.controls);
    }
    else if (data.type === "drag-force") {
      if (data.object === null) {
        mouseDrag.set(null);
      }
      else {
        var object = skiier.bodies[data.object];
        var a = data.anchor;
        var p = data.position;
        var point = new Ammo.btVector3(p.x, p.y, p.z);
        var anchor = new Ammo.btVector3(a.x, a.y, a.z);
        mouseDrag.set(object, point, anchor, data.strength);
      }
    }
  };
});
