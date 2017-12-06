/*jshint esversion: 6*/

importScripts('ammo.js');

Ammo().then(function(Ammo) {
  // Bullet code
  var collisionConfig = new Ammo.btDefaultCollisionConfiguration();
  var dispatcher = new Ammo.btCollisionDispatcher(collisionConfig);
  var overlappingPairCache = new Ammo.btDbvtBroadphase();
  var solver = new Ammo.btSequentialImpulseConstraintSolver();
  var dynamicsWorld = new Ammo.btDiscreteDynamicsWorld(
    dispatcher,
    overlappingPairCache,
    solver,
    collisionConfig);
  dynamicsWorld.setGravity(new Ammo.btVector3(0, -9.81, 0));

  var bodies = {};
  var dynamicBodies = {};

  function loadTerrain(terrainMesh) {
    var groundTransform = new Ammo.btTransform();
    groundTransform.setIdentity();
    var groundShapeMesh = new Ammo.btTriangleMesh();
    for (var i = 0; i < terrainMesh.faces.length; i++) {
      var face = terrainMesh.faces[i];
      var v1 = terrainMesh.vertices[face.a];
      var a = new Ammo.btVector3(v1.x, v1.y, v1.z);
      var v2 = terrainMesh.vertices[face.b];
      var b = new Ammo.btVector3(v2.x, v2.y, v2.z);
      var v3 = terrainMesh.vertices[face.c];
      var c = new Ammo.btVector3(v3.x, v3.y, v3.z);
      groundShapeMesh.addTriangle(a, b, c, true);
    }
    var groundShape = new Ammo.btBvhTriangleMeshShape(groundShapeMesh, true);
    var mass = 0;
    var localInertia = new Ammo.btVector3(0,0,0);
    var myMotionState = new Ammo.btDefaultMotionState(groundTransform);
    var rbInfo = new Ammo.btRigidBodyConstructionInfo(0, myMotionState, groundShape, localInertia);
    var body = new Ammo.btRigidBody(rbInfo);
    dynamicsWorld.addRigidBody(body);
    bodies.ground = body;
  }

  function createBox(params) {
    var {id, x, y, z, w = 1, d = 1, h = 1, yaw = 0, pitch = 0, roll = 0} = params;
    var boxShape = new Ammo.btBoxShape(new Ammo.btVector3(w/2, h/2, d/2));
    var rotation = new Ammo.btQuaternion();
    rotation.setEulerZYX(yaw, pitch, roll);
    var translation = new Ammo.btVector3(x, y, z);
    var startTransform = new Ammo.btTransform(rotation, translation);
    var mass = 1;
    var localInertia = new Ammo.btVector3(0, 0, 0);
    boxShape.calculateLocalInertia(mass, localInertia);
    var myMotionState = new Ammo.btDefaultMotionState(startTransform);
    var rbInfo = new Ammo.btRigidBodyConstructionInfo(mass, myMotionState, boxShape, localInertia);
    var body = new Ammo.btRigidBody(rbInfo);
    dynamicsWorld.addRigidBody(body);
    bodies[id] = body;
    dynamicBodies[id] = body;

    return {
      id, x, y, z, w, d, h,
      ox: rotation.x(), oy: rotation.y(), oz: rotation.z(), ow: rotation.w()
    };
  }

  function startUp(terrainMesh) {
    // Register the terrain's geometry with the physics engine
    loadTerrain(terrainMesh);

    var boxes = [];
    for (var i = 0; i < 10; i++) {
      boxes.push(createBox({id: i,
        x: 0, y: 5 + i, z: 4,
        w: 2, d: 2, h: 0.5,
        roll: 0.5
      }));
    }

    // Register physics objects with the renderer
    postMessage({
      type: "scene-description",
      bodies: boxes
    });
  }

  var transform = new Ammo.btTransform();

  function readBulletObject(id, object) {
    var body = bodies[id];
    body.getMotionState().getWorldTransform(transform);
    var origin = transform.getOrigin();
    object[0] = origin.x();
    object[1] = origin.y();
    object[2] = origin.z();
    var rotation = transform.getRotation();
    object[3] = rotation.x();
    object[4] = rotation.y();
    object[5] = rotation.z();
    object[6] = rotation.w();
  }

  function simulate(dt) {
    dt = dt || 1;
    dynamicsWorld.stepSimulation(dt, 2);

    var data = {objects: {}};
    for (var id in dynamicBodies) {
      var props = [];
      readBulletObject(id, props);
      data.objects[id] = props;
    }
    postMessage(data);
  }

  function mainLoop() {
    simulate(1000/60);
  }

  var interval = null;

  onmessage = function(event) {
    var data = event.data;
    if (data.type == "start-up") {
      startUp(event.data.terrain);

      if (interval) clearInterval(interval);
      interval = setInterval(mainLoop, 1000/60);
    }
  };
});
