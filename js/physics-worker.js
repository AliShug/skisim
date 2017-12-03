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

  var groundShape = null;

  var bodies = [];

  var groundTransform = new Ammo.btTransform();
  groundTransform.setIdentity();

  var boxShape = new Ammo.btBoxShape(new Ammo.btVector3(0.5, 0.5, 0.5));

  function startUp(terrainMesh) {
    // Register the terrain's geometry with the physics engine
    (function () {
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
      groundShape = new Ammo.btBvhTriangleMeshShape(groundShapeMesh, true);
      var mass = 0;
      var localInertia = new Ammo.btVector3(0,0,0);
      var myMotionState = new Ammo.btDefaultMotionState(groundTransform);
      var rbInfo = new Ammo.btRigidBodyConstructionInfo(0, myMotionState, groundShape, localInertia);
      var body = new Ammo.btRigidBody(rbInfo);
      dynamicsWorld.addRigidBody(body);
      bodies.push(body);
    })();

    var startTransform = new Ammo.btTransform();
    startTransform.setIdentity();
    startTransform.setOrigin(new Ammo.btVector3(4, 5, 0));
    var mass = 1;
    var localInertia = new Ammo.btVector3(0, 0, 0);
    boxShape.calculateLocalInertia(mass, localInertia);

    var myMotionState = new Ammo.btDefaultMotionState(startTransform);
    var rbInfo = new Ammo.btRigidBodyConstructionInfo(mass, myMotionState, boxShape, localInertia);
    var body = new Ammo.btRigidBody(rbInfo);

    dynamicsWorld.addRigidBody(body);
    bodies.push(body);
  }

  var transform = new Ammo.btTransform();

  function readBulletObject(i, object) {
    var body = bodies[i];
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

    data = {objects: []};
    box = [];
    readBulletObject(1, box);
    data.objects.push(box);
    postMessage(data);
  }

  var interval = null;

  onmessage = function(event) {
    startUp(event.data.terrain);
    function mainLoop() {
      simulate(1000/60);
    }

    if (interval) clearInterval(interval);
    interval = setInterval(mainLoop, 1000/60);
  };
});
