/*jshint esversion: 6*/

importScripts('ammo.js');

Ammo().then(function(Ammo) {
  // Bullet code
  var collisionConfig =null;
  var dispatcher = null;
  var overlappingPairCache = null;
  var solver = null;
  var dynamicsWorld = null;
  var bodies = {};
  var dynamicBodies = {};
  var joints = {};

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
    dynamicsWorld = new Ammo.btDiscreteDynamicsWorld(
      dispatcher,
      overlappingPairCache,
      solver,
      collisionConfig);
    dynamicsWorld.setGravity(new Ammo.btVector3(0, -9.81, 0));

    bodies = {};
    dynamicBodies = {};
    joints = {};
  }

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

  function createBox(params, transform = null) {
    var {id, x=0, y=0, z=0, w=1, d=1, h=1, yaw=0, pitch=0, roll=0} = params;
    var boxShape = new Ammo.btBoxShape(new Ammo.btVector3(w/2, h/2, d/2));
    var rotation = new Ammo.btQuaternion();
    rotation.setEulerZYX(yaw, pitch, roll);
    var translation = new Ammo.btVector3(x, y, z);
    if (transform !== null) {
      translation = translation.op_add(transform.getOrigin());
    }
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
    body.forceActivationState(4); // disable deactivation
    return {
      id, x, y, z, w, d, h,
      ox: rotation.x(), oy: rotation.y(), oz: rotation.z(), ow: rotation.w()
    };
  }

  function pinBodies(bodyA, bodyB, pivot, transform = null) {
    // var constraint = new Ammo.btFixedConstraint(bodyA, bodyB, new Ammo.btTransform(), new Ammo.btTransform());
  }

  function startUp(terrainMesh = null) {
    if (terrainMesh === null) {
      dynamicsWorld.removeRigidBody(bodies.ground);
      var ground = bodies.ground;
      resetPhysics();
      bodies.ground = ground;
      dynamicsWorld.addRigidBody(ground);
    }
    else {
      // Register the terrain's geometry with the physics engine
      resetPhysics();
      loadTerrain(terrainMesh);
    }

    var boxes = [];

    var rootTransform = new Ammo.btTransform();
    rootTransform.setIdentity();
    rootTransform.setOrigin(new Ammo.btVector3(0, 5, 0));

    // Humanish character
    var h = 1.8;
    var u = h / 7.5;
    var shoulderRadial = u * 0.9;
    var shoulderHeight = h - u * (4/3);
    var armUpperLength = u * (4/3);
    var armLowerLength = u * 1.2;
    var handLength = u * 0.8;
    var chestLength = u * 1.5;
    var chestHeight = h - u * 2;
    var headHeight = h - u / 2;
    var neckLength = (h - u) - shoulderHeight;
    boxes.push(createBox({
      id: "head", y: headHeight,
      w: u*0.8, d: u*0.8, h: u
    }, rootTransform));
    boxes.push(createBox({
      id: "chest", y: chestHeight,
      w: u*1.3, d: u*0.75, h: chestLength
    }, rootTransform));
    boxes.push(createBox({
      id: "gut", y: h-u*3.1,
      w: u*1.1, d: u*0.7, h: u
    }, rootTransform));
    // legs
    boxes.push(createBox({
      id: "l_leg_u", x: -u*0.7, y: u*2.9,
      w: u*0.6, d: u*0.6, h: u*1.75
    }, rootTransform));
    boxes.push(createBox({
      id: "l_leg_l", x: -u*0.7, y: u*1.1,
      w: u*0.5, d: u*0.5, h: u*1.75
    }, rootTransform));
    boxes.push(createBox({
      id: "l_foot", x: -u*0.7, y: u/6, z:-u/5,
      w: u*0.6, d: u, h: u/3
    }, rootTransform));
    boxes.push(createBox({
      id: "r_leg_u", x: u*0.7, y: u*2.9,
      w: u*0.6, d: u*0.6, h: u*1.75
    }, rootTransform));
    boxes.push(createBox({
      id: "r_leg_l", x: u*0.7, y: u*1.1,
      w: u*0.5, d: u*0.5, h: u*1.75
    }, rootTransform));
    boxes.push(createBox({
      id: "r_foot", x: u*0.7, y: u/6, z:-u/5,
      w: u*0.6, d: u, h: u/3
    }, rootTransform));
    // arms
    boxes.push(createBox({
      id: "l_arm_u",
      x: -shoulderRadial - armUpperLength/2,
      y: shoulderHeight,
      w: armUpperLength, d: u*0.4, h: u*0.4
    }, rootTransform));
    boxes.push(createBox({
      id: "l_arm_l",
      x: -shoulderRadial - armUpperLength - armLowerLength/2,
      y: shoulderHeight,
      w: armLowerLength, d: u*0.35, h: u*0.4
    }, rootTransform));
    boxes.push(createBox({
      id: "l_hand",
      x: -shoulderRadial - armUpperLength - armLowerLength - handLength/2,
      y: shoulderHeight,
      w: handLength, d: u*0.2, h: u*0.5
    }, rootTransform));
    boxes.push(createBox({
      id: "r_arm_u",
      x: shoulderRadial + armUpperLength/2,
      y: shoulderHeight,
      w: armUpperLength, d: u*0.4, h: u*0.4
    }, rootTransform));
    boxes.push(createBox({
      id: "r_arm_l",
      x: shoulderRadial + armUpperLength + armLowerLength/2,
      y: shoulderHeight,
      w: armLowerLength, d: u*0.35, h: u*0.4
    }, rootTransform));
    boxes.push(createBox({
      id: "r_hand",
      x: shoulderRadial + armUpperLength + armLowerLength + handLength/2,
      y: shoulderHeight,
      w: handLength, d: u*0.2, h: u*0.5
    }, rootTransform));

    // TODO: remove (testing)
    bodies.chest.setMassProps(0, new Ammo.btVector3(0,0,0));
    // Joints
    var shoulderPivotA = new Ammo.btVector3(-shoulderRadial, shoulderHeight-chestHeight, 0);
    var shoulderPivotB = new Ammo.btVector3(armUpperLength / 2, 0, 0);
    var shoulderTransformA = new Ammo.btTransform();
    shoulderTransformA.setIdentity();
    shoulderTransformA.setOrigin(shoulderPivotA);
    var shoulderTransformB = new Ammo.btTransform();
    shoulderTransformB.setIdentity();
    shoulderTransformB.setOrigin(shoulderPivotB);
    var shoulder = new Ammo.btConeTwistConstraint(
      bodies.chest, bodies.l_arm_u,
      shoulderTransformA, shoulderTransformB
    );
    //shoulder.setMotorTarget();
    dynamicsWorld.addConstraint(shoulder, true);
    joints.shoulder = shoulder;

    var elbowPivotA = new Ammo.btVector3(armLowerLength / 2, 0, 0);
    var elbowPivotB = new Ammo.btVector3(-armUpperLength / 2, 0, 0);
    var elbowAxis = new Ammo.btVector3(0, 0, 1);
    var elbow = new Ammo.btHingeConstraint(
      bodies.l_arm_l, bodies.l_arm_u,
      elbowPivotA, elbowPivotB, elbowAxis, elbowAxis
    );
    elbow.enableMotor(true);
    elbow.setMaxMotorImpulse(0.005);
    // Set motor target angle difference to reach in time dt
    elbow.setMotorTarget(Math.PI, 1);
    //elbow.setLimit(-Math.PI/4, Math.PI/4, 0.9, 0.1, 1.0);
    dynamicsWorld.addConstraint(elbow, true);
    joints.elbow = elbow;

    var spinePivotA = new Ammo.btVector3(0, -chestLength / 2, u/6);
    var spinePivotB = new Ammo.btVector3(0, u / 2, u/6);
    var spineAxis = new Ammo.btVector3(1, 0, 0);
    var spine = new Ammo.btHingeConstraint(
      bodies.chest, bodies.gut,
      spinePivotA, spinePivotB, spineAxis, spineAxis
    );
    // spine.enableMotor(true);
    // spine.setMaxMotorImpulse(0.005);
    // Set motor target angle difference to reach in time dt
    // spine.setMotorTarget(Math.PI, 1);
    // +ve = backward lean
    spine.setLimit(-Math.PI/4, Math.PI/6, 0.9, 0.1, 1.0);
    dynamicsWorld.addConstraint(spine, true);
    joints.spine = spine;

    var neckPivotA = new Ammo.btVector3(0, chestLength / 2 + neckLength / 2, u/6);
    var neckPivotB = new Ammo.btVector3(0, -u / 2 - neckLength / 2, u/6);
    var neckAxis = new Ammo.btVector3(1, 0, 0);
    var neck = new Ammo.btHingeConstraint(
      bodies.chest, bodies.head,
      neckPivotA, neckPivotB, neckAxis, neckAxis
    );
    // neck.enableMotor(true);
    // neck.setMaxMotorImpulse(0.005);
    // Set motor target angle difference to reach in time dt
    // neck.setMotorTarget(0, 1);
    // +ve = backwards lean
    neck.setLimit(-Math.PI/6, Math.PI/6, 0.9, 0.1, 1.0);
    dynamicsWorld.addConstraint(neck, true);
    joints.neck = neck;

    // pin hands to lower arms
    pinBodies(bodies.l_hand, bodies.l_arm_l,
      new Ammo.btVector3(
        -shoulderRadial - armUpperLength - armLowerLength,
        shoulderHeight, 0
      ), rootTransform
    );

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
    dynamicsWorld.stepSimulation(dt, 2, dt);
  }

  function updateView() {
    var data = {objects: {}};
    for (var id in dynamicBodies) {
      var props = [];
      readBulletObject(id, props);
      data.objects[id] = props;
    }
    postMessage(data);
  }

  function mainLoop() {
    for (var i = 0; i < 4; i++) {
      simulate(1/200);
    }
    updateView();
    // clearInterval(interval); // UNCOMMENT TO PAUSE ON START
  }

  var interval = null;

  onmessage = function(event) {
    var data = event.data;
    if (data.type == "start-up" || data.type == "reset") {
      if ("terrain" in event.data) {
        startUp(event.data.terrain);
      }
      else {
        startUp();
      }

      if (interval) clearInterval(interval);
      interval = setInterval(mainLoop, 1000/60);
    }
  };
});
