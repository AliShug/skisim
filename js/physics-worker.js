/*jshint esversion: 6*/

importScripts('ammo.js');


Ammo().then(function(Ammo) {
  // Convenience
  var Vec3 = Ammo.btVector3;
  Vec3.prototype.clone = function () {
    var out = new Vec3(this.x(), this.y(), this.z());
    return out;
  };
  Vec3.prototype.prettyPrint = function () {
    var x = this.x(), y = this.y(), z = this.z();
    console.log(`${x}, ${y}, ${z}`);
  };

  // General-purpose PID control
  class PIDController {
    constructor(kp, ki, kd) {
      this.integral = 0.0;
      this.last_error = 0.0;
      this.kp = kp;
      this.ki = ki;
      this.kd = kd;
    }

    getUpdate(target, state, dt) {
      var error = target - state;
      this.integral += error * dt;
      var derivative = (error - this.last_error) / dt;
      this.last_error = error;
      return this.kp*error + this.ki*this.integral + this.kd*derivative;
    }
  }

  // Hinge joint controller
  // TODO: custom control?
  class HingeController {
    constructor (joint, target = 0.0) {
      this.target = target;
      this.joint = joint;
      this.pid = new PIDController(0.5, 0.1, 0.0);
    }

    setTarget(target) {
      this.target = target;
    }

    update(dt) {
      var state = this.joint.getHingeAngle();
      // console.log(state);
      var input = this.pid.getUpdate(this.target, state, dt);
      //this.joint.setMotorTarget(input, 0.1);
      this.joint.setMotorTarget(this.target, 0.1);
    }
  }

  // General-purpose external force
  class ExternalForce {
    constructor() {
      this.target = null;
      this.force = null;
      this.point = null;
    }

    set(target, force=null, point=null) {
      this.target = target;
      this.force = force;
      this.point = point;
    }

    apply() {
      if (this.target === null) return;
      // Figure out where to apply the force
      // inverse-then-inverse pattern because emscripten hates operators and
      // Bullet loves them
      var worldTransform = this.target.getWorldTransform();
      var worldInverse = worldTransform.inverse();
      var point = worldInverse.invXform(this.point);
      console.log([point.x(), point.y(), point.z()]);
      point.op_sub(worldTransform.getOrigin());
      this.target.applyForce(this.force, point);
    }
  }

  var mouseDragTransform = new Ammo.btTransform();
  class MouseDrag {
    constructor() {
      this.dragForce = new ExternalForce();
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
      var worldBasis = mouseDragTransform.getBasis();
      var invWorld = mouseDragTransform.inverse();
      var anchorPosition = invWorld.invXform(this.dragAnchor);
      var offsetPosition = anchorPosition.clone();
      offsetPosition.op_sub(mouseDragTransform.getOrigin());
      var force = this.point.clone();
      force.op_sub(anchorPosition);
      force.op_mul(this.strength);
      force.prettyPrint();
      this.target.applyForce(force, offsetPosition);
    }
  }

  // Bullet code
  var collisionConfig =null;
  var dispatcher = null;
  var overlappingPairCache = null;
  var solver = null;
  var dynamicsWorld = null;
  var bodies = {};
  var dynamicBodies = {};
  var joints = {};
  var jointControllers = {};
  var mouseDrag = new MouseDrag();

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
    dynamicsWorld.setGravity(new Vec3(0, -9.81, 0));

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
      var a = new Vec3(v1.x, v1.y, v1.z);
      var v2 = terrainMesh.vertices[face.b];
      var b = new Vec3(v2.x, v2.y, v2.z);
      var v3 = terrainMesh.vertices[face.c];
      var c = new Vec3(v3.x, v3.y, v3.z);
      groundShapeMesh.addTriangle(a, b, c, true);
    }
    var groundShape = new Ammo.btBvhTriangleMeshShape(groundShapeMesh, true);
    var mass = 0;
    var localInertia = new Vec3(0,0,0);
    var myMotionState = new Ammo.btDefaultMotionState(groundTransform);
    var rbInfo = new Ammo.btRigidBodyConstructionInfo(0, myMotionState, groundShape, localInertia);
    var body = new Ammo.btRigidBody(rbInfo);
    dynamicsWorld.addRigidBody(body);
    bodies.ground = body;
  }

  function createBox(params, transform = null) {
    var {id, x=0, y=0, z=0, w=1, d=1, h=1, yaw=0, pitch=0, roll=0} = params;
    var boxShape = new Ammo.btBoxShape(new Vec3(w/2, h/2, d/2));
    var rotation = new Ammo.btQuaternion();
    rotation.setEulerZYX(yaw, pitch, roll);
    var translation = new Vec3(x, y, z);
    if (transform !== null) {
      translation = translation.op_add(transform.getOrigin());
    }
    var startTransform = new Ammo.btTransform(rotation, translation);
    var mass = 1;
    var localInertia = new Vec3(0, 0, 0);
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
    if (transform !== null) {
      var invTransform = transform.inverse();
      pivot = invTransform.invXform(pivot);
      console.log([pivot.x(), pivot.y(), pivot.z()]);
    }
    var xformA = bodyA.getWorldTransform();
    var xformB = bodyB.getWorldTransform();
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
    rootTransform.setOrigin(new Vec3(0, 5, 0));

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
    bodies.chest.setMassProps(0, new Vec3(0,0,0));
    // Joints
    var shoulderPivotA = new Vec3(-shoulderRadial, shoulderHeight-chestHeight, 0);
    var shoulderPivotB = new Vec3(armUpperLength / 2, 0, 0);
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

    var elbowPivotA = new Vec3(armLowerLength / 2, 0, 0);
    var elbowPivotB = new Vec3(-armUpperLength / 2, 0, 0);
    var elbowAxis = new Vec3(0, 0, 1);
    var elbow = new Ammo.btHingeConstraint(
      bodies.l_arm_l, bodies.l_arm_u,
      elbowPivotA, elbowPivotB, elbowAxis, elbowAxis
    );
    elbow.enableMotor(true);
    elbow.setMaxMotorImpulse(0.015);
    // Set motor target angle difference to reach in time dt
    elbow.setMotorTarget(Math.PI, 1);
    //elbow.setLimit(-Math.PI/4, Math.PI/4, 0.9, 0.1, 1.0);
    dynamicsWorld.addConstraint(elbow, true);
    joints.elbow = elbow;
    jointControllers.elbow = new HingeController(elbow);

    var spinePivotA = new Vec3(0, -chestLength / 2, u/6);
    var spinePivotB = new Vec3(0, u / 2, u/6);
    var spineAxis = new Vec3(1, 0, 0);
    var spine = new Ammo.btHingeConstraint(
      bodies.chest, bodies.gut,
      spinePivotA, spinePivotB, spineAxis, spineAxis
    );
    // spine.enableMotor(true);
    // spine.setMaxMotorImpulse(0.005);
    // Set motor target angle difference to reach in time dt
    // spine.setMotorTarget(Math.PI, 1);
    // +ve = backward lean
    // spine.setLimit(-Math.PI/4, Math.PI/6, 0.9, 0.1, 1.0);
    dynamicsWorld.addConstraint(spine, true);
    joints.spine = spine;

    var neckPivotA = new Vec3(0, chestLength / 2 + neckLength / 2, u/6);
    var neckPivotB = new Vec3(0, -u / 2 - neckLength / 2, u/6);
    var neckAxis = new Vec3(1, 0, 0);
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
      new Vec3(
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
    var dt = 1/200;
    for (var i = 0; i < 4; i++) {
      simulate(dt);
      controlUpdate(dt);
    }
    updateView();
    // clearInterval(interval); // UNCOMMENT TO PAUSE ON START
  }

  function controlUpdate(dt) {
    for (var jointId in jointControllers) {
      jointControllers[jointId].update(dt);
    }

    mouseDrag.apply();
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
      jointControllers.elbow.setTarget(data.controls.l_arm);
    }
    else if (data.type === "drag-force") {
      if (data.object === null) {
        mouseDrag.set(null);
      }
      else {
        var object = bodies[data.object];
        var a = data.anchor;
        var p = data.position;
        var point = new Vec3(p.x, p.y, p.z);
        var anchor = new Vec3(a.x, a.y, a.z);
        mouseDrag.set(object, point, anchor, data.strength);
      }
    }
  };
});
