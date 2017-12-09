/*jshint esversion: 6*/

importScripts('ammo.js');


Ammo().then(function(Ammo) {
  // Convenience
  Ammo.btVector3.prototype.clone = function () {
    var out = new Ammo.btVector3(this.x(), this.y(), this.z());
    return out;
  };
  function vecClone(v) {
    var out = new Ammo.btVector3(v.x(), v.y(), v.z());
    return out;
  }
  Ammo.btVector3.prototype.prettyPrint = function () {
    var x = this.x(), y = this.y(), z = this.z();
    console.log(`<${x}, ${y}, ${z}>`);
  };
  Ammo.btTransform.prototype.xform = function (pt) {
    var myInverse = this.inverse();
    return myInverse.invXform(pt);
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
      this.joint.setMaxMotorImpulse(0.05*Math.min(Math.abs(state-this.target)*20, 1));
    }
  }

  // General-purpose external force
  // class ExternalForce {
  //   constructor() {
  //     this.target = null;
  //     this.force = null;
  //     this.point = null;
  //   }
  //
  //   set(target, force=null, point=null) {
  //     this.target = target;
  //     this.force = force;
  //     this.point = point;
  //   }
  //
  //   apply() {
  //     if (this.target === null) return;
  //     // Figure out where to apply the force
  //     // inverse-then-inverse pattern because emscripten hates operators and
  //     // Bullet loves them
  //     // Note: code here is broken, fix transform stuff
  //     var worldTransform = this.target.getWorldTransform();
  //     var worldInverse = worldTransform.inverse();
  //     var point = worldInverse.invXform(this.point);
  //     console.log([point.x(), point.y(), point.z()]);
  //     point.op_sub(worldTransform.getOrigin());
  //     this.target.applyForce(this.force, point);
  //   }
  // }

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
    var mass = w*d*h*1000;
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

  function createFixed(bodyA, bodyB, pivot, transform = null) {
    if (transform !== null) {
      pivot = transform.xform(pivot).clone();
      pivot.prettyPrint();
    }
    var bodyAXform = bodyA.getCenterOfMassTransform();
    var bodyBXform = bodyB.getCenterOfMassTransform();
    // Local transforms of the rigid attachment points
    var xformA = new Ammo.btTransform();
    xformA.setIdentity();
    var xformB = new Ammo.btTransform();
    xformB.setIdentity();
    xformA.setOrigin(bodyAXform.invXform(pivot));
    xformB.setOrigin(bodyBXform.invXform(pivot));
    var fixedJoint = new Ammo.btFixedConstraint(bodyA, bodyB, xformA, xformB);
    dynamicsWorld.addConstraint(fixedJoint, true);
    return fixedJoint;
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
    var gutHeight = h - u * 3.1;
    var headHeight = h - u / 2;
    var neckLength = (h - u) - shoulderHeight;
    var hipRadial = u * 0.7;
    var legUpperLength = u * 1.75;
    var legLowerLength = legUpperLength;
    var ankleLength = u / 3;
    var hipHeight = ankleLength + legLowerLength + legUpperLength;
    var ankleOffset = u / 5;
    boxes.push(createBox({
      id: "head", y: headHeight,
      w: u*0.8, d: u*0.8, h: u
    }, rootTransform));
    boxes.push(createBox({
      id: "chest", y: chestHeight,
      w: u*1.3, d: u*0.75, h: chestLength
    }, rootTransform));
    boxes.push(createBox({
      id: "gut", y: gutHeight,
      w: u*1.1, d: u*0.7, h: u
    }, rootTransform));
    // legs
    boxes.push(createBox({
      id: "l_leg_u",
      x: -hipRadial, y: ankleLength + legLowerLength + legUpperLength/2,
      w: u*0.6, d: u*0.6, h: legUpperLength
    }, rootTransform));
    boxes.push(createBox({
      id: "l_leg_l", x: -hipRadial, y: ankleLength + legLowerLength/2,
      w: u*0.5, d: u*0.5, h: legLowerLength
    }, rootTransform));
    boxes.push(createBox({
      id: "l_foot", x: -hipRadial, y: ankleLength/2, z:-ankleOffset,
      w: u*0.6, d: u, h: ankleLength
    }, rootTransform));
    boxes.push(createBox({
      id: "r_leg_u",
      x: hipRadial, y: ankleLength + legLowerLength + legUpperLength/2,
      w: u*0.6, d: u*0.6, h: legUpperLength
    }, rootTransform));
    boxes.push(createBox({
      id: "r_leg_l", x: hipRadial, y: ankleLength + legLowerLength/2,
      w: u*0.5, d: u*0.5, h: legLowerLength
    }, rootTransform));
    boxes.push(createBox({
      id: "r_foot", x: hipRadial, y: ankleLength/2, z:-ankleOffset,
      w: u*0.6, d: u, h: ankleLength
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
    // bodies.chest.setMassProps(0, new Ammo.btVector3(0,0,0));
    // Joints
    var l_shoulderPivotA = new Ammo.btVector3(-shoulderRadial, shoulderHeight-chestHeight, 0);
    var l_shoulderPivotB = new Ammo.btVector3(armUpperLength / 2, 0, 0);
    var l_shoulderTransformA = new Ammo.btTransform();
    l_shoulderTransformA.setIdentity();
    l_shoulderTransformA.setOrigin(l_shoulderPivotA);
    var l_shoulderTransformB = new Ammo.btTransform();
    l_shoulderTransformB.setIdentity();
    l_shoulderTransformB.setOrigin(l_shoulderPivotB);
    var l_shoulder = new Ammo.btConeTwistConstraint(
      bodies.chest, bodies.l_arm_u,
      l_shoulderTransformA, l_shoulderTransformB
    );
    //l_shoulder.setMotorTarget();
    dynamicsWorld.addConstraint(l_shoulder, true);
    joints.l_shoulder = l_shoulder;

    var l_elbowPivotA = new Ammo.btVector3(armLowerLength / 2, 0, 0);
    var l_elbowPivotB = new Ammo.btVector3(-armUpperLength / 2, 0, 0);
    var l_elbowAxis = new Ammo.btVector3(0, 0, 1);
    var l_elbow = new Ammo.btHingeConstraint(
      bodies.l_arm_l, bodies.l_arm_u,
      l_elbowPivotA, l_elbowPivotB, l_elbowAxis, l_elbowAxis
    );
    // l_elbow.enableAngularMotor(true, 0, 0.1);
    l_elbow.enableMotor(true);
    l_elbow.setMaxMotorImpulse(0.015);
    // Set motor target angle difference to reach in time dt
    l_elbow.setMotorTarget(Math.PI, 1);
    //l_elbow.setLimit(-Math.PI/4, Math.PI/4, 0.9, 0.1, 1.0);
    dynamicsWorld.addConstraint(l_elbow, true);
    joints.l_elbow = l_elbow;
    jointControllers.l_elbow = new HingeController(l_elbow);

    var r_shoulderPivotA = new Ammo.btVector3(shoulderRadial, shoulderHeight-chestHeight, 0);
    var r_shoulderPivotB = new Ammo.btVector3(-armUpperLength / 2, 0, 0);
    var r_shoulderTransformA = new Ammo.btTransform();
    r_shoulderTransformA.setIdentity();
    r_shoulderTransformA.setOrigin(r_shoulderPivotA);
    var r_shoulderTransformB = new Ammo.btTransform();
    r_shoulderTransformB.setIdentity();
    r_shoulderTransformB.setOrigin(r_shoulderPivotB);
    var r_shoulder = new Ammo.btConeTwistConstraint(
      bodies.chest, bodies.r_arm_u,
      r_shoulderTransformA, r_shoulderTransformB
    );
    //r_shoulder.setMotorTarget();
    dynamicsWorld.addConstraint(r_shoulder, true);
    joints.r_shoulder = r_shoulder;

    var r_elbowPivotA = new Ammo.btVector3(-armLowerLength / 2, 0, 0);
    var r_elbowPivotB = new Ammo.btVector3(armUpperLength / 2, 0, 0);
    var r_elbowAxis = new Ammo.btVector3(0, 0, 1);
    var r_elbow = new Ammo.btHingeConstraint(
      bodies.r_arm_l, bodies.r_arm_u,
      r_elbowPivotA, r_elbowPivotB, r_elbowAxis, r_elbowAxis
    );
    r_elbow.enableMotor(true);
    r_elbow.setMaxMotorImpulse(0.015);
    // Set motor target angle difference to reach in time dt
    r_elbow.setMotorTarget(Math.PI, 1);
    r_elbow.setLimit(0, 0.9*Math.PI, 0.9, 0.1, 1.0);
    dynamicsWorld.addConstraint(r_elbow, true);
    joints.r_elbow = r_elbow;
    jointControllers.r_elbow = new HingeController(r_elbow);

    var spinePivotA = new Ammo.btVector3(0, -chestLength / 2, u/6);
    var spinePivotB = new Ammo.btVector3(0, u / 2, u/6);
    var spineAxis = new Ammo.btVector3(1, 0, 0);
    var spine = new Ammo.btHingeConstraint(
      bodies.chest, bodies.gut,
      spinePivotA, spinePivotB, spineAxis, spineAxis
    );
    spine.enableAngularMotor(true, 0, 0.01);
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
    joints.l_hand = createFixed(bodies.l_hand, bodies.l_arm_l,
      new Ammo.btVector3(
        -shoulderRadial - armUpperLength - armLowerLength,
        shoulderHeight, 0
      ), rootTransform
    );
    joints.r_hand = createFixed(bodies.r_hand, bodies.r_arm_l,
      new Ammo.btVector3(
        shoulderRadial + armUpperLength + armLowerLength,
        shoulderHeight, 0
      ), rootTransform
    );

    // legs
    var r_hipPivotA = new Ammo.btVector3(hipRadial, hipHeight-gutHeight, 0);
    var r_hipPivotB = new Ammo.btVector3(0, legUpperLength/2, 0);
    var r_hipTransformA = new Ammo.btTransform();
    r_hipTransformA.setIdentity();
    r_hipTransformA.setOrigin(r_hipPivotA);
    var r_hipTransformB = new Ammo.btTransform();
    r_hipTransformB.setIdentity();
    r_hipTransformB.setOrigin(r_hipPivotB);
    var r_hip = new Ammo.btConeTwistConstraint(
      bodies.gut, bodies.r_leg_u,
      r_hipTransformA, r_hipTransformB
    );
    //r_hip.setMotorTarget();
    dynamicsWorld.addConstraint(r_hip, true);
    joints.r_hip = r_hip;

    var r_kneePivotA = new Ammo.btVector3(0, -legUpperLength/2, 0);
    var r_kneePivotB = new Ammo.btVector3(0, legLowerLength/2, 0);
    var r_kneeAxis = new Ammo.btVector3(1, 0, 0);
    var r_knee = new Ammo.btHingeConstraint(
      bodies.r_leg_u, bodies.r_leg_l,
      r_kneePivotA, r_kneePivotB, r_kneeAxis, r_kneeAxis
    );
    // r_knee.enableMotor(true);
    // r_knee.setMaxMotorImpulse(0.015);
    // Set motor target angle difference to reach in time dt
    // r_knee.setMotorTarget(Math.PI, 1);
    r_knee.setLimit(0, 0.9*Math.PI, 0.9, 0.1, 1.0);
    dynamicsWorld.addConstraint(r_knee, true);
    joints.r_knee = r_knee;
    // jointControllers.r_knee = new HingeController(r_knee);

    var r_anklePivotA = new Ammo.btVector3(0, -legLowerLength/2, 0);
    var r_anklePivotB = new Ammo.btVector3(0, ankleLength/2, ankleOffset);
    var r_ankleAxis = new Ammo.btVector3(1, 0, 0);
    var r_ankle = new Ammo.btHingeConstraint(
      bodies.r_leg_l, bodies.r_foot,
      r_anklePivotA, r_anklePivotB, r_ankleAxis, r_ankleAxis
    );
    // r_ankle.enableMotor(true);
    // r_ankle.setMaxMotorImpulse(0.015);
    // Set motor target angle difference to reach in time dt
    // r_ankle.setMotorTarget(Math.PI, 1);
    r_ankle.setLimit(0, 0.4*Math.PI, 0.9, 0.1, 1.0);
    dynamicsWorld.addConstraint(r_ankle, true);
    joints.r_ankle = r_ankle;
    // jointControllers.r_knee = new HingeController(r_knee);

    var l_hipPivotA = new Ammo.btVector3(-hipRadial, hipHeight-gutHeight, 0);
    var l_hipPivotB = new Ammo.btVector3(0, legUpperLength/2, 0);
    var l_hipTransformA = new Ammo.btTransform();
    l_hipTransformA.setIdentity();
    l_hipTransformA.setOrigin(l_hipPivotA);
    var l_hipTransformB = new Ammo.btTransform();
    l_hipTransformB.setIdentity();
    l_hipTransformB.setOrigin(l_hipPivotB);
    var l_hip = new Ammo.btConeTwistConstraint(
      bodies.gut, bodies.l_leg_u,
      l_hipTransformA, l_hipTransformB
    );
    //l_hip.setMotorTarget();
    dynamicsWorld.addConstraint(l_hip, true);
    joints.l_hip = l_hip;

    var l_kneePivotA = new Ammo.btVector3(0, -legUpperLength/2, 0);
    var l_kneePivotB = new Ammo.btVector3(0, legLowerLength/2, 0);
    var l_kneeAxis = new Ammo.btVector3(1, 0, 0);
    var l_knee = new Ammo.btHingeConstraint(
      bodies.l_leg_u, bodies.l_leg_l,
      l_kneePivotA, l_kneePivotB, l_kneeAxis, l_kneeAxis
    );
    // l_knee.enableMotor(true);
    // l_knee.setMaxMotorImpulse(0.015);
    // Set motor target angle difference to reach in time dt
    // l_knee.setMotorTarget(Math.PI, 1);
    l_knee.setLimit(0, 0.9*Math.PI, 0.9, 0.1, 1.0);
    dynamicsWorld.addConstraint(l_knee, true);
    joints.l_knee = l_knee;
    // jointControllers.l_knee = new HingeController(l_knee);

    var l_anklePivotA = new Ammo.btVector3(0, -legLowerLength/2, 0);
    var l_anklePivotB = new Ammo.btVector3(0, ankleLength/2, ankleOffset);
    var l_ankleAxis = new Ammo.btVector3(1, 0, 0);
    var l_ankle = new Ammo.btHingeConstraint(
      bodies.l_leg_l, bodies.l_foot,
      l_anklePivotA, l_anklePivotB, l_ankleAxis, l_ankleAxis
    );
    // l_ankle.enableMotor(true);
    // l_ankle.setMaxMotorImpulse(0.015);
    // Set motor target angle difference to reach in time dt
    // l_ankle.setMotorTarget(Math.PI, 1);
    l_ankle.setLimit(0, 0.4*Math.PI, 0.9, 0.1, 1.0);
    dynamicsWorld.addConstraint(l_ankle, true);
    joints.l_ankle = l_ankle;
    // jointControllers.l_knee = new HingeController(l_knee);

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
    var dt = 1/250;
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
      jointControllers.l_elbow.setTarget(data.controls.l_arm);
      jointControllers.r_elbow.setTarget(data.controls.l_arm);
    }
    else if (data.type === "drag-force") {
      if (data.object === null) {
        mouseDrag.set(null);
      }
      else {
        var object = bodies[data.object];
        var a = data.anchor;
        var p = data.position;
        var point = new Ammo.btVector3(p.x, p.y, p.z);
        var anchor = new Ammo.btVector3(a.x, a.y, a.z);
        mouseDrag.set(object, point, anchor, data.strength);
      }
    }
  };
});
