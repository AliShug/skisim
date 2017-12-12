/*jshint esversion: 6*/

// Humanish character
var p = {};
p.h = 1.8;
p.u = p.h / 7.5;
p.shoulderRadial = p.u * 0.9;
p.shoulderHeight = p.h - p.u * (4/3);
p.armUpperLength = p.u * (4/3);
p.armLowerLength = p.u * 1.2;
p.handLength = p.u * 0.8;
p.chestLength = p.u * 1.5;
p.chestHeight = p.h - p.u * 2;
p.gutHeight = p.h - p.u * 3.1;
p.headHeight = p.h - p.u / 2;
p.neckLength = (p.h - p.u) - p.shoulderHeight;
p.hipRadial = p.u * 0.7;
p.legUpperLength = p.u * 1.75;
p.legLowerLength = p.legUpperLength;
p.ankleLength = p.u / 3;
p.hipHeight = p.ankleLength + p.legLowerLength + p.legUpperLength;
p.ankleOffset = p.u / 5;
p.hipDamping = 0.3;
var defaultSkiier = p;

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
  constructor (joint, maxTorque = 0.1, springRange = 0.1, target = 0.0) {
    this.target = target;
    this.joint = joint;
    this.maxTorque = maxTorque;
    this.springRange = springRange;
    // this.pid = new PIDController(0.5, 0.1, 0.0);
    this.joint.enableMotor(true);
    this.min = -Math.PI;
    this.max = Math.PI;
    this.range = 2 * Math.PI;
  }

  setLimit(min, max, a, b, c) {
    this.min = min;
    this.max = max;
    this.range = max - min;
    this.joint.setLimit(min, max, a, b, c);
  }

  setTarget(target) {
    this.target = target;
  }

  update(dt) {
    var targetAngle = this.min + this.target*this.range;
    var state = this.joint.getHingeAngle();
    this.joint.setMotorTarget(targetAngle, 0.1);
    // Servo-like torque limiting
    var absError = Math.abs(state - targetAngle);
    var torqueCoeff = Math.min(absError/this.springRange, 1);
    this.joint.setMaxMotorImpulse(this.maxTorque*torqueCoeff);
  }
}

class ShoulderController {
  constructor (joint) {
    this.target = target;
    this.joint = joint;
    this.maxTorque = maxTorque;
    this.springRange = springRange;
    this.joint.enableMotor(true);
  }

  setLimit(x, y, z, a, b, c) {
    this.x_lim = x;
    this.y_lim = y;
    this.z_lim = z;
    this.joint.setLimit(x, y, z, a, b, c);
  }

  setTarget(x, y, z) {

  }

  update(dt) {
    this.joint.setMaxMotorImpulse(0.2);
    this.joint.setMotorTarget(new Ammo.btQuaternion(0,0,0,1));
  }
}

class Skiier {
  constructor(dynamicsWorld) {
    this.bodies = {};
    this.joints = {};
    this.jointControllers = {};
    this.world = dynamicsWorld;
    this.visibles = [];
  }

  update(dt) {
    for (var jointId in this.jointControllers) {
      this.jointControllers[jointId].update(dt);
    }
  }

  inputControls(controls) {
    this.jointControllers.l_elbow.setTarget(controls.l_elbow);
    this.jointControllers.r_elbow.setTarget(controls.r_elbow);
    this.joints.l_shoulder.setEquilibriumPoint(3, controls.l_shoulder_x);
    this.joints.l_shoulder.setEquilibriumPoint(4, controls.l_shoulder_y);
    this.joints.l_shoulder.setEquilibriumPoint(5, controls.l_shoulder_z);
  }

  // Returns array of rigid body descriptions to be passed to the render thread
  initSkiier(p, pinned = false) {
    var rootTransform = new Ammo.btTransform();
    rootTransform.setIdentity();
    rootTransform.setOrigin(new Ammo.btVector3(0, 5, 0));

    // abdomen
    this.createBox({
      id: "head", y: p.headHeight,
      w: p.u*0.8, d: p.u*0.8, h: p.u
    }, rootTransform);
    this.createBox({
      id: "chest", y: p.chestHeight,
      w: p.u*1.3, d: p.u*0.75, h: p.chestLength
    }, rootTransform);
    this.createBox({
      id: "gut", y: p.gutHeight,
      w: p.u*1.1, d: p.u*0.7, h: p.u
    }, rootTransform);
    // legs
    this.createBox({
      id: "l_leg_u",
      x: -p.hipRadial, y: p.ankleLength + p.legLowerLength + p.legUpperLength/2,
      w: p.u*0.6, d: p.u*0.6, h: p.legUpperLength
    }, rootTransform);
    this.createBox({
      id: "l_leg_l", x: -p.hipRadial, y: p.ankleLength + p.legLowerLength/2,
      w: p.u*0.5, d: p.u*0.5, h: p.legLowerLength
    }, rootTransform);
    this.createBox({
      id: "l_foot", x: -p.hipRadial, y: p.ankleLength/2, z:-p.ankleOffset,
      w: p.u*0.6, d: p.u, h: p.ankleLength
    }, rootTransform);
    this.createBox({
      id: "r_leg_u",
      x: p.hipRadial, y: p.ankleLength + p.legLowerLength + p.legUpperLength/2,
      w: p.u*0.6, d: p.u*0.6, h: p.legUpperLength
    }, rootTransform);
    this.createBox({
      id: "r_leg_l", x: p.hipRadial, y: p.ankleLength + p.legLowerLength/2,
      w: p.u*0.5, d: p.u*0.5, h: p.legLowerLength
    }, rootTransform);
    this.createBox({
      id: "r_foot", x: p.hipRadial, y: p.ankleLength/2, z:-p.ankleOffset,
      w: p.u*0.6, d: p.u, h: p.ankleLength
    }, rootTransform);
    // arms
    this.createBox({
      id: "l_arm_u",
      x: -p.shoulderRadial - p.armUpperLength/2,
      y: p.shoulderHeight,
      w: p.armUpperLength, d: p.u*0.4, h: p.u*0.4
    }, rootTransform);
    this.createBox({
      id: "l_arm_l",
      x: -p.shoulderRadial - p.armUpperLength - p.armLowerLength/2,
      y: p.shoulderHeight,
      w: p.armLowerLength, d: p.u*0.35, h: p.u*0.4
    }, rootTransform);
    this.createBox({
      id: "l_hand",
      x: -p.shoulderRadial - p.armUpperLength - p.armLowerLength - p.handLength/2,
      y: p.shoulderHeight,
      w: p.handLength, d: p.u*0.2, h: p.u*0.5
    }, rootTransform);
    this.createBox({
      id: "r_arm_u",
      x: p.shoulderRadial + p.armUpperLength/2,
      y: p.shoulderHeight,
      w: p.armUpperLength, d: p.u*0.4, h: p.u*0.4
    }, rootTransform);
    this.createBox({
      id: "r_arm_l",
      x: p.shoulderRadial + p.armUpperLength + p.armLowerLength/2,
      y: p.shoulderHeight,
      w: p.armLowerLength, d: p.u*0.35, h: p.u*0.4
    }, rootTransform);
    this.createBox({
      id: "r_hand",
      x: p.shoulderRadial + p.armUpperLength + p.armLowerLength + p.handLength/2,
      y: p.shoulderHeight,
      w: p.handLength, d: p.u*0.2, h: p.u*0.5
    }, rootTransform);

    // setting mass=0 creates a static body - pins the character in place
    if (pinned) {
      this.bodies.chest.setMassProps(0, new Ammo.btVector3(0,0,0));
    }

    // Joints
    var l_shoulderPivotA = new Ammo.btVector3(-p.shoulderRadial, p.shoulderHeight-p.chestHeight, 0);
    var l_shoulderPivotB = new Ammo.btVector3(p.armUpperLength / 2, 0, 0);
    var l_shoulderTransformA = new Ammo.btTransform();
    l_shoulderTransformA.setIdentity();
    l_shoulderTransformA.setOrigin(l_shoulderPivotA);
    var l_shoulderTransformB = new Ammo.btTransform();
    l_shoulderTransformB.setIdentity();
    l_shoulderTransformB.setRotation(new Ammo.btQuaternion(new Ammo.btVector3(1, 0, 0), Math.PI/2));
    l_shoulderTransformB.setOrigin(l_shoulderPivotB);
    // var l_shoulder = new Ammo.btConeTwistConstraint(
    //   this.bodies.chest, this.bodies.l_arm_u,
    //   l_shoulderTransformA, l_shoulderTransformB
    // );
    var l_shoulder = new Ammo.btGeneric6DofSpringConstraint(
      this.bodies.chest, this.bodies.l_arm_u,
      l_shoulderTransformA, l_shoulderTransformB, false
    );
    l_shoulder.setLinearLowerLimit(new Ammo.btVector3(0,0,0));
    l_shoulder.setLinearUpperLimit(new Ammo.btVector3(0,0,0));
    // Angular - twist (back/forward), back/forward, down/up
    l_shoulder.setAngularLowerLimit(new Ammo.btVector3(-Math.PI/2, -0.3*Math.PI, -0.46*Math.PI));
    l_shoulder.setAngularUpperLimit(new Ammo.btVector3(Math.PI/2, 0.6*Math.PI, 0.55*Math.PI));
    l_shoulder.enableSpring(3, true);
    l_shoulder.enableSpring(4, true);
    l_shoulder.enableSpring(5, true);
    l_shoulder.setStiffness(3, 50.0);
    l_shoulder.setStiffness(4, 200.0);
    l_shoulder.setStiffness(5, 200.0);
    l_shoulder.setDamping(3, 0.0005);
    l_shoulder.setDamping(4, 0.0005);
    l_shoulder.setDamping(5, 0.0005);
    this.world.addConstraint(l_shoulder, true);
    // this.jointControllers.l_shoulder = new ConeController(l_shoulder, 0.05, 0.1);
    // this.jointControllers.l_shoulder.setLimit(Math.PI/2, Math.PI/2, Math.PI/2, 0.9, 0.1, 1.0);
    this.joints.l_shoulder = l_shoulder;

    var l_elbowPivotA = new Ammo.btVector3(p.armLowerLength / 2, 0, 0);
    var l_elbowPivotB = new Ammo.btVector3(-p.armUpperLength / 2, 0, 0);
    var l_elbowAxis = new Ammo.btVector3(0, 0, -1);
    var l_elbow = new Ammo.btHingeConstraint(
      this.bodies.l_arm_l, this.bodies.l_arm_u,
      l_elbowPivotA, l_elbowPivotB, l_elbowAxis, l_elbowAxis
    );
    // Set motor target angle difference to reach in time dt
    this.world.addConstraint(l_elbow, true);
    this.joints.l_elbow = l_elbow;
    this.jointControllers.l_elbow = new HingeController(l_elbow);
    this.jointControllers.l_elbow.setLimit(0, 0.9*Math.PI, 0.9, 0.1, 1.0);

    var r_shoulderPivotA = new Ammo.btVector3(p.shoulderRadial, p.shoulderHeight-p.chestHeight, 0);
    var r_shoulderPivotB = new Ammo.btVector3(-p.armUpperLength / 2, 0, 0);
    var r_shoulderTransformA = new Ammo.btTransform();
    r_shoulderTransformA.setIdentity();
    r_shoulderTransformA.setOrigin(r_shoulderPivotA);
    var r_shoulderTransformB = new Ammo.btTransform();
    r_shoulderTransformB.setIdentity();
    r_shoulderTransformB.setRotation(new Ammo.btQuaternion(new Ammo.btVector3(1, 0, 0), Math.PI/2));
    r_shoulderTransformB.setOrigin(r_shoulderPivotB);
    var r_shoulder = new Ammo.btConeTwistConstraint(
      this.bodies.chest, this.bodies.r_arm_u,
      r_shoulderTransformA, r_shoulderTransformB
    );
    r_shoulder.setLimit(Math.PI/2, Math.PI/2, Math.PI/2, 0.9, 0.1, 1.0);
    this.world.addConstraint(r_shoulder, true);
    this.joints.r_shoulder = r_shoulder;

    var r_elbowPivotA = new Ammo.btVector3(-p.armLowerLength / 2, 0, 0);
    var r_elbowPivotB = new Ammo.btVector3(p.armUpperLength / 2, 0, 0);
    var r_elbowAxis = new Ammo.btVector3(0, 0, 1);
    var r_elbow = new Ammo.btHingeConstraint(
      this.bodies.r_arm_l, this.bodies.r_arm_u,
      r_elbowPivotA, r_elbowPivotB, r_elbowAxis, r_elbowAxis
    );
    // Set motor target angle difference to reach in time dt
    this.world.addConstraint(r_elbow, true);
    this.joints.r_elbow = r_elbow;
    this.jointControllers.r_elbow = new HingeController(r_elbow);
    this.jointControllers.r_elbow.setLimit(0, 0.9*Math.PI, 0.9, 0.1, 1.0);

    var spinePivotA = new Ammo.btVector3(0, -p.chestLength / 2, p.u/6);
    var spinePivotB = new Ammo.btVector3(0, p.u / 2, p.u/6);
    var spineAxis = new Ammo.btVector3(1, 0, 0);
    var spine = new Ammo.btHingeConstraint(
      this.bodies.chest, this.bodies.gut,
      spinePivotA, spinePivotB, spineAxis, spineAxis
    );
    // spine.enableAngularMotor(true, 0, 0.01);
    // spine.enableMotor(true);
    // spine.setMaxMotorImpulse(0.005);
    // Set motor target angle difference to reach in time dt
    // spine.setMotorTarget(Math.PI, 1);
    // +ve = backward lean
    spine.setLimit(-Math.PI/4, Math.PI/6, 0.9, 0.1, 1.0);
    this.world.addConstraint(spine, true);
    this.joints.spine = spine;

    var neckPivotA = new Ammo.btVector3(0, p.chestLength / 2 + p.neckLength / 2, p.u/6);
    var neckPivotB = new Ammo.btVector3(0, -p.u / 2 - p.neckLength / 2, p.u/6);
    var neckAxis = new Ammo.btVector3(1, 0, 0);
    var neck = new Ammo.btHingeConstraint(
      this.bodies.chest, this.bodies.head,
      neckPivotA, neckPivotB, neckAxis, neckAxis
    );
    // neck.enableMotor(true);
    // neck.setMaxMotorImpulse(0.005);
    // Set motor target angle difference to reach in time dt
    // neck.setMotorTarget(0, 1);
    // +ve = backwards lean
    neck.setLimit(-Math.PI/6, Math.PI/6, 0.9, 0.1, 1.0);
    this.world.addConstraint(neck, true);
    this.joints.neck = neck;

    // pin hands to lower arms
    this.joints.l_hand = this.createFixed(this.bodies.l_hand, this.bodies.l_arm_l,
      new Ammo.btVector3(
        -p.shoulderRadial - p.armUpperLength - p.armLowerLength,
        p.shoulderHeight, 0
      ), rootTransform
    );
    this.joints.r_hand = this.createFixed(this.bodies.r_hand, this.bodies.r_arm_l,
      new Ammo.btVector3(
        p.shoulderRadial + p.armUpperLength + p.armLowerLength,
        p.shoulderHeight, 0
      ), rootTransform
    );

    // legs
    var r_hipPivotA = new Ammo.btVector3(p.hipRadial, p.hipHeight-p.gutHeight, 0);
    var r_hipPivotB = new Ammo.btVector3(0, p.legUpperLength/2, 0);
    var r_hipTransformA = new Ammo.btTransform();
    r_hipTransformA.setIdentity();
    r_hipTransformA.setOrigin(r_hipPivotA);
    var r_hipTransformB = new Ammo.btTransform();
    r_hipTransformB.setIdentity();
    r_hipTransformB.setOrigin(r_hipPivotB);
    var r_hip = new Ammo.btConeTwistConstraint(
      this.bodies.gut, this.bodies.r_leg_u,
      r_hipTransformA, r_hipTransformB
    );
    r_hip.setLimit(Math.PI/4, 0.05, Math.PI/4, 0.9, 0.1, 1.0);
    r_hip.setDamping(p.hipDamping);
    this.world.addConstraint(r_hip, true);
    this.joints.r_hip = r_hip;

    var r_kneePivotA = new Ammo.btVector3(0, -p.legUpperLength/2, 0);
    var r_kneePivotB = new Ammo.btVector3(0, p.legLowerLength/2, 0);
    var r_kneeAxis = new Ammo.btVector3(1, 0, 0);
    var r_knee = new Ammo.btHingeConstraint(
      this.bodies.r_leg_u, this.bodies.r_leg_l,
      r_kneePivotA, r_kneePivotB, r_kneeAxis, r_kneeAxis
    );
    // r_knee.enableMotor(true);
    // r_knee.setMaxMotorImpulse(0.015);
    // Set motor target angle difference to reach in time dt
    // r_knee.setMotorTarget(Math.PI, 1);
    r_knee.setLimit(0, 0.9*Math.PI, 0.9, 0.1, 1.0);
    this.world.addConstraint(r_knee, true);
    this.joints.r_knee = r_knee;
    // this.jointControllers.r_knee = new HingeController(r_knee);

    var r_anklePivotA = new Ammo.btVector3(0, -p.legLowerLength/2, 0);
    var r_anklePivotB = new Ammo.btVector3(0, p.ankleLength/2, p.ankleOffset);
    var r_ankleAxis = new Ammo.btVector3(1, 0, 0);
    var r_ankle = new Ammo.btHingeConstraint(
      this.bodies.r_leg_l, this.bodies.r_foot,
      r_anklePivotA, r_anklePivotB, r_ankleAxis, r_ankleAxis
    );
    // r_ankle.enableMotor(true);
    // r_ankle.setMaxMotorImpulse(0.015);
    // Set motor target angle difference to reach in time dt
    // r_ankle.setMotorTarget(Math.PI, 1);
    r_ankle.setLimit(0, 0.4*Math.PI, 0.9, 0.1, 1.0);
    this.world.addConstraint(r_ankle, true);
    this.joints.r_ankle = r_ankle;
    // this.jointControllers.r_knee = new HingeController(r_knee);

    var l_hipPivotA = new Ammo.btVector3(-p.hipRadial, p.hipHeight-p.gutHeight, 0);
    var l_hipPivotB = new Ammo.btVector3(0, p.legUpperLength/2, 0);
    var l_hipTransformA = new Ammo.btTransform();
    l_hipTransformA.setIdentity();
    l_hipTransformA.setOrigin(l_hipPivotA);
    var l_hipTransformB = new Ammo.btTransform();
    l_hipTransformB.setIdentity();
    l_hipTransformB.setOrigin(l_hipPivotB);
    var l_hip = new Ammo.btConeTwistConstraint(
      this.bodies.gut, this.bodies.l_leg_u,
      l_hipTransformA, l_hipTransformB
    );
    l_hip.setLimit(Math.PI/4, 0.05, Math.PI/4, 0.9, 0.1, 1.0);
    l_hip.setDamping(p.hipDamping);
    this.world.addConstraint(l_hip, true);
    this.joints.l_hip = l_hip;

    var l_kneePivotA = new Ammo.btVector3(0, -p.legUpperLength/2, 0);
    var l_kneePivotB = new Ammo.btVector3(0, p.legLowerLength/2, 0);
    var l_kneeAxis = new Ammo.btVector3(1, 0, 0);
    var l_knee = new Ammo.btHingeConstraint(
      this.bodies.l_leg_u, this.bodies.l_leg_l,
      l_kneePivotA, l_kneePivotB, l_kneeAxis, l_kneeAxis
    );
    // l_knee.enableMotor(true);
    // l_knee.setMaxMotorImpulse(0.015);
    // Set motor target angle difference to reach in time dt
    // l_knee.setMotorTarget(Math.PI, 1);
    l_knee.setLimit(0, 0.9*Math.PI, 0.9, 0.1, 1.0);
    this.world.addConstraint(l_knee, true);
    this.joints.l_knee = l_knee;
    // this.jointControllers.l_knee = new HingeController(l_knee);

    var l_anklePivotA = new Ammo.btVector3(0, -p.legLowerLength/2, 0);
    var l_anklePivotB = new Ammo.btVector3(0, p.ankleLength/2, p.ankleOffset);
    var l_ankleAxis = new Ammo.btVector3(1, 0, 0);
    var l_ankle = new Ammo.btHingeConstraint(
      this.bodies.l_leg_l, this.bodies.l_foot,
      l_anklePivotA, l_anklePivotB, l_ankleAxis, l_ankleAxis
    );
    // l_ankle.enableMotor(true);
    // l_ankle.setMaxMotorImpulse(0.015);
    // Set motor target angle difference to reach in time dt
    // l_ankle.setMotorTarget(Math.PI, 1);
    l_ankle.setLimit(0, 0.4*Math.PI, 0.9, 0.1, 1.0);
    this.world.addConstraint(l_ankle, true);
    this.joints.l_ankle = l_ankle;
    // this.jointControllers.l_knee = new HingeController(l_knee);
  }

  createBox(params, transform = null) {
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
    this.world.addRigidBody(body);
    this.bodies[id] = body;
    body.forceActivationState(4); // disable deactivation
    this.visibles.push({
      id, x, y, z, w, d, h,
      ox: rotation.x(), oy: rotation.y(), oz: rotation.z(), ow: rotation.w()
    });
  }

  createFixed(bodyA, bodyB, pivot, transform = null) {
    if (transform !== null) {
      pivot = transform.xform(pivot).clone();
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
    this.world.addConstraint(fixedJoint, true);
    return fixedJoint;
  }
}
