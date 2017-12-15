/*jshint esversion: 6*/

// Humanish character
var defaultPID = [1, 0, 0.1];
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
p.gutLength = p.u;
p.headHeight = p.h - p.u / 2;
p.neckLength = (p.h - p.u) - p.shoulderHeight;
p.hipRadial = p.u * 0.65;
p.legUpperLength = p.u * 1.75;
p.legLowerLength = p.legUpperLength;
p.ankleLength = p.u / 3;
p.hipHeight = p.ankleLength + p.legLowerLength + p.legUpperLength;
p.hipDamping = 0.3;
p.legUpperThickness = p.u * 0.7;
p.legLowerThickness = p.u * 0.6;
p.armUpperThickness = p.u * 0.4;
p.armLowerThickness = p.u * 0.35;
// joint settings
p.kneeLimits = [0, 0.9*Math.PI];
p.kneeTorque = 0.9;
p.kneePid = [3, 0, 0.2];
p.ankleLimits = [-Math.PI/2, Math.PI/2];
p.ankleTorque = 1.5;
p.anklePid = [5, 0, 0.4];
p.hipTorques = [0.4, 0.6];
p.hipPids = [[1.5, 0, 0.2], [2.8, 0, 0.2]];
p.hipLimits = [[-Math.PI/4, Math.PI/4], [-0.1*Math.PI, 0.7*Math.PI]];
p.spineTorques = [0.4, 0.4]; // forward, sideways
p.spinePids = [[2, 0, 0.3], [1.5, 0, 0.2]];
p.neckTorque = 0.05;
p.neckPid = [1, 0, 0.15];
p.neckLimits = [-Math.PI/5, Math.PI/5];
p.elbowTorque = 0.1;
p.shoulderTorques = [0.1, 0.1];
p.shoulderLimits = [[-Math.PI/4, 0.7*Math.PI], [-Math.PI/2, Math.PI/2]];
// skiis!
p.skiFriction = 0.0;
p.skiDensity = 0.2;
p.skiOffset = p.u;

var defaultSkiier = p;

var axisOptions = [
  new Ammo.btVector3(1, 0, 0),
  new Ammo.btVector3(0, 1, 0),
  new Ammo.btVector3(0, 0, 1)
];
var axisLabels = ['x', 'y', 'z'];

class Ski {
  constructor(width, length, dynamicsWorld, body, ground) {
    this.world = dynamicsWorld;
    this.body = body;
    this.ground = ground;
    this.constraints = [];
    this.xoffset = width/2;
    this.zoffset = length/2;
    this.constraintOffsets = [
      [-1, -1], [1, -1],
      [-1, 1],  [1, 1]
    ];
    // this.contactNormals = [];
    // for (var i = 0; i < 4; i++) {
    //   var offset = this.constraintOffsets[i];
    //   this.createSlidingConstraint(offset[0]*this.xoffset, offset[1]*this.zoffset);
    //   this.contactNormals[i] = new Ammo.btVector3();
    // }
    var rayCaster = new Ammo.btDefaultVehicleRaycaster(this.world);
    this.tuning = new Ammo.btVehicleTuning();
    this.vehicle = new Ammo.btRaycastVehicle(this.tuning, this.body, rayCaster);
    this.vehicle.setCoordinateSystem(0, 1, 2);
    this.world.addAction(this.vehicle);
    this.wheels = [];
    var rad = 0.2;
    var thick = 0.01;
    this.createWheel(true, new Ammo.btVector3(-this.xoffset-0.02, 0.15, -this.zoffset-0.1), rad, thick, 0);
    this.createWheel(true, new Ammo.btVector3(this.xoffset+0.02, 0.15, -this.zoffset-0.1), rad, thick, 1);
    // this.createWheel(false, new Ammo.btVector3(-this.xoffset-0.02, 0.15, 0), rad, thick, 2);
    // this.createWheel(false, new Ammo.btVector3(this.xoffset+0.02, 0.15, 0), rad, thick, 3);
    this.createWheel(false, new Ammo.btVector3(-this.xoffset-0.02, 0.15, this.zoffset+0.1), rad, thick, 2);
    this.createWheel(false, new Ammo.btVector3(this.xoffset+0.02, 0.15, this.zoffset+0.1), rad, thick, 3);
    this.time = 0.0;
  }

  createWheel(isFront, pos, radius, width, index) {
    var directionCS0 = new Ammo.btVector3(0, -1, 0);
    var axleCS = new Ammo.btVector3(-1, 0, 0);
    var suspensionRestLength = 0.0;
    var suspensionCompression = 180;
    var suspensionRelaxation = 1.0;
    var suspensionStiffness = 5000;
    var rollInfluence = 0.05;
    var slipFriction = 10;
    var wheel = this.vehicle.addWheel(
      pos, directionCS0, axleCS, suspensionRestLength, radius, this.tuning,
      isFront
    );
    wheel.set_m_suspensionStiffness(suspensionStiffness);
    wheel.set_m_wheelsDampingRelaxation(suspensionRelaxation);
    wheel.set_m_wheelsDampingCompression(suspensionCompression);
    wheel.set_m_frictionSlip(slipFriction);
    wheel.set_m_rollInfluence(rollInfluence);
    this.wheels[index] = wheel;
    Ammo.destroy(axleCS);
    Ammo.destroy(directionCS0);
  }
}

class Skiier {
  constructor(dynamicsWorld, ground) {
    this.bodies = {};
    this.joints = {};
    this.jointControllers = {};
    this.world = dynamicsWorld;
    this.visibles = [];
    this.rootTransform = new Ammo.btTransform();
    this.setPinned = false;
    this.skiConstraints = {};
    this.skiis = {};
    this.ground = ground;
    this.controller = new SkiierController(this);
  }

  update(dt) {
    // Get new control input
    this.controller.update(dt);
    // Individual joint controllers (these apply the PD/PID torque control)
    for (var jointId in this.jointControllers) {
      this.jointControllers[jointId].update(dt);
    }
  }

  postUpdate(dt) {
    // for (var skiId in this.skiis) {
    //   this.skiis[skiId].postUpdate(dt);
    // }
    // var vehicle = this.skiis.l_ski.vehicle;
    // vehicle.updateWheelTransform(0, true);
    // var transform = vehicle.getWheelTransformWS(0);
    // this.bodies.wheeltest.getCenterOfMassTransform().setOrigin(transform.getOrigin());
    // this.bodies.wheeltest.getCenterOfMassTransform().setBasis(transform.getBasis());
    // transform = vehicle.getWheelTransformWS(1);
    // this.bodies.wheeltest1.getCenterOfMassTransform().setOrigin(transform.getOrigin());
    // this.bodies.wheeltest1.getCenterOfMassTransform().setBasis(transform.getBasis());
    // transform = vehicle.getWheelTransformWS(2);
    // this.bodies.wheeltest2.getCenterOfMassTransform().setOrigin(transform.getOrigin());
    // this.bodies.wheeltest2.getCenterOfMassTransform().setBasis(transform.getBasis());
    // transform = vehicle.getWheelTransformWS(3);
    // this.bodies.wheeltest3.getCenterOfMassTransform().setOrigin(transform.getOrigin());
    // this.bodies.wheeltest3.getCenterOfMassTransform().setBasis(transform.getBasis());
  }

  inputControls(controls) {
    this.controller.setInput(controls);
  }

  generateBodies(p) {
    this.createBox({
      id: "head", y: p.headHeight,
      w: p.u*0.8, d: p.u*0.8, h: p.u
    }, this.rootTransform);
    this.createBox({
      id: "chest", y: p.chestHeight,
      w: p.u*1.3, d: p.u*0.75, h: p.chestLength,
      group: 4, collision: 1|2|4,
    }, this.rootTransform);
    this.createBox({
      id: "gut", y: p.gutHeight,
      w: p.u*1.1, d: p.u*0.7, h: p.gutLength,
      group: 8, collision: 1|2|8,
    }, this.rootTransform);
    // legs
    this.createBox({
      id: "l_leg_u",
      x: -p.hipRadial, y: p.ankleLength + p.legLowerLength + p.legUpperLength/2,
      w: p.legUpperThickness, d: p.legUpperThickness, h: p.legUpperLength,
      group: 4, collision: 1|2|4,
    }, this.rootTransform);
    this.createBox({
      id: "r_leg_u",
      x: p.hipRadial, y: p.ankleLength + p.legLowerLength + p.legUpperLength/2,
      w: p.legUpperThickness, d: p.legUpperThickness, h: p.legUpperLength,
      group: 4, collision: 1|2|4,
    }, this.rootTransform);
    this.createBox({
      id: "l_leg_l", x: -p.hipRadial, y: p.ankleLength + p.legLowerLength/2,
      w: p.legLowerThickness, d: p.legLowerThickness, h: p.legLowerLength,
      group: 8, collision: 1|2|8,
    }, this.rootTransform);
    this.createBox({
      id: "r_leg_l", x: p.hipRadial, y: p.ankleLength + p.legLowerLength/2,
      w: p.legLowerThickness, d: p.legLowerThickness, h: p.legLowerLength,
      group: 8, collision: 1|2|8,
    }, this.rootTransform);
    this.createSki({
      id: "l_ski", x: -p.hipRadial, y: p.ankleLength/2, z:-p.skiOffset,
      w: p.legUpperThickness, d: p.u*9, h: p.ankleLength,
      group: 4, collision: 1|4,
      friction: p.skiFriction,
      density: p.skiDensity,
    }, this.rootTransform);
    this.createSki({
      id: "r_ski", x: p.hipRadial, y: p.ankleLength/2, z:-p.skiOffset,
      w: p.legUpperThickness, d: p.u*9, h: p.ankleLength,
      group: 4, collision: 1|4,
      friction: p.skiFriction,
      density: p.skiDensity,
    }, this.rootTransform);
    // arms
    this.createBox({
      id: "l_arm_u",
      x: -p.shoulderRadial - p.armUpperLength/2,
      y: p.shoulderHeight,
      w: p.armUpperLength, d: p.armUpperThickness, h: p.armUpperThickness,
      group: 8, collision: 1|2|8,
    }, this.rootTransform);
    this.createBox({
      id: "r_arm_u",
      x: p.shoulderRadial + p.armUpperLength/2,
      y: p.shoulderHeight,
      w: p.armUpperLength, d: p.armUpperThickness, h: p.armUpperThickness,
      group: 8, collision: 1|2|8,
    }, this.rootTransform);
    this.createBox({
      id: "l_arm_l",
      x: -p.shoulderRadial - p.armUpperLength - p.armLowerLength/2,
      y: p.shoulderHeight,
      w: p.armLowerLength, d: p.armLowerThickness, h: p.armUpperThickness,
      group: 4, collision: 1|2|4,
    }, this.rootTransform);
    this.createBox({
      id: "r_arm_l",
      x: p.shoulderRadial + p.armUpperLength + p.armLowerLength/2,
      y: p.shoulderHeight,
      w: p.armLowerLength, d: p.armLowerThickness, h: p.armUpperThickness,
      group: 4, collision: 1|2|4,
    }, this.rootTransform);
    this.createBox({
      id: "l_hand",
      x: -p.shoulderRadial - p.armUpperLength - p.armLowerLength - p.handLength/2,
      y: p.shoulderHeight,
      w: p.handLength, d: p.u*0.2, h: p.u*0.5,
      collision: 0,
    }, this.rootTransform);
    this.createBox({
      id: "r_hand",
      x: p.shoulderRadial + p.armUpperLength + p.armLowerLength + p.handLength/2,
      y: p.shoulderHeight,
      w: p.handLength, d: p.u*0.2, h: p.u*0.5,
      collision: 0,
    }, this.rootTransform);
  }

  generateJoints(p) {
    // upper body
    this.createDualCompoundJoint({
      id: 'l_shoulder',
      bodyA: this.bodies.chest, bodyB: this.bodies.l_arm_u,
      pivot: new Ammo.btVector3(-p.shoulderRadial, p.shoulderHeight, 0),
      thickness: p.armUpperThickness,
      torques: p.shoulderTorques,
      limits: p.shoulderLimits,
      flipAxes: [true, false],
    }, this.rootTransform);
    this.createDualCompoundJoint({
      id: 'r_shoulder',
      bodyA: this.bodies.chest, bodyB: this.bodies.r_arm_u,
      pivot: new Ammo.btVector3(p.shoulderRadial, p.shoulderHeight, 0),
      thickness: p.armUpperThickness,
      flipAxes: [false, true],
      torques: p.shoulderTorques,
      limits: p.shoulderLimits,
    }, this.rootTransform);
    this.createHinge({
      id: 'l_elbow',
      bodyA: this.bodies.l_arm_l, bodyB: this.bodies.l_arm_u,
      pivot: new Ammo.btVector3(-p.shoulderRadial-p.armUpperLength, p.shoulderHeight, 0),
      limits: [0, 0.9*Math.PI],
      axis: 2,
      torque: p.elbowTorque,
    }, this.rootTransform);
    this.createHinge({
      id: 'r_elbow',
      bodyA: this.bodies.r_arm_l, bodyB: this.bodies.r_arm_u,
      pivot: new Ammo.btVector3(p.shoulderRadial+p.armUpperLength, p.shoulderHeight, 0),
      limits: [0, 0.9*Math.PI],
      axis: 2, flipAxis: true,
      torque: p.elbowTorque,
    }, this.rootTransform);
    this.createDualCompoundJoint({
      id: 'spine',
      bodyA: this.bodies.chest, bodyB: this.bodies.gut,
      pivot: new Ammo.btVector3(0, p.gutHeight+p.gutLength/2, p.u/6),
      limits: [[-Math.PI/4, Math.PI/6], [-Math.PI/4, Math.PI/4]],
      axes: [0, 2],
      thickness: p.u/2,
      torques: p.spineTorques, pids: p.spinePids,
    }, this.rootTransform);
    this.createHinge({
      id: 'neck',
      bodyA: this.bodies.chest, bodyB: this.bodies.head,
      pivot: new Ammo.btVector3(0, p.shoulderHeight+p.neckLength/2, p.u/6),
      limits: p.neckLimits,
      axis: 0, flipAxis: true,
      torque: p.neckTorque, pid: p.neckPid,
    }, this.rootTransform);

    // pin hands to lower arms
    this.joints.l_hand = this.createFixed(this.bodies.l_hand, this.bodies.l_arm_l,
      new Ammo.btVector3(
        -p.shoulderRadial - p.armUpperLength - p.armLowerLength,
        p.shoulderHeight, 0
      ), this.rootTransform
    );
    this.joints.r_hand = this.createFixed(this.bodies.r_hand, this.bodies.r_arm_l,
      new Ammo.btVector3(
        p.shoulderRadial + p.armUpperLength + p.armLowerLength,
        p.shoulderHeight, 0
      ), this.rootTransform
    );

    // legs
    this.createDualCompoundJoint({
      id: 'l_hip',
      bodyA: this.bodies.gut, bodyB: this.bodies.l_leg_u,
      pivot: new Ammo.btVector3(-p.hipRadial, p.hipHeight, 0),
      thickness: p.legUpperThickness,
      limits: p.hipLimits,
      axes: [1, 0],
      torques: p.hipTorques, pids: p.hipPids,
    }, this.rootTransform);
    this.createDualCompoundJoint({
      id: 'r_hip',
      bodyA: this.bodies.gut, bodyB: this.bodies.r_leg_u,
      pivot: new Ammo.btVector3(p.hipRadial, p.hipHeight, 0),
      thickness: p.legUpperThickness,
      limits: p.hipLimits,
      axes: [1, 0],
      torques: p.hipTorques, pids: p.hipPids,
    }, this.rootTransform);
    this.createHinge({
      id: 'l_knee',
      bodyA: this.bodies.l_leg_u, bodyB: this.bodies.l_leg_l,
      pivot: new Ammo.btVector3(-p.hipRadial, p.hipHeight-p.legUpperLength, 0),
      limits: p.kneeLimits,
      axis: 0, flipAxis: true,
      torque: p.kneeTorque, pid: p.kneePid,
    }, this.rootTransform);
    this.createHinge({
      id: 'r_knee',
      bodyA: this.bodies.r_leg_u, bodyB: this.bodies.r_leg_l,
      pivot: new Ammo.btVector3(p.hipRadial, p.hipHeight-p.legUpperLength, 0),
      limits: p.kneeLimits,
      axis: 0, flipAxis: true,
      torque: p.kneeTorque, pid: p.kneePid,
    }, this.rootTransform);
    this.createHinge({
      id: 'l_ankle',
      bodyA: this.bodies.l_leg_l, bodyB: this.bodies.l_ski,
      pivot: new Ammo.btVector3(-p.hipRadial, p.hipHeight-p.legUpperLength-p.legLowerLength, 0),
      limits: p.ankleLimits,
      axis: 0, flipAxis: true,
      torque: p.ankleTorque, pid: p.anklePid,
    }, this.rootTransform);
    this.createHinge({
      id: 'r_ankle',
      bodyA: this.bodies.r_leg_l, bodyB: this.bodies.r_ski,
      pivot: new Ammo.btVector3(p.hipRadial, p.hipHeight-p.legUpperLength-p.legLowerLength, 0),
      limits: p.ankleLimits,
      axis: 0, flipAxis: true,
      torque: p.ankleTorque, pid: p.anklePid,
    }, this.rootTransform);
  }

  // Returns array of rigid body descriptions to be passed to the render thread
  initSkiier(p, pinned = false) {
    this.rootTransform.setIdentity();
    var translation = new Ammo.btVector3(0, 5, -2);
    this.rootTransform.setOrigin(translation);
    Ammo.destroy(translation);

    this.generateBodies(p);
    this.generateJoints(p);

    // this.createBox({
    //   id: "wheeltest",
    //   w: 0.2, d: 0.1, h: 0.2,
    //   x: 0, y: 0, z: 0,
    //   collision: 0,
    // });
    // this.createBox({
    //   id: "wheeltest1",
    //   w: 0.2, d: 0.1, h: 0.2,
    //   x: 0, y: 0, z: 0,
    //   collision: 0,
    // });
    // this.createBox({
    //   id: "wheeltest2",
    //   w: 0.2, d: 0.1, h: 0.2,
    //   x: 0, y: 0, z: 0,
    //   collision: 0,
    // });
    // this.createBox({
    //   id: "wheeltest3",
    //   w: 0.2, d: 0.1, h: 0.2,
    //   x: 0, y: 0, z: 0,
    //   collision: 0,
    // });

    // setting mass=0 creates a static body - pins the character in place
    if (pinned) {
      this.setPinned = true;
      this.bodies.l_ski.makeStatic();
      this.bodies.r_ski.makeStatic();
    }
    else {
      this.setPinned = false;
    }
  }

  createBox(params, transform = null) {
    var {
      id,
      x=0, y=0, z=0,
      w=1, d=1, h=1,
      yaw=0, pitch=0, roll=0,
      collision=-1,
      group=1,
      density=1,
      friction=0.4,
    } = params;
    var dims = new Ammo.btVector3(w/2, h/2, d/2);
    var boxShape = new Ammo.btBoxShape(dims);
    var rotation = new Ammo.btQuaternion();
    rotation.setEulerZYX(yaw, pitch, roll);
    var translation = new Ammo.btVector3(x, y, z);
    if (transform !== null) {
      translation = translation.op_add(transform.getOrigin());
    }
    var startTransform = new Ammo.btTransform(rotation, translation);
    var mass = w*d*h*1000*density;
    var localInertia = new Ammo.btVector3(0, 0, 0);
    boxShape.calculateLocalInertia(mass, localInertia);
    var myMotionState = new Ammo.btDefaultMotionState(startTransform);
    var rbInfo = new Ammo.btRigidBodyConstructionInfo(mass, myMotionState, boxShape, localInertia);
    rbInfo.set_m_friction(friction);
    var body = new Ammo.btRigidBody(rbInfo);
    body.startMass = mass;
    body.startInertia = localInertia.clone();
    this.world.addRigidBody(body, group, collision);
    this.bodies[id] = body;
    body.forceActivationState(4); // disable deactivation
    this.visibles.push({
      id, x, y, z, w, d, h,
      ox: rotation.x(), oy: rotation.y(), oz: rotation.z(), ow: rotation.w()
    });
    Ammo.destroy(rotation);
    Ammo.destroy(dims);
    Ammo.destroy(translation);
    Ammo.destroy(startTransform);
    Ammo.destroy(localInertia);
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
    Ammo.destroy(xformA);
    Ammo.destroy(xformB);
    Ammo.destroy(pivot); // clone() creates a new heap object
    return fixedJoint;
  }

  createHinge(params, transform = null) {
    var {
      id, pivot, bodyA, bodyB,
      torque=0.05,
      limits=[-Math.PI/2, Math.PI/2],
      pid = defaultPID,
      axis,
      flipAxis=false,
    } = params;

    if (transform !== null) {
      pivot.copy(transform.xform(pivot));
    }

    // convert to local pivots
    var xformA = bodyA.getCenterOfMassTransform();
    var xformB = bodyB.getCenterOfMassTransform();
    var pivotA = xformA.invXform(pivot).clone();
    var pivotB = xformB.invXform(pivot).clone();
    axis = axisOptions[axis].clone();
    if (flipAxis) {
      axis.op_mul(-1);
    }

    var joint = new Ammo.btHingeConstraint(
      bodyA, bodyB,
      pivotA, pivotB, axis, axis, true
    );
    var controller = new HingeController(joint, torque, 0.5, flipAxis);
    controller.setLimit(limits[0], limits[1], 0.9, 0.1, 1.0);
    controller.setPID(pid[0], pid[1], pid[2]);
    this.jointControllers[id] = controller;
    this.joints[id] = joint;
    this.world.addConstraint(joint);
  }

  createDualCompoundJoint(params, transform = null) {
    // creates a colocated joint chain with 3 axes of movement
    var {
      id, pivot, bodyA, bodyB, thickness,
      torques=[0.05, 0.05],
      limits=[[-Math.PI/2, Math.PI/2], [-Math.PI/2, Math.PI/2]],
      pids = [defaultPID, defaultPID],
      axes = [1, 2],
      flipAxes = [false, false],
    } = params;

    var loc;
    if (transform !== null) {
      loc = pivot.clone();
      pivot.copy(transform.xform(pivot));
    }

    // convert to local pivots
    var xformA = bodyA.getCenterOfMassTransform();
    var xformB = bodyB.getCenterOfMassTransform();
    var pivotA = xformA.invXform(pivot).clone();
    var pivotB = xformB.invXform(pivot).clone();
    // interim object
    this.createBox({
      id: id+'_c', x: loc.x(), y: loc.y(), z: loc.z(),
      w: thickness, d: thickness, h: thickness, collision: 0,
      density: 0.5,
    }, transform);
    var pivot0 = new Ammo.btVector3(0, 0, 0);
    var joint;
    for (var i = 0; i < 2; i++) {
      // first joint
      var axis = axisOptions[axes[i]].clone();
      if (flipAxes[i]) {
        axis.op_mul(-1);
      }
      if (i === 0) {
        joint = new Ammo.btHingeConstraint(
          bodyA, this.bodies[id+'_c'],
          pivotA, pivot0, axis, axis, true
        );
      }
      else {
        joint = new Ammo.btHingeConstraint(
          this.bodies[id+'_c'], bodyB,
          pivot0.clone(), pivotB, axis, axis, true
        );
      }
      var controller = new HingeController(joint, torques[0], 0.5, flipAxes[i]);
      controller.setLimit(limits[i][0], limits[i][1], 0.9, 0.1, 1.0);
      controller.setPID(pids[i][0], pids[i][1], pids[i][2]);
      this.jointControllers[id+'_'+axisLabels[i]] = controller;
      this.joints[id+'_'+axisLabels[i]] = joint;
      this.world.addConstraint(joint);
    }
    Ammo.destroy(pivotA);
    Ammo.destroy(pivotB);
    Ammo.destroy(pivot0);
    Ammo.destroy(loc);
  }

  createSki(params, transform = null) {
    this.createBox(params, transform);
    var ski = new Ski(params.w, params.d, this.world, this.bodies[params.id], this.ground);
    this.skiis[params.id] = ski;
  }
}
