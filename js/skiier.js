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
p.gutLength = p.u;
p.headHeight = p.h - p.u / 2;
p.neckLength = (p.h - p.u) - p.shoulderHeight;
p.hipRadial = p.u * 0.7;
p.legUpperLength = p.u * 1.75;
p.legLowerLength = p.legUpperLength;
p.ankleLength = p.u / 3;
p.hipHeight = p.ankleLength + p.legLowerLength + p.legUpperLength;
p.ankleOffset = p.u / 5;
p.hipDamping = 0.3;
p.legUpperThickness = p.u * 0.6;
p.legLowerThickness = p.u * 0.5;
p.armUpperThickness = p.u * 0.4;
p.armLowerThickness = p.u * 0.35;
// joint settings
p.kneeLimits = [0, 0.9*Math.PI];
p.kneeTorque = 0.35;
p.ankleLimits = [-Math.PI/2, Math.PI/2];
p.ankleTorque = 1.5;
p.anklePid = [3, 0, 0.2];
p.hipTorques = [0.2, 0.4];
p.spineTorques = [0.2, 0.2];
p.neckTorque = 0.05;
p.elbowTorque = 0.05;
p.shoulderTorques = [0.1, 0.1];
// skiis!
p.skiFriction = 0.0;

var defaultSkiier = p;

var axisOptions = [
  new Ammo.btVector3(1, 0, 0),
  new Ammo.btVector3(0, 1, 0),
  new Ammo.btVector3(0, 0, 1)
];
var axisLabels = ['x', 'y', 'z'];
var defaultPID = [1, 0, 0.1];

class Ski {
  constructor(dynamicsWorld, body, ground) {
    this.world = dynamicsWorld;
    this.body = body;
    this.ground = ground;
    this.constraints = [];
    this.constraintOffsets = [-1, 1];
    for (var i = 0; i < 2; i++) {
      this.createSlidingConstraint(this.constraintOffsets[i]);
    }

    this.time = 0.0;
  }

  createSlidingConstraint(zpos) {
    // generic constraints - responsible for ground contact behaviour
    var limitTransform = new Ammo.btTransform();
    limitTransform.setIdentity();
    limitTransform.setOrigin(new Ammo.btVector3(0, 0, zpos));
    var rootTransform = new Ammo.btTransform();
    rootTransform.setIdentity();
    var limitConstraint = new Ammo.btGeneric6DofConstraint(
      this.ground, this.body,
      rootTransform, limitTransform, true
    );
    limitConstraint.setAngularLowerLimit(new Ammo.btVector3(1, 1, 1));
    limitConstraint.setAngularUpperLimit(new Ammo.btVector3(0, 0, 0));
    limitConstraint.setLinearLowerLimit(new Ammo.btVector3(1, 0.05, 1));
    limitConstraint.setLinearUpperLimit(new Ammo.btVector3(0, 1000, 0));
    this.world.addConstraint(limitConstraint);
    this.constraints.push(limitConstraint);
  }

  update(dt) {
    for (var i = 0; i < 2; i++) {
      var constraint = this.constraints[i];
      var start = new Ammo.btVector3(0, 0, this.constraintOffsets[i]);
      var end = new Ammo.btVector3(0, -5, this.constraintOffsets[i]);
      var transform = this.body.getCenterOfMassTransform();
      start.copy(transform.xform(start));
      end.copy(transform.xform(end));
      var result = new Ammo.ClosestRayResultCallback(start, end);
      this.world.rayTest(start, end, result);

      if (result.hasHit()) {
        // references - can modify in place
        var frame = constraint.getFrameOffsetA();
        var basis = frame.getBasis();
        var hitPoint = result.get_m_hitPointWorld();
        if (hitPoint.distance(start) > 0.4) {
          constraint.setEnabled(false);
        }
        else {
          constraint.setEnabled(true);
        }
        frame.setOrigin(hitPoint);
        var n = result.get_m_hitNormalWorld();
        if (n.y() !== 1) {
          var y = new Ammo.btVector3(0, 1, 0);
          var a = n.cross(y).clone();
          var b = n.cross(a).clone();
          basis.setValue(b.x(), n.x(), a.x(), b.y(), n.y(), a.y(), b.z(), n.z(), a.z());
          Ammo.destroy(a);
          Ammo.destroy(b);
          Ammo.destroy(y);
        }
      }
      else {
        constraint.setEnabled(false);
      }
      // Woo C++
      Ammo.destroy(start);
      Ammo.destroy(end);
      Ammo.destroy(result);
    }
    // this.time += dt;
    // if (this.time > 1) {
    //   result.get_m_hitPointWorld().prettyPrint();
    //   this.time = 0.0;
    // }
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
  }

  update(dt) {
    for (var jointId in this.jointControllers) {
      this.jointControllers[jointId].update(dt);
    }
    for (var skiId in this.skiis) {
      this.skiis[skiId].update(dt);
    }
  }

  inputControls(controls) {
    this.jointControllers.l_elbow.setTarget(controls.l_elbow);
    this.jointControllers.r_elbow.setTarget(controls.r_elbow);
    this.jointControllers.l_shoulder_x.setTarget(controls.l_shoulder_x);
    this.jointControllers.l_shoulder_y.setTarget(controls.l_shoulder_y);
    this.jointControllers.r_shoulder_x.setTarget(controls.r_shoulder_x);
    this.jointControllers.r_shoulder_y.setTarget(controls.r_shoulder_y);
    this.jointControllers.l_hip_x.setTarget(controls.l_hip_x);
    this.jointControllers.l_hip_y.setTarget(controls.l_hip_y);
    this.jointControllers.r_hip_x.setTarget(controls.r_hip_x);
    this.jointControllers.r_hip_y.setTarget(controls.r_hip_y);
    this.jointControllers.l_knee.setTarget(controls.l_knee);
    this.jointControllers.r_knee.setTarget(controls.r_knee);
    this.jointControllers.l_ankle.setTarget(controls.l_ankle);
    this.jointControllers.r_ankle.setTarget(controls.r_ankle);
    this.jointControllers.neck.setTarget(controls.neck);
    this.jointControllers.spine_x.setTarget(controls.spine);
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
      id: "l_foot", x: -p.hipRadial, y: p.ankleLength/2, z:-p.ankleOffset,
      w: p.legUpperThickness, d: p.u*9, h: p.ankleLength,
      group: 4, collision: 1|4,
      friction: p.skiFriction,
    }, this.rootTransform);
    this.createSki({
      id: "r_foot", x: p.hipRadial, y: p.ankleLength/2, z:-p.ankleOffset,
      w: p.legUpperThickness, d: p.u*9, h: p.ankleLength,
      group: 4, collision: 1|4,
      friction: p.skiFriction,
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
    }, this.rootTransform);
    this.createDualCompoundJoint({
      id: 'r_shoulder',
      bodyA: this.bodies.chest, bodyB: this.bodies.r_arm_u,
      pivot: new Ammo.btVector3(p.shoulderRadial, p.shoulderHeight, 0),
      thickness: p.armUpperThickness,
      flipAxes: [true, true],
      torques: p.shoulderTorques,
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
      torques: p.spineTorques,
    }, this.rootTransform);
    this.createHinge({
      id: 'neck',
      bodyA: this.bodies.chest, bodyB: this.bodies.head,
      pivot: new Ammo.btVector3(0, p.shoulderHeight+p.neckLength/2, p.u/6),
      limits: [-Math.PI/6, Math.PI/6],
      axis: 0, flipAxis: true,
      torque: p.neckTorque,
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
      axes: [1, 0],
      torques: p.hipTorques,
    }, this.rootTransform);
    this.createDualCompoundJoint({
      id: 'r_hip',
      bodyA: this.bodies.gut, bodyB: this.bodies.r_leg_u,
      pivot: new Ammo.btVector3(p.hipRadial, p.hipHeight, 0),
      thickness: p.legUpperThickness,
      axes: [1, 0],
      torques: p.hipTorques,
    }, this.rootTransform);
    this.createHinge({
      id: 'l_knee',
      bodyA: this.bodies.l_leg_u, bodyB: this.bodies.l_leg_l,
      pivot: new Ammo.btVector3(-p.hipRadial, p.hipHeight-p.legUpperLength, 0),
      limits: p.kneeLimits,
      axis: 0, flipAxis: true,
      torque: p.kneeTorque,
    }, this.rootTransform);
    this.createHinge({
      id: 'r_knee',
      bodyA: this.bodies.r_leg_u, bodyB: this.bodies.r_leg_l,
      pivot: new Ammo.btVector3(p.hipRadial, p.hipHeight-p.legUpperLength, 0),
      limits: p.kneeLimits,
      axis: 0, flipAxis: true,
      torque: p.kneeTorque,
    }, this.rootTransform);
    this.createHinge({
      id: 'l_ankle',
      bodyA: this.bodies.l_leg_l, bodyB: this.bodies.l_foot,
      pivot: new Ammo.btVector3(-p.hipRadial, p.hipHeight-p.legUpperLength-p.legLowerLength, 0),
      limits: p.ankleLimits,
      axis: 0, flipAxis: true,
      torque: p.ankleTorque, pid: p.anklePid,
    }, this.rootTransform);
    this.createHinge({
      id: 'r_ankle',
      bodyA: this.bodies.r_leg_l, bodyB: this.bodies.r_foot,
      pivot: new Ammo.btVector3(p.hipRadial, p.hipHeight-p.legUpperLength-p.legLowerLength, 0),
      limits: p.ankleLimits,
      axis: 0, flipAxis: true,
      torque: p.ankleTorque, pid: p.anklePid,
    }, this.rootTransform);
  }

  // Returns array of rigid body descriptions to be passed to the render thread
  initSkiier(p, pinned = false) {
    this.rootTransform.setIdentity();
    this.rootTransform.setOrigin(new Ammo.btVector3(0, 4, 0));

    this.generateBodies(p);
    this.generateJoints(p);

    // setting mass=0 creates a static body - pins the character in place
    if (pinned) {
      this.setPinned = true;
      this.bodies.l_foot.makeStatic();
      this.bodies.r_foot.makeStatic();
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
    var boxShape = new Ammo.btBoxShape(new Ammo.btVector3(w/2, h/2, d/2));
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
  }

  createSki(params, transform = null) {
    this.createBox(params, transform);
    this.skiis[params.id] = new Ski(this.world, this.bodies[params.id], this.ground);
  }
}
