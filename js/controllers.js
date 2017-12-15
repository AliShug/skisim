/*jshint esversion:6*/

class SkiierController {
  constructor(skiier) {
    this.skiier = skiier;
    this.controls = {hip_lean: 0.2};
    this.controllers = skiier.jointControllers;
    this.skiis = skiier.skiis;
    this.keyframes = [];
  }

  setInput(controls) {
    this.controls = controls;
  }

  setKeyframes(keyframes) {
    this.keyframes = keyframes;
  }

  getKeyedValue(time, param) {
    var last = {time: 0, value: this.controls[param]};
    var next = {time: time+30, value: this.controls[param]};
    var keyed = false;
    for (var i = 0; i < this.keyframes.length; i++) {
      var key = this.keyframes[i];
      if (key.param === param) {
        if (key.time > last.time && key.time < time) {
          last = key;
          keyed = true;
        }
        if (key.time < next.time && key.time > time) {
          next = key;
          keyed = true;
        }
      }
    }
    if (keyed) {
      var range = next.time - last.time;
      var point = time - last.time;
      var mix = point/range;
      return (1-mix)*last.value + mix*next.value;
    }
    else {
      return this.controls[param];
    }
  }

  update(time, dt) {
    // Mixing for controls
    var squat = this.getKeyedValue(time, 'squat');
    var front_lean = this.getKeyedValue(time, 'front_lean');
    var side_lean = this.getKeyedValue(time, 'side_lean');
    var twist = (this.getKeyedValue(time, 'twist') - 0.5);
    var plough = -(this.getKeyedValue(time, 'plough') - 0.5);
    // keyboard state input
    var keys = this.controls.keys;
    if (32 in keys) { // space
      squat = 1.0;
    }
    if (65 in keys) { // A
      side_lean -= 0.2;
      twist -= 0.3;
    }
    if (68 in keys) { // D
      side_lean += 0.2;
      twist += 0.3;
    }
    if (87 in keys) { // W
      front_lean += 0.3;
    }
    if (83 in keys) { // S
      front_lean -= 0.3;
    }
    if (37 in keys) { // left
      side_lean -= 0.3;
    }
    if (39 in keys) { // right
      side_lean += 0.3;
    }
    if (38 in keys) { // up
      squat -= 0.2;
    }
    if (40 in keys) { // down
      squat += 0.6;
    }
    // joint control
    var hip_lean = 0.26 + squat*0.75 + (front_lean - 0.5) * 0.3; // forward lean at the hips
    this.controllers.l_hip_y.setTarget(hip_lean);
    this.controllers.r_hip_y.setTarget(hip_lean);
    var knees = 0.25 + 0.55*squat - (front_lean - 0.5) * 0.2; // knee flexion
    this.controllers.l_knee.setTarget(knees - twist*0.2);
    this.controllers.r_knee.setTarget(knees + twist*0.2);
    var arms_drop = 0.9 - 0.45*squat + (Math.sin(front_lean*Math.PI) - 0.5);
    this.controllers.l_shoulder_y.setTarget(arms_drop);
    this.controllers.r_shoulder_y.setTarget(arms_drop);
    var arms_inward = 0.7+0.12*squat;
    this.controllers.l_shoulder_x.setTarget(arms_inward);
    this.controllers.r_shoulder_x.setTarget(arms_inward);
    this.controllers.spine_y.setTarget(side_lean);
    var hip_twist = (twist*0.5) + 0.5;
    this.controllers.l_hip_x.setTarget(hip_twist + plough*0.5);
    this.controllers.r_hip_x.setTarget(hip_twist - plough*0.5);
    var ankles = 0.37 - 0.1*squat;
    this.controllers.l_ankle.setTarget(ankles);
    this.controllers.r_ankle.setTarget(ankles);
    var neck = 0.46 - 0.1*squat - 1.4*(front_lean - 0.5);
    this.controllers.neck.setTarget(neck);
    var back = 0.1 + front_lean*0.5;
    var elbows = (Math.sin(front_lean*Math.PI) - 0.5);
    this.controllers.spine_x.setTarget(back);
    this.controllers.l_elbow.setTarget(elbows);
    this.controllers.r_elbow.setTarget(elbows);
    // cheaty steering control
    var norm_lean = side_lean-0.5;
    for (var skiId in this.skiis) {
      var ski = this.skiis[skiId];
      var steering = -twist*0.1-norm_lean*0.1;
      var speed = this.skiier.bodies.gut.getLinearVelocity().length();
      steering /= Math.max(speed/10, 1);
      ski.vehicle.setSteeringValue(steering, 0);
      ski.vehicle.setSteeringValue(steering, 1);
    }
  }
}

// General-purpose PID control
class PIDController {
  constructor(kp, ki, kd) {
    this.integral = 0.0;
    this.last_error = 0.0;
    this.kp = kp;
    this.ki = ki;
    this.kd = kd;
  }

  setPID(kp, ki, kd) {
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
class HingeController {
  constructor (joint, maxTorque = 0.1, target = 0.0, flip=false) {
    this.target = target;
    this.joint = joint;
    this.maxTorque = maxTorque;
    this.pid = new PIDController(0.5, 0.1, 0.1);
    this.joint.enableAngularMotor(true, 0.0, maxTorque);
    this.min = -Math.PI;
    this.max = Math.PI;
    this.range = 2 * Math.PI;
    this.lastState = 0;
    if (flip) {
      this.forwardVel = 20;
      this.backVel = -20;
    }
    else {
      this.forwardVel = 20;
      this.backVel = -20;
    }
  }

  setPID(kp, ki, kd) {
    this.pid.setPID(kp, ki, kd);
  }

  setLimit(min, max, a, b, c) {
    this.min = min;
    this.max = max;
    this.range = max - min;
    this.joint.setLimit(
      min-this.range*0.1,
      max+this.range*0.1,
      a, b, c
    );
  }

  setTarget(target) {
    if (target > 1) {
      target = 1;
    }
    else if (target < 0) {
      target = 0;
    }
    this.target = target;
  }

  update(dt) {
    // PID control outputs a signed input
    // Use the input sign to direct to + or - direction of movement
    // Limit max torque by magnitude of the input
    // Make sure the torque setting takes effect by using high target velocity
    var targetAngle = this.min + this.target*this.range;
    var state = this.joint.getHingeAngle();
    var input = this.pid.getUpdate(targetAngle, state, dt);
    var impulse = Math.min(this.maxTorque, Math.abs(input));
    if (input > 0) {
      this.joint.enableAngularMotor(true, this.forwardVel, impulse);
    }
    else {
      this.joint.enableAngularMotor(true, this.backVel, impulse);
    }
    this.lastState = state;
  }
}
