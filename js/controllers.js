/*jshint esversion:6*/

class SkiierController {
  constructor(skiier) {
    this.skiier = skiier;
    this.controls = {};
    this.controllers = skiier.jointControllers;
  }

  setInput(controls) {
    this.controls = controls;
  }

  update(dt) {
    this.controllers.l_hip_x.setTarget(controls.l_hip_x);
    this.controllers.r_hip_x.setTarget(controls.r_hip_x);
    this.controllers.l_hip_y.setTarget(controls.l_hip_y);
    this.controllers.r_hip_y.setTarget(controls.r_hip_y);
    this.controllers.l_knee.setTarget(controls.l_knee);
    this.controllers.r_knee.setTarget(controls.r_knee);
    this.controllers.l_ankle.setTarget(controls.l_ankle);
    this.controllers.r_ankle.setTarget(controls.r_ankle);
    this.controllers.neck.setTarget(controls.neck);
    this.controllers.spine_x.setTarget(controls.spine);
    this.controllers.l_elbow.setTarget(controls.l_elbow);
    this.controllers.r_elbow.setTarget(controls.r_elbow);
    this.controllers.l_shoulder_x.setTarget(controls.l_shoulder_x);
    this.controllers.l_shoulder_y.setTarget(controls.l_shoulder_y);
    this.controllers.r_shoulder_x.setTarget(controls.r_shoulder_x);
    this.controllers.r_shoulder_y.setTarget(controls.r_shoulder_y);
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
    this.joint.setLimit(min, max, a, b, c);
  }

  setTarget(target) {
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
