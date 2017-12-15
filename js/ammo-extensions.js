/*jshint esversion: 6*/

// Convenience
Ammo.btVector3.prototype.clone = function () {
  var out = new Ammo.btVector3(this.x(), this.y(), this.z());
  return out;
};

Ammo.btVector3.prototype.copy = function (v) {
  this.setX(v.x());
  this.setY(v.y());
  this.setZ(v.z());
};

Ammo.btVector3.prototype.prettyPrint = function () {
  var x = this.x(), y = this.y(), z = this.z();
  console.log(`<${x}, ${y}, ${z}>`);
};

Ammo.btTransform.prototype.xform = function (pt, dir=false) {
  var myInverse = this.inverse();
  var result = myInverse.invXform(pt);
  Ammo.destroy(myInverse);
  if (dir) {
    result.op_sub(this.getOrigin());
  }
  return result;
};

Ammo.btTransform.prototype.clone = function () {
  var out = new Ammo.btTransform();
  out.op_eq(this);
  return out;
};

Ammo.btTransform.prototype.copy = function (other) {
  return this.op_eq(other);
};

Ammo.btRigidBody.prototype.startMass = 0.0;
Ammo.btRigidBody.prototype.startInertia = null;

Ammo.btRigidBody.prototype.makeStatic = function () {
  var zeroes = new Ammo.btVector3(0, 0, 0);
  this.setMassProps(0, zeroes);
  Ammo.destroy(zeroes);
};

Ammo.btRigidBody.prototype.applyLocalForce = function (force, pos) {
  var transform = this.getCenterOfMassTransform();
  var worldForce = transform.xform(force, true).clone();
  var relPos = transform.xform(pos, true).clone();
  this.applyForce(worldForce, relPos);
  Ammo.destroy(worldForce);
  Ammo.destroy(relPos);
};

Ammo.btRigidBody.prototype.makeDynamic = function () {
  this.setMassProps(this.startMass, this.startInertia);
};
