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

Ammo.btTransform.prototype.xform = function (pt) {
  var myInverse = this.inverse();
  return myInverse.invXform(pt);
};
