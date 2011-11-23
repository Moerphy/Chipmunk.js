define([], function(){
    "use strict";

  /**
   * Convenience constructor for cpVect structs
   * @param {Number} x the x portion of the vector
   * @param {Number} y the y portion of the vector
   * @return {cp/Vect} an object with the values as members (.x and .y)
   */
  var Vect = function(x, y){
    this.x = x;
    this.y = y;
  };
  
  Vect.prototype = {
    /**
     * Returns the length of v.
     * @return {Number} the length of the vector.
     */
    length: function(){
      // originally cpfsqrt instead of Math.sqrt. defined in chipmunk_types.h (just defined to sqrt or sqrtf)
      return Math.sqrt( this.dot(this) );
    },
    
    /**
     * Linearly interpolate between v1 and v2.
     * @param {cp/Vect} v2 vector to interpolate to (target)
     * @param {Number} the amount of v2 to use.
     */
    lerp: function( v2, t ){
      return this.mult(1 - t).add(v2.mult(t));
    },
     
    /**
     * Spherical linearly interpolate between v1 towards v2 by no more than angle a radians
     * @param {cp/Vect} v2 vector to interpolate to (target)
     * @param {Number} angle of radians to use for interpolation.
     */
    slerpconst: function( v2, a ){
      var angle = Math.acos(this.dot(v2)); 
      return this.slerp(v2, Math.min(a, angle)/angle);
    },

    
    /**
     * Returns the angular direction v is pointing in (in radians).
     * @return {Number} Angular direction in radians.
     */
    toangle: function(){
      return Math.atan2(this.y, this.x);
    },
    
    /**
     * Returns a string representation of v. Intended mostly for debugging purposes and not production use.
     * @attention The original C-function reset the local field on every call, this function does not.
     * @return {String} a string representation like "(1, 0)"
     */
    str: function(){
      return "("+this.x+", "+this.y+")";
    },

    /**
     * Check if two vectors are equal. (Be careful when comparing floating point numbers!)
     * @param {cp/Vect} v2 the second vector
     * @return {boolean}
     */
    eql: function(v2){
      return (this.x === v2.x && this.y === v2.y);
    },

    /**
     * Add two vectors
     * @param {cp/Vect} v2 the second vector
     * @return {cp/Vect} a new vector that is the sum of the two other vectors.
     */
    add: function(v2){
      return new Vect( this.x + v2.x, this.y + v2.y );
    },
    
    /**
     * Subtract two vectors.
     * @param {cp/Vect} v2 the second vector
     * @return {cp/Vect} a new vector that is the difference of the two other vectors.
     */
    sub: function(v2){
      return new Vect(this.x - v2.x, this.y - v2.y);
    },

    /**
     * Negate a vector
     * @return {cp/Vect} a vector pointing in the opposite direction ( -v.x, -v.y )
     */
    neg: function(){
      return new Vect( -this.x, -this.y );
    },
    
    /**
     * Scalar multiplication
     * @param s {Number} the scalar factor.
     * @return {cp/Vect} a new vector that is the scalar product
     */
    mult: function(s){
      return new Vect( this.x*s, this.y*s );
    },
    
    /**
     * Vector dot product
     * @param {cp/Vect} v2
     * @return {Number}
     */
    dot: function(v2){
      return this.x*v2.x + this.y*v2.y;
    },
    
    /**
     * 2D vector cross product analog.
     * The cross product of 2D vectors results in a 3D vector with only a z component.
     * This function returns the magnitude of the z value.
     * @param {cp/Vect} v2
     * @return {Number}
     */
    cross: function(v2){
      return this.x*v2.y - this.y*v2.x;
    },
    
    /**
     * Returns a perpendicular vector. (90 degree rotation)
     * @return {cp/Vect}
     */
    perp: function(){
      return new Vect( -this.y, this.x );
    },
    
    /**
     * Returns a perpendicular vector. (-90 degree rotation)
     * @param {cp/Vect}
     * @return {cp/Vect}
     */
    rperp: function(){
      return new Vect(this.y, -this.x);
    },
    
    /**
     * Returns the vector projection of this onto v2.
     * @param {cp/Vect} v2
     * @return {cp/Vect}
     */
    project: function(v2){
      return v2.mult(this.dot(v2)/v2.dot(v2));
    },
    
    /**
     * Uses complex number multiplication to rotate this by v2. Scaling will occur if v1 is not a unit vector.
     * @param {cp/Vect} v2
     * @return {cp/Vect}
     */
    rotate: function(v2){
      return new Vect(this.x*v2.x - this.y*v2.y, this.x*v2.y + this.y*v2.x);
    },
    
    /**
     * Inverse of rotate().
     * @param {cp/Vect}  v2
     * @return {cp/Vect}
     */
    unrotate: function(v2){
      return new Vect(this.x*v2.x + this.y*v2.y, this.y*v2.x - this.x*v2.y);
    },
    
    /**
     * Returns the squared length of this. Faster than cpvlength() when you only need to compare lengths.
     * @return {Number}
     */
    lengthsq: function(){
      return this.dot(this);
    },
    
    
    /**
     * Returns a normalized copy of v.
     * @return {cp/Vect}
     */
    normalize: function(){
      return this.mult(1/this.length());
    },
    
    /**
     * Returns a normalized copy of v or cpvzero if v was already cpvzero. Protects against divide by zero errors.
     * @return {cp/Vect}
     */
    normalize_safe: function(){
      return (this.x === 0 && this.y === 0 ? Vect.zero : this.normalize());
    },
    
    /**
     * Clamp this to length len
     * @param {Number} len
     */
    clamp: function(len){
      return (this.dot(this) > len*len) ? this.normalize().mult(len) : this;
    },
    
    /**
     * Linearly interpolate between v1 towards v2 by distance d.
     * @param {cp/Vect} v2
     * @param d {Number}
     */
    lerpconst: function(v2, d){
      return this.add(v2.sub(this).clamp(d));
    },
    
    /**
     * Returns the distance between v1 and v2.
     * @param {cp/Vect} v2
     * @return {Number}
     */
    dist: function(v1, v2){
      return this.sub(v2).length();
    },
    
    /**
     * Returns the squared distance between v1 and v2. Faster than cpvdist() when you only need to compare distances.
     * @param {cp/Vect} v2
     * @return {Number}
     */
    distsq: function(v2){
      return this.sub(v2).lengthsq();
    },
    
    /**
     * Returns true if the distance between v1 and v2 is less than dist.
     * @param {cp/Vect} v2
     * @return {boolean}
     */
    near: function(v2, dist){
      return this.distsq(v2) < dist*dist;
    },
    
    /**
     * Spherical linearly interpolate between v1 (this) and v2.
     * @param {cp/Vect} v2 vector to interpolate to (target)
     * @param t {Number} 
     */
    cpvslerp: function(v2, t){
      var omega = Math.acos(this.dot(v2));
      
      if(omega !== undefined){ // TODO: unnecessary in JS?
        var denom = 1/Math.sin(omega);
        return this.mult(Math.sin((1 - t)*omega)*denom).add( v2.mult(Math.sin(t*omega)*denom) );
      } else {
        return this;
      }
    },
    
    toString: function(){
      return '('+this.x + '/'+ this.y + ')';
    }
  };
  
  /**
   * The (0,0) Vector
   */
  Vect.zero = new Vect(0, 0);
  /**
   * Returns the unit length vector for the given angle (in radians).
   * @param {Number} a Angle for the new vector in radians.
   * @return {cp/Vect} the unit length vector for this angle.
   */
  Vect.forangle = function(a){
    return new Vect(Math.cos(a), Math.sin(a));
  };


  return Vect;
});
