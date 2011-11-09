define([], function(){
    "use strict";

  /**
   * Convenience constructor for cpVect structs
   * @param x {Number} the x portion of the vector
   * @param y {Number} the y portion of the vector
   * @return {object} an object with the values as members (.x and .y)
   */
  var Vect = function(x, y){
    this.x = x;
    this.y = y;
  };
  
  Vect.prototype = {
    /**
     * Returns the length of v.
     * @param v {object} the vector to use
     * @return {Number} the length of the vector.
     */
    length: function(){
      // originally cpfsqrt instead of Math.sqrt. defined in chipmunk_types.h (just defined to sqrt or sqrtf)
      return Math.sqrt( this.dot(this) );
    },
    
    /**
     * Linearly interpolate between v1 and v2.
     * @param v1 {object} vector to interpolate from (source)
     * @param v2 {object} vector to interpolate to (target)
     * @param t {Number} the amount of v2 to use.
     */
    lerp: function( v2, t ){
      return this.mult(1 - t).add(v2.mult(t));
    },
     
    /**
     * Spherical linearly interpolate between v1 towards v2 by no more than angle a radians
     * @param v1 {object} vector to interpolate from (source)
     * @param v2 {object} vector to interpolate to (target)
     * @param a {Number} angle of radians to use for interpolation.
     */
    slerpconst: function( v2, a ){
      var angle = Math.acos(this.dot(v2)); 
      return this.slerp(v2, Math.min(a, angle)/angle);
    },

    
    /**
     * Returns the angular direction v is pointing in (in radians).
     * @param v the vector to use.
     * @return {object} Angular direction in radians.
     */
    toangle: function(){
      return Math.atan2(this.y, this.x);
    },
    
    /**
     * Returns a string representation of v. Intended mostly for debugging purposes and not production use.
     * @attention The original C-function reset the local field on every call, this function does not.
     * @param v {object} the vector to stringify
     * @return {string} a string representation like "(1, 0)"
     */
    str: function(){
      return "("+this.x+", "+this.y+")"; // TODO: find more elegant solution for this. Maybe grab a sprintf implementation for JS?
    },

    /**
     * Check if two vectors are equal. (Be careful when comparing floating point numbers!)
     * @param v1 {object} the first vector
     * @param v2 {object} the second vector
     * @return {boolean}
     */
    eql: function(v2){
      return (this.x === v2.x && this.y === v2.y);
    },

    /**
     * Add two vectors
     * @param v1 {object} the first vector
     * @param v2 {object} the second vector
     * @return {object} a new vector that is the sum of the two other vectors.
     */
    add: function(v2){
      return new Vect( this.x + v2.x, this.y + v2.y );
    },
    
    /**
     * Subtract two vectors.
     * @param v1 {object} the first vector
     * @param v2 {object} the second vector
     * @return {object} a new vector that is the difference of the two other vectors.
     */
    sub: function(v2){
      return new Vect(this.x - v2.x, this.y - v2.y);
    },

    /**
     * Negate a vector
     * @param v {object} the vector to negate
     * @return {object} a vector pointing in the opposite direction ( -v.x, -v.y )
     */
    neg: function(){
      return new Vect( -this.x, -this.y );
    },
    
    /**
     * Scalar multiplication
     * @param v {object} the vector to multiplicate
     * @param s {Number} the scalar factor.
     * @return {object} a new vector that is the scalar product
     */
    mult: function(s){
      return new Vect( this.x*s, this.y*s );
    },
    
    /**
     * Vector dot product
     * @param v1 {object} 
     * @param v2 {object}
     * @return {Number}
     */
    dot: function(v2){
      return this.x*v2.x + this.y*v2.y;
    },
    
    /**
     * 2D vector cross product analog.
     * The cross product of 2D vectors results in a 3D vector with only a z component.
     * This function returns the magnitude of the z value.
     * @param v1 {object} 
     * @param v2 {object}
     * @return {Number}
     */
    cross: function(v2){
      return this.x*v2.y - this.y*v2.x;
    },
    
    /**
     * Returns a perpendicular vector. (90 degree rotation)
     * @param v {object}
     * @return {object}
     */
    perp: function(){
      return new Vect( -this.y, this.x );
    },
    
    /**
     * Returns a perpendicular vector. (-90 degree rotation)
     * @param v {object}
     * @return {object}
     */
    rperp: function(){
      return new Vect(this.y, -this.x);
    },
    
    /**
     * Returns the vector projection of v1 onto v2.
     * @param v1 {object}
     * @param v2 {object}
     * @return {object}
     */
    project: function(v2){
      return v2.mult(this.dot(v2)/v2.dot(v2));
    },
    
    /**
     * Uses complex number multiplication to rotate v1 by v2. Scaling will occur if v1 is not a unit vector.
     * @param v1 {object}
     * @param v2 {object}     
     * @return {object}
     */
    rotate: function(v2){
      return new Vect(this.x*v2.x - this.y*v2.y, this.x*v2.y + this.y*v2.x);
    },
    
    /**
     * Inverse of rotate().
     * @param v1 {object}
     * @param v2 {object}     
     * @return {object}
     */
    unrotate: function(v2){
      return new Vect(this.x*v2.x + this.y*v2.y, this.y*v2.x - this.x*v2.y);
    },
    
    /**
     * Returns the squared length of v. Faster than cpvlength() when you only need to compare lengths.
     * @param v {object}
     * @return {Number}
     */
    lengthsq: function(){
      return this.dot(this);
    },
    
    
    /**
     * Returns a normalized copy of v.
     * @param v {object}
     */
    normalize: function(){
      return this.mult(1/this.length());
    },
    
    /**
     * Returns a normalized copy of v or cpvzero if v was already cpvzero. Protects against divide by zero errors.
     * @param v {object}
     * @return {object}
     */
    normalize_safe: function(){
      return (this.x === 0 && this.y === 0 ? Vect.zero : this.normalize());
    },
    
    /**
     * Clamp v to length len
     * @param v {object} 
     * @param len {Number}
     */
    clamp: function(len){
      return (this.dot(this) > len*len) ? this.normalize().mult(len) : this;
    },
    
    /**
     * Linearly interpolate between v1 towards v2 by distance d.
     * @param v1 {object}
     * @param v2 {object}
     * @param d {Number}
     */
    lerpconst: function(v2, d){
      return this.add(v2.sub(this).clamp(d));
    },
    
    /**
     * Returns the distance between v1 and v2.
     * @param v1 {object}
     * @param v2 {object}
     * @return {Number}
     */
    dist: function(v1, v2){
      return this.sub(v2).length();
    },
    
    /**
     * Returns the squared distance between v1 and v2. Faster than cpvdist() when you only need to compare distances.
     * @param v1 {object}
     * @param v2 {object}
     * @return {Number}
     */
    distsq: function(v2){
      return this.sub(v2).lengthsq();
    },
    
    /**
     * Returns true if the distance between v1 and v2 is less than dist.
     * @param v1 {object}
     * @param v2 {object}
     * @return {boolean}
     */
    near: function(v2, dist){
      return this.distsq(v2) < dist*dist;
    },
    
    /**
     * Spherical linearly interpolate between v1 and v2.
     * @param v1 {object} vector to interpolate from (source)
     * @param v2 {object} vector to interpolate to (target)
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

  Vect.zero = new Vect(0, 0);
  /**
   * Returns the unit length vector for the given angle (in radians).
   * @param a {Number} Angle for the new vector in radians.
   * @return {object} the unit length vector for this angle.
   */
  Vect.forangle = function(a){
    return new Vect(Math.cos(a), Math.sin(a));
  };


  return Vect;
});
