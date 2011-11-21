define(['cp/constraints/Constraint', 'cp/constraints/util', 'cp/cpf'], function(Constraint, util, cpf){
  "use strict";
  
  /**
   * @param {cp/Body} a
   * @param {cp/Body} b
   * @param {cp/Vect} anchr1
   * @param {cp/Vect} anchr2
   */
  var PinJoint = function(a, b, anchr1, anchr2){
    Constraint.call(this, a, b);
    this.anchr1 = anchr1;
    this.anchr2 = anchr2;
    
    // STATIC_BODY_CHECK
    var p1 = (a ? a.p.add( anchr1.rotate(a.rot) ) : anchr1 );
    var p2 = (b ? b.p.add( anchr2.rotate(b.rot) ) : anchr2 );
    
    this.dist = p2.sub(p1).length();
    
    this.jnAcc = 0;
    this.jnMax = 0;
    
    this.r1 = undefined;
    this.r2 = undefined;
    this.n = undefined;
    this.nMass = 0;
    
    this.bias = 0;
  };
  
  PinJoint.prototype = util.extend( new Constraint(), {
    
    /**
     * @param {Number} dt
     */
    preStep: function(dt){
      var a = this.a;
      var b = this.b;
      
      this.r1 = this.anchr1.rotate(a.rot);
      this.r2 = this.anchr2.rotate(b.rot);
      
      var delta = b.p.add(this.r2).sub(a.p.add(this.r1));
      var dist = delta.length();
      this.n = delta.mult( 1 / (dist?dist:Number.POSITIVE_INFINITY) );
      
      // calculate mass normal
      this.nMass = 1 / util.k_scalar( a, b, this.r1, this.r2, this.n );
      
      // calculate bias velocity
      var maxBias = this.maxBias;
      this.bias = cpf.clamp( -util.bias_coef(this.errorBias, dt)*(dist-this.dist)/dt, -maxBias, maxBias );
      
      // compute max impulse
      this.jnMax = this.maxForce*dt; 
    },
    
    /**
     * @param {Number} dt_coef
     */
    applyCachedImpulse: function(dt_coef){
      var a = this.a;
      var b = this.b;
      
      var j = this.n.mult( this.jnAcc*dt_coef );
      util.apply_impulses( a, b, this.r1, this.r2, j );
    },
    
    applyImpulse: function(){
      var a = this.a;
      var b = this.b;
      
      var n = this.n;
      
      // compute relative velocity
      var vrn = util.normal_relative_velocity( a, b, this.r1, this.r2, n );
      
      // compute normal impulse
      var jn = (this.bias - vrn) * this.nMass;
      var jnOld = this.jnAcc;
      
      this.jnAcc = cpf.clamp( jnOld + jn, -this.jnMax, this.jnMax );
      jn = this.jnAcc - jnOld;
      
      // apply impulse
      util.apply_impulses( a, b, this.r1, this.r2, n.mult(jn) );
    },
    
    /**
     * @returns {Number}
     */
    getImpulse: function(){
      return Math.abs(this.jnAcc);
    },
    
    getAnchr1: function(){
      return this.anchr1;
    },
    setAnchr1: function(value){
      this.activateBodies();
      this.anchr1 = value;
    },
    getAnchr2: function(){
      return this.anchr2;
    },
    setAnchr2: function(value){
      this.activateBodies();
      this.anchr2 = value;
    },
    getDist: function(){
      return this.dist;
    },
    setDist: function(value){
      this.activateBodies();
      this.dist = value;
    }
  });
  
  return PinJoint;
});
