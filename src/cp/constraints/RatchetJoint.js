define(['cp/constraints/Constraint', 'cp/constraints/util', 'cp/cpf'], function(Constraint, util, cpf){
  "use strict";
  
  /**
   * Works like a socket wrench. 
   * @param {cp/Body} a The first body
   * @param {cp/Body} b The second body
   * @param {Number} phase the initial offset to use when deciding where the ratchet angles are
   * @param {Number} ratchet the distance between “clicks”
   */
  var RatchetJoint = function(a, b, phase, ratchet){
    Constraint.call(this, a, b);
    this.angle = 0;
    this.phase = phase;
    this.ratchet = ratchet;
    
    // STATIC_BODY_CHECK
    this.angle = (b? b.a : 0 ) - (a? a.a : 0 );
  };

  RatchetJoint.prototype = util.extend( new Constraint(), {
    
    /**
     * @param {Number} dt
     */
    preStep: function(dt){
      var a = this.a;
      var b = this.b;
      
      var angle = this.angle;
      var phase = this.phase;
      var ratchet = this.ratchet;
      
      var delta = b.a - a.a;
      var diff = angle - delta;
      var pdist = 0;

      
      if( diff * ratchet > 0 ){
        pdist = diff;
      } else {
        this.angle = Math.floor( (delta - phase)/ratchet ) * ratchet + phase;
      }
      
      // calculate moment of inertia coefficient.
      this.iSum = 1 / (a.i_inv + b.i_inv);
      
      // calculate bias velocity
      var maxBias = this.maxBias;
      this.bias = cpf.clamp( -util.bias_coef(this.errorBias, dt)*pdist/dt, -maxBias, maxBias );
      
      // compute max impulse
      this.jMax = this.maxForce * dt;

      // If the bias is 0, the joint is not at a limit. Reset the impulse.
      if( !this.bias ){
        this.jAcc = 0;
      }  
    },
    
    /**
     * @param {Number} dt_coef
     */
    applyCachedImpulse: function(dt_coef){
      var a = this.a;
      var b = this.b;
      
      var j = this.jAcc * dt_coef;
      a.w -= j * a.i_inv;
      b.w += j * b.i_inv;
    },
    
    applyImpulse: function(){
      if( !this.bias ){
        return;
      }
      
      var a = this.a;
      var b = this.b;

      // compute relative rotational velocity
      var wr = b.w - a.w;
      var ratchet = this.ratchet;
      
      // compute normal impulse	
      var j = -(this.bias + wr) * this.iSum;
      var jOld = this.jAcc;
      
      this.jAcc = cpf.clamp( (jOld + j)*ratchet, 0, this.jMax * Math.abs(ratchet)) / ratchet;
      j = this.jAcc - jOld;
      
      // apply impulse
      a.w -= j * a.i_inv;
      b.w += j * b.i_inv;
    },
    
    /**
     * @returns {Number}
     */
    getImpulse: function(){
      return Math.abs(this.jAcc);
    },
    
    getAngle: function(){
      return this.angle;
    },
    setAngle: function(value){
      this.angle = value;
    },
    getPhase: function(){
      return this.phase;
    },
    setPhase: function(value){
      this.phase = value;
    },
    getRatchet: function(){
      return this.ratchet;
    },
    setRatchet: function(value){
      this.ratchet = value;
    }

  });
  
  return RatchetJoint;
});
