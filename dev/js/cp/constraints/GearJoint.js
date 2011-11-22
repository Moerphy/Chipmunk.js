define(['cp/constraints/Constraint', 'cp/constraints/util', 'cp/cpf'], function(Constraint, util, cpf){
  "use strict";
  
  /**
   * Keeps the angular velocity ratio of a pair of bodies constant. 
   * @param {cp/Body} a
   * @param {cp/Body} b
   * @param {Number} phase the initial angular offset of the two bodies.
   * @param {Number} ratio The ratio of a.getAngVel() / b.getAngVel() that should be kept constant. Always measured in absolute terms
   */
  var GearJoint = function(a, b, phase, ratio){
    Constraint.call(this, a, b);
    this.phase = phase;
    this.ratio = ratio;
    this.ratio_inv = 1 / ratio;
    
    this.jAcc = 0;
  };
  
  GearJoint.prototype = util.extend( new Constraint(), {
    
    /**
     * @param {Number} dt
     */
    preStep: function(dt){
      var a = this.a;
      var b = this.b;
      
      // calculate moment of inertia coefficient.
      this.iSum = 1 / (a.i_inv * this.ratio_inv + this.ratio * b.i_inv);

      // calculate bias velocity
      var maxBias = this.maxBias;
      this.bias = cpf.clamp( -util.bias_coef(this.errorBias, dt)*(b.a*this.ratio - a.a - this.phase)/dt, -maxBias, maxBias );

      // compute max impulse
      this.jMax = this.maxForce * dt;
    },
    
    /**
     * @param {Number} dt_coef
     */
    applyCachedImpulse: function(dt_coef){
      var a = this.a;
      var b = this.b;
      
      var j = this.jAcc * dt_coef;
      a.w -= j * a.i_inv * this.ratio_inv;
      b.w += j * b.i_inv;
    },
    
    applyImpulse: function(){
      var a = this.a;
      var b = this.b;
      
      // compute relative rotational velocity
      var wr = b.w * this.ratio - a.w;
      
      // compute normal impulse	
      var j = (this.bias - wr) * this.iSum;
      var jOld = this.jAcc;
      this.jAcc = cpf.clamp( jOld + j, -this.jMax, this.jMax);
      j = this.jAcc - jOld;
      
      // apply impulse
      a.w -= j * a.i_inv * this.ratio_inv;
      b.w += j * b.i_inv
    },
    
    /**
     * @returns {Number}
     */
    getImpulse: function(){
      return Math.abs(this.jAcc);
    },
   
    getRatio: function(){
      return this.ratio;
    },
    setRatio: function(value){
      this.ratio = value;
      this.ratio_inv = 1 / value;
      
      this.activateBodies();
    },
    getPhase: function(){
      return this.phase;
    },
    setPhase: function(value){
      this.phase = value;
    }
   
  });
  
  return GearJoint;
});
