define(['cp/constraints/Constraint', 'cp/constraints/util', 'cp/cpf'], function(Constraint, util, cpf){
  "use strict";
  
  /**
   * @param {cp/Body} a
   * @param {cp/Body} b
   * @param {Number} rate
   */
  var SimpleMotor = function(a, b, rate){
    Constraint.call(this, a, b);
    this.rate = rate;
    this.jAcc = 0;
  };
  
  SimpleMotor.prototype = util.extend( new Constraint(), {
    
    /**
     * @param {Number} dt
     */
    preStep: function(dt){
      var a = this.a;
      var b = this.b;
      
      // calculate moment of inertia coefficient.
      this.iSum = 1/(a.i_inv + b.i_inv);
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
      a.w -= j*a.i_inv;
      b.w += j*b.i_inv;
    },
    
    applyImpulse: function(){
      var a = this.a;
      var b = this.b;
      
      // compute relative rotational velocity
      var wr = b.w - a.w + this.rate;
      
      // compute normal impulse
      var j = -wr * this.iSum;
      var jOld = this.jAcc;
      this.jAcc = cpf.clamp( jOld + j, -this.jMax, this.jMax );
      j = this.jAcc - jOld;
      
      // apply impulse
      a.w -= j*a.i_inv;
      b.w += j*b.i_inv;
    },
    
    getImpulse: function(){
      return Math.abs( this.jAcc );
    },
    
    getRate: function(){
      return this.rate;
    },
    
    setRate: function(rate){
      this.rate = rate;
    }

  });
  
  return SimpleMotor;
});
