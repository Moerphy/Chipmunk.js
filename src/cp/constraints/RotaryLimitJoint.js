define(['cp/constraints/Constraint', 'cp/constraints/util', 'cp/cpf'], function(Constraint, util, cpf){
  "use strict";
  
  /**
   * Constrains the relative rotations of two bodies. min and max are the angular limits in radians. 
   * It is implemented so that it’s possible to for the range to be greater than a full revolution.
   * @param {cp/Body} a The first body
   * @param {cp/Body} b The second body
   * @param {Number} min the minimum angular difference in radians
   * @param {Number} max the maximum angular difference in radians
   */
  var RotaryLimitJoint = function(a, b, min, max){
    Constraint.call(this, a, b);
    
    this.min = min;
    this.max = max;
    
    this.jAcc = 0;
  };

  RotaryLimitJoint.prototype = util.extend( new Constraint(), {
    
    /**
     * @param {Number} dt
     */
    preStep: function(dt){
      var a = this.a;
      var b = this.b;
      
      var dist = b.a - a.a;
      var pdist = 0;
      
      if( dist > this.max ){
        pdist = this.max - dist;
      }else if( dist < this.min ){
        pdist = this.min - dist;
      }
     
      // calculate moment of inertia coefficient.
      this.iSum = 1 / (1/a.i + 1/b.i);
      
      // calculate bias velocity
      var maxBias = this.maxBias;
      this.bias = cpf.clamp( -util.bias_coef(this.errorBias, dt) * pdist / dt, -maxBias, maxBias );

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
      
      // compute normal impulse	
      var j = -(this.bias + wr) * this.iSum;
      var jOld = this.jAcc;
      if( this.bias < 0 ){
        this.jAcc = cpf.clamp(jOld + j, 0, this.jMax);
      }else{
        this.jAcc = cpf.clamp( jOld + j, -this.jMax, 0);
      }
      j = this.jAcc - jOld;
      
      // apply impulse
      a.w -= j * a.i_inv;
      b.w += j * b.i_inv;
    },
    
    /**
     * @returns {Number}
     */
    getImpulse: function(){
      Math.abs(this.jAcc);
    },
    
    getMin: function(){
      return this.min;
    },
    setMin: function(value){
      this.min = value;
    },
    getMax: function(){
      return this.max;
    },
    setMax: function(value){
      this.max = value;
    }
    

  });
  
  return RotaryLimitJoint;
});
