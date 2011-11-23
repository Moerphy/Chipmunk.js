define(['cp/constraints/Constraint', 'cp/constraints/util', 'cp/cpf'], function(Constraint, util, cpf){
  "use strict";
  
  /**
   * @param {cp/Body} a the first body to connect
   * @param {cp/Body} b the second body to connect
   * @param {Number} restAngle the relative angle in radians that the bodies want to have
   * @param {Number} stiffness is the spring constant (Young's modulus)
   * @param {Number} damping how soft to make the damping of the spring.
   */
  var DampedRotarySpring = function(a, b, restAngle, stiffness, damping){
    Constraint.call(this, a, b);

    this.restAngle = restAngle;
    this.stiffness = stiffness;
    this.damping = damping;
  };
  
  DampedRotarySpring.prototype = util.extend( new Constraint(), {
    
    /**
     * @param {Number} dt
     */
    preStep: function(dt){   
      var a = this.a;
      var b = this.b;
      
      var moment = a.i_inv + b.i_inv;
      // assert.soft( moment !== 0, "Unsolveable spring.");
      this.iSum = 1 / moment;
      
      this.w_coef = 1 - Math.exp( -this.damping * dt * moment );
      this.target_wrn = 0;
      
      // apply spring torque
      var j_spring = this.springTorqueFunc(a.a - b.a) * dt;
      a.w -= j_spring * a.i_inv;
      b.w += j_spring * b.i_inv;
    },
    
    /**
     * @param {Number} dt_coef
     */
    applyCachedImpulse: function(dt_coef){},
    
    applyImpulse: function(){
      var a = this.a;
      var b = this.b;
      // compute relative velocity
      var wrn = a.w - b.w; 

      // compute velocity loss from drag
      // not 100% certain this is derived correctly, though it makes sense
      var w_damp = (this.target_wrn - wrn) * this.w_coef;
      this.target_wrn = wrn + w_damp;
      
      var j_damp = w_damp * this.iSum;
      a.w += j_damp * a.i_inv;
      a.w -= j_damp * b.i_inv;
    },
    
    getImpulse: function(){
      return 0;
    },
   
    /**
     * @param {Number} dist
     */
    springTorqueFunc: function(relativeAngle){
      return (relativeAngle - this.restAngle)*this.stiffness;
    },
    
    getRestAngle: function(){
      return this.restAngle;
    },
    setRestAngle: function(value){
      this.restAngle = value;
    },
    getStiffness: function(){
      return this.stiffness;
    },
    setStiffness: function(value){
      this.stiffness = value;
    },
    getDamping: function(){
      return this.restLength;
    },
    setDamping: function(value){
      this.damping = value;
    }
  });
  
  return DampedRotarySpring;
});
