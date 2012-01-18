define(['cp/Vect', 'cp/constraints/Constraint', 'cp/constraints/util', 'cp/cpf'], function(Vect, Constraint, util, cpf){
  "use strict";
  
  /**
   * Keeps the angular velocity ratio of a pair of bodies constant. 
   * @param {cp/Body} a
   * @param {cp/Body} b
   * @param {Number} phase the initial angular offset of the two bodies.
   * @param {Number} ratio The ratio of a.getAngVel() / b.getAngVel() that should be kept constant. Always measured in absolute terms
   */
  var PivotJoint = function(a, b, anchr1, anchr2){
    Constraint.call(this, a, b);
    if( !anchr2 ){ // is cpPivotJointNew(3), not cpPivotJointNew2(4)
      var pivot = anchr1;
      anchr1 = (a ? a.world2Local(pivot):pivot);
      anchr2 = (b ? b.world2Local(pivot):pivot);
    }
    
    this.anchr1 = anchr1;
    this.anchr2 = anchr2;
    
    this.jAcc = Vect.zero;
  };
  
  PivotJoint.prototype = util.extend( new Constraint(), {
    
    /**
     * @param {Number} dt
     */
    preStep: function(dt){ 
      var a = this.a;
      var b = this.b;
      
      this.r1 = this.anchr1.rotate(a.rot);
      this.r2 = this.anchr2.rotate(b.rot);

      // Calculate mass tensor
      util.k_tensor( a, b, this.r1, this.r2, this ); // passes this instead of adresses of k1 and k2, k_tensor only set .k1 .k2 fields on this object
      // compute max impulse
      this.jMaxLen = this.maxForce * dt;

      // calculate bias velocity
      var delta = b.p.add(this.r2).sub( a.p.add(this.r1) );
      this.bias = delta.mult( - util.bias_coef(this.errorBias, dt) / dt ).clamp(this.maxBias );
    },
    
    /**
     * @param {Number} dt_coef
     */
    applyCachedImpulse: function(dt_coef){ 
      util.apply_impulses( this.a, this.b, this.r1,this.r2, this.jAcc.mult(dt_coef) );
    },
    
    applyImpulse: function(){ 
      var a = this.a;
      var b = this.b;
      
      var r1 = this.r1;
      var r2 = this.r2;
      // compute relative velocity
      var vr = util.relative_velocity(a, b, r1, r2);
      
      // compute normal impulse
      var j = util.mult_k( this.bias.sub(vr), this.k1, this.k2 );
      var jOld = this.jAcc;
      this.jAcc = this.jAcc.add(j).clamp(this.jMaxLen);
      j = this.jAcc.sub(jOld);

      // apply impulse
      util.apply_impulses(a, b, this.r1, this.r2, j);
    },
    
    /**
     * @returns {Number}
     */
    getImpulse: function(){  
      this.jAcc.length();
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
    }
   
  });
  
  return PivotJoint;
});
