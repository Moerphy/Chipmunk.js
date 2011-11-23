define(['cp/constraints/Constraint', 'cp/constraints/util', 'cp/cpf'], function(Constraint, util, cpf){
  "use strict";
  
  /**
   * @param {cp/Body} a the first body to connect
   * @param {cp/Body} b the second body to connect
   * @param {cp/Vect} anchr1 the anchor point on the first body
   * @param {cp/Vect} anchr2 the anchor point on the second body
   * @param {Number} restLength the length the spring wants to be
   * @param {Number} stiffness is the spring constant (Young's modulus)
   * @param {Number} damping how soft to make the damping of the spring.
   */
  var DampedSpring = function(a, b, anchr1, anchr2, restLength, stiffness, damping){
    Constraint.call(this, a, b);
    this.anchr1 = anchr1;
    this.anchr2 = anchr2;
    
    this.restLength = restLength;
    this.stiffness = stiffness;
    this.damping = damping;
    // this.springForceFunc = defaultSpringForce
  };
  
  DampedSpring.prototype = util.extend( new Constraint(), {
    
    /**
     * @param {Number} dt
     */
    preStep: function(dt){   
      var a = this.a;
      var b = this.b;
      
      this.r1 = this.anchr1.rotate(a.rot);
      this.r2 = this.anchr2.rotate(b.rot);
      
      var delta = b.p.add(this.r2).sub( a.p.add(this.r1) );
      var dist = delta.length();
      
      this.n = delta.mult( 1/(dist?dist:Number.POSITIVE_INFINITY) );
      var k = util.k_scalar(a, b, this.r1, this.r2, this.n );
      
      // assert.soft( k != 0, "Unsolvable spring." );
      this.nMass = 1 / k;
      this.target_vrn = 0;
      
      this.v_coef = 1 - Math.exp( -this.damping * dt * k );
      
      // apply spring force
      var f_spring = this.springForceFunc(dist);
      util.apply_impulses( a, b, this.r1, this.r2, this.n.mult(f_spring*dt) );
    },
    
    /**
     * @param {Number} dt_coef
     */
    applyCachedImpulse: function(dt_coef){},
    
    applyImpulse: function(){
      var a = this.a;
      var b = this.b;
      
      var n = this.n;
      var r1 = this.r1;
      var r2 = this.r2;
      
      // compute realtive velocity
      var vrn = util.normal_relative_velocity(a, b, r1, r2, n);
      
      // compute velocity loss from drag
      var v_damp = (this.target_vrn - vrn) * this.v_coef;
      this.target_vrn = vrn + v_damp;
      
      util.apply_impulses( a, b, this.r1, this.r2, this.n.mult( v_damp * this.nMass ) );
    },
    
    getImpulse: function(){
      return 0;
    },
   
    /**
     * @param {Number} dist
     */
    springForceFunc: function(dist){
      return (this.restLength - dist)*this.stiffness;
    },
    
    getAnchr1: function(){
      return this.anchr1;
    },
    setAnchr1: function(a){
      this.anchr1 = a;
    },
    getAnchr2: function(){
      return this.anchr2;
    },
    setAnchr2: function(a){
      this.anchr2 = a;
    },
    getRestLength: function(){
      return this.restLength;
    },
    setRestLength: function(value){
      this.restLength = value;
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
  
  return DampedSpring;
});
