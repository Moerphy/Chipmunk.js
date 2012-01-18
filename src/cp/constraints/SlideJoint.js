define(['cp/constraints/Constraint', 'cp/constraints/util', 'cp/cpf'], function(Constraint, util, cpf){
  "use strict";
  
  /**
   * @param {cp/Body} a
   * @param {cp/Body} b
   * @param {cp/Vect} anchr1
   * @param {cp/Vect} anchr2
   * @param {Number} min
   * @param {Number} max
   */
  var SlideJoint = function(a, b, anchr1, anchr2, min, max){
    Constraint.call(this, a, b);
    this.anchr1 = anchr1;
    this.anchr2 = anchr2;
    this.min = min;
    this.max = max;
    this.jnAcc = 0;
  };
  
  SlideJoint.prototype = util.extend( new Constraint(), {
    
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
      
      var pdist = 0;
      if( dist > this.max ){
        pdist = dist - this.max;
        this.n = delta.normalize_safe();
      }else if( dist < this.min ){
        pdist = this.min - dist;
      }else{
        this.n = Vect.zero;
        this.jnAcc = 0;
      }
      
      // calculate mass normal
      this.nMass = 1 / util.k_scalar( a, b, this.r1, this.r2, this.n );
      
      // calculate bias velocity
      var maxBias = this.maxBias;
      this.bias = cpf.clamp( -util.bias_coef(this.errorBias, dt)*pdist/dt, -maxBias, maxBias );
      
      // compute max impulse
      this.jnMax = this.maxForce * dt;
    },
    
    /**
     * @param {Number} dt_coef
     */
    applyCachedImpulse: function(dt_coef){    
      var a = this.a;
      var b = this.b;
      
      var j = this.n.mult( this.jnAcc * dt_coef );
      util.apply_impulses(a, b, this.r1, this.r2, j);
    },
    
    applyImpulse: function(){  
      if(this.n.eql(cpvzero)){
        return; // early exit
      }
      
      var a = this.a;
      var b = this.b;
      
      var n = this.n;
      var r1 = this.r1;
      var r2 = this.r2;
      
      // compute relative velocity
      var vr = util.relative_velocity(a, b, r1, r2);
      var vrn = vr.dot(n);
      
      // compute normal impulse
      var jn = (this.bias - vrn) * this.nMass;
      var jnOld = this.jnAcc;
      
      this.jnAcc = cpf.clamp(jnOld + jn, -this.jnMax, 0 );
      jn = this.jnAcc - jnOld;
      
      // apply impulse
      util.apply_impulses( a, b, this.r1, this.r2, n.mult(jn) );
    },
    
    getImpulse: function(){    
      return Math.abs( this.jnAcc );
    },
    
    getAnchr1: function(){
      return this.anchr1;
    },
    
    setAnchr1: function(anchr){
      this.anchr1 = anchr;
    },

    getAnchr2: function(){
      return this.anchr2;
    },
    
    setAnchr2: function(anchr){
      this.anchr2 = anchr;
    },

    getMin: function(){
      return this.min;
    },
    
    setMin: function(m){
      this.min = m;
    },
    
    getMax: function(){
      return this.max;
    },
    
    setMax: function(m){
      this.max = m;
    }

  });
  
  return SlideJoint;
});
