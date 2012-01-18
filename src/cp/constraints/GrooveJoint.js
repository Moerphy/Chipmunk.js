define(['cp/Vect', 'cp/constraints/Constraint', 'cp/constraints/util', 'cp/cpf'], function(Vect, Constraint, util, cpf){
  "use strict";
  
  /**
   * The groove goes from groov_a to groove_b on body a, and the pivot is attached to anchr2 on body b. 
   * All coordinates are body local.
   * @param {cp/Body} a The first body (groove will be attached to this body)
   * @param {cp/Body} b The second body
   * @param {cp/Vect} groove_a one point of the groove
   * @param {cp/Vect} groove_b the second point of the groove
   * @param {cp/Vect} anchr2 the point on body b that the pivot will be attached to
   */
  var GrooveJoint = function(a, b, groove_a, groove_b, anchr2){
    Constraint.call(this, a, b);

    this.grv_a = groove_a;
    this.grv_b = groove_b;
    this.grv_n = groove_b.sub(groove_a).normalize().perp();
    this.anchr2 = anchr2;
    
    this.jAcc = Vect.zero;
  };

  GrooveJoint.prototype = util.extend( new Constraint(), {
    
    /**
     * @param {Number} dt
     */
    preStep: function(dt){
      var a = this.a;
      var b = this.b;

      // calculate endpoints in worldspace
      var ta = a.local2World(this.grv_a);
      var tb = a.local2World(this.grv_b);

      // calculate axis
      var n = this.grv_n.rotate(a.rot);
      var d = ta.dot(n);
      
      this.grv_tn = n;
      this.r2 = this.anchr2.rotate(b.rot);
      
      // calculate tangential distance along the axis of r2
      var td = b.p.add(this.r2).cross(n);
      // calculate clamping factor and r2
      if( td <= ta.cross(n) ){
        this.clamp = 1;
        this.r1 = ta.sub(a.p);
      } else if( td >= tb.cross(n) ){
        this.clamp = -1;
        this.r1 = tb.sub(a.p);
      } else {
        this.clamp = 0;
        this.r1 = n.perp().mult(-td).add( n.mult(d) ).sub(a.p);
      }

      // Calculate mass tensor
      util.k_tensor( a, b, this.r1, this.r2, this ); // last parameter is used to inject .k1 & .k2 into this
      
      // compute max impulse
      this.jMaxLen = this.maxForce * dt;
      
      // calculate bias velocity
      var delta = b.p.add(this.r2).sub( a.p.add(this.r1) );
      this.bias = delta.mult( -util.bias_coef(this.errorBias, dt)/dt ).clamp( this.maxBias );
    },
    
    /**
     * @param {Number} dt_coef
     */
    applyCachedImpulse: function(dt_coef){
      var a = this.a;
      var b = this.b;

      util.apply_impulses(a, b, this.r1, this.r2, this.jAcc.mult(dt_coef));
    },
    
    applyImpulse: function(){
      var a = this.a;
      var b = this.b;
      
      var r1 = this.r1;
      var r2 = this.r2;

      // compute impulse
      var vr = util.relative_velocity(a, b, r1, r2);

      var j = util.mult_k( this.bias.sub(vr), this.k1, this.k2 );
      var jOld = this.jAcc;
      this.jAcc = this.grooveConstrain(jOld.add(j));
      j = this.jAcc.sub(jOld);
      
      // apply impulse
      util.apply_impulses(a, b, this.r1, this.r2, j);
    },
    
    grooveConstrain: function(j){
      var n = this.grv_tn;
      var jClamp = (this.clamp * j.cross(n) > 0) ? j : j.project(n);
      
      return jClamp.clamp(this.jMaxLen);
    },
    
    /**
     * @returns {Number}
     */
    getImpulse: function(){
      return this.jAcc.length();
    },
    
    getGrooveA: function(){
      return this.grv_a;
    },
    setGrooveA: function(value){
      this.grv_a = value;
      this.grv_n = this.grv_b.sub(value).normalize().perp();
      this.activateBodies();
    },
    getGrooveB: function(){
      return this.grv_b;
    },
    setGrooveB: function(value){
      this.grv_b = value;
      this.grv_n = value.sub(this.grv_a).normalize().perp();
      this.activateBodies();
    },
    getAnchr2: function(){
      return this.anchr2;
    },
    setAnchr2: function(value){
      this.anchr2 = value;
    }

  });
  
  return GrooveJoint;
});
