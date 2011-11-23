define(['cp/Vect', 'cp/cpf', 'cp/constraints/util', 'cp/Array', 'cp/assert'], function(Vect, cpf, util, arrays, assert){
  "use strict";
  // StaticBodySingleton
  
/// Used internally to track information on the collision graph.
/// @private
  var ComponentNode = function(root, next, idleTime){
    this.root = root;
    this.next = next;
    this.idleTime = idleTime;
  };
  
  
  
  /// Chipmunk's rigid body struct.
  var Body = function(mass, moment){   
    /// Function that is called to integrate the body's velocity. (Defaults to cpBodyUpdateVelocity)
    this.velocity_func = this.updateVelocity;
    /// Function that is called to integrate the body's position. (Defaults to cpBodyUpdatePosition)
    this.position_func = this.updatePosition;
    
    var node = new ComponentNode(undefined, undefined, 0);
    this.node = node;
    
    /// Position of the rigid body's center of gravity.
    this.p = Vect.zero;
    /// Velocity of the rigid body's center of gravity.
    this.v = Vect.zero;
    /// Force acting on the rigid body's center of gravity.
    this.f = Vect.zero;
    
    /// Angular velocity of the body around it's center of gravity in radians/second.
    this.w = 0;
    /// Torque applied to the body around it's center of gravity.
    this.t = 0;
    
    this.v_bias = Vect.zero;
    this.w_bias = 0;
    
    /// Maximum velocity allowed when updating the velocity.
    this.v_limit = Number.POSITIVE_INFINITY;
    /// Maximum rotational rate (in radians/second) allowed when updating the angular velocity.
    this.w_limit = Number.POSITIVE_INFINITY;
    
    /// Mass of the body.
    /// Must agree with cpBody.m_inv! Use cpBodySetMass() when changing the mass for this reason.
    this.setMass(mass);
    /// Moment of inertia of the body.
    /// Must agree with cpBody.i_inv! Use cpBodySetMoment() when changing the moment for this reason.
    this.setMoment(moment);
    /// Rotation of the body around it's center of gravity in radians.
    /// Must agree with cpBody.rot! Use cpBodySetAngle() when changing the angle for this reason.
    this.setAngle(0);
    
    this.arbiterList = undefined;
  };
  
  Body.prototype = {
    
    /**
     *  Returns true if the body is sleeping.
     * (true is node.root is set)
     */
    isSleeping: function(){
      return !!this.node.root;
    },
    
    /**
     * Returns true if the body is static.
     * (false if node.idleTime is infinite)
     */
    isStatic: function(){
      return this.node.idleTime === Number.POSITIVE_INFINITY;
    },
    
    isRogue: function(){
      this.space === undefined;
    },
    
    /**
     * Convert body relative/local coordinates to absolute/world coordinates.
     * @param {object}
     */
    local2World: function(v){
      return this.p.add( v.rotate(this.rot) );
    },
    
    /**
     * Convert body absolute/world coordinates to  relative/local coordinates.
     */
    world2Local: function(v){
      return v.sub(this.p).unrotate(this.rot);
    },
    
    kineticEnergy: function(){
      var vsq = this.v.dot(this.v);
      var wsq = this.w * this.w;
      return (vsq ? vsq*this.m : 0) + (wsq ? wsq*this.i : 0);
    },
    
    setMass: function(mass){
      this.activate();
      this.m = mass;
      this.m_inv = 1/mass;
    },
    
    setMoment: function(moment){
      this.activate();
      this.i = moment;
      this.i_inv = 1/moment;
    },
    
    addShape: function(shape){
      var next = this.shapeList;
      if( next ){
        next.prev = shape;
      }
      shape.next = next;
      this.shapeList = shape;
    },
    
    removeShape: function(shape){
      var prev = shape.prev;
      var next = shape.next;
      
      if( prev ){
        prev.next = next;
      }else{
        this.shapeList = next;
      }
      
      if( next ){
        next.prev = prev;
      }
      
      shape.prev = undefined;
      shape.next = undefined;
    },
    
    filterConstraints: function(node, filter){
      if( node === filter ){
        return node.next(body);
      }else if( node.a === this ){
        node.next_a = this.filterConstraints(node.next_a, filter);
      }else{
        node.next_b = this.filterConstraints(node.next_b, filter);
      }
      return node;
    },
    
    removeConstraint: function(constraint){
      this.constraintList = this.filterConstraints( this.constraintList, constraint );
    },
    
    setPos: function(pos){
      this.activate();
      this.p = pos;
    },
    
    setAngle: function(angle){
      this.activate();
      this.a = angle;
      this.rot = Vect.forangle(angle);
    },
    
    updateVelocity: function(gravity, damping, dt){
      this.v = this.v.mult(damping).add(gravity.add(this.f.mult(this.m_inv)).mult(dt)).clamp(this.v_limit);

      var w_limit = this.w_limit;
      this.w = cpf.clamp(this.w*damping + this.t*this.i_inv*dt, -w_limit, w_limit);
    },
    
    updatePosition: function(dt){
      this.p = this.p.add( this.v.add(this.v_bias).mult(dt) );
      this.setAngle(this.a + (this.w + this.w_bias)*dt);
      
      this.v_bias = Vect.zero;
      this.w_bias = 0;
    },
    
    resetForces: function(){
      this.f = Vect.zero;
      this.t = 0;
    },
    
    applyForce: function(force, r){
      this.f = this.f.add(force);
      this.t += r.cross(force);
    },
    
    applyImpulse: function( j, r ){
      this.activate();
      this.v = this.v.add( j.mult(this.m_inv) );
      this.w += this.i_inv * r.cross(j);
      //util.apply_impulse(this, j, r); 
    },
    
    eachShape: function( func, data ){
      var shape = this.shapeList;
      while(shape){
        var next = shape.next;
        func.call(this, shape, data);
        shape = next;
      }
    },
    
    eachConstraint: function( func, data ){
      var constraint = this.constraintList;
      while( constraint ){
        var next = constraint.next(this);
        func.call(this, constraint, data);
        constraint = next;
      }
    },
    
    eachArbiter: function(func, data ){
      var arb = this.arbiterList;
      
      while( arb ){
        var next = arb.next(this);
        arb.swappedColl = (this === arb.body_b);
        func.call(this, arb, data);
        arb = next;
      }
    },
    
    activate: function(){
      if( !this.isRogue() ){
        
        var root = this.componentRoot();
        if( root ){
          root.componentActivate();
        }
        this.node.idleTime = 0;
      }
    },
    // #define CP_BODY_FOREACH_ARBITER(bdy, var) for(cpArbiter *var = bdy->arbiterList; var; var = cpArbiterNext(var, bdy))
    activateStatic: function(filter){
      if( this.isStatic() ){
        for( var var1 = this.arbiterList; var1; var1 = var1.next(this) ){
          if( !filter || filter === var1.a || filter === var1.b ){
            ((var1.body_a === this)? var1.body_b : var1.body_a).activate();
          }
        }
      }
    },

    
    sleep: function(){
      this.sleepWithGroup(undefined);
    },
    sleepWithGroup: function(group){
      assert.hard( !this.isStatic() && !this.isRogue() , "Rogue and static bodies cannot be put to sleep." );
      var space = this.space;
        
      assert.hard(space, "Cannot put a rogue body to sleep." );

      assert.hard(!space.locked, "Bodies cannot be put to sleep during a query or a call to cpSpaceStep(). Put these calls into a post-step callback.");
      assert.hard(group === undefined || group.isSleeping(), "Cannot use a non-sleeping body as a group identifier.");
	
      if(this.isSleeping()){
        assert.hard(this.componentRoot() === group.componentRoot(), "The body is already sleeping and it's group cannot be reassigned.");
      }
        
      // #define CP_BODY_FOREACH_SHAPE(body, var)	for(cpShape *var = body->shapeList; var; var = var->next)
      for( var var1 = this.shapeList; var1; var1 = var1.next ){
        var1.update(this.p, this.rot);
      }
      space.deactivateBody(this);
      if( group ){
        var root = group.componentRoot();
        var node = new ComponentNode(root, root.node.next, 0);
        this.node = node;
        root.node.next = this;
      }else{
        var node = new ComponentNode(this, undefined, 0);
        this.node = node;
        space.sleepingComponents.push(this);
      }
      arrays.deleteObj(space.bodies, this);
    
    },
    
    getMass: function(){
      return this.m;
    },
    
    getMoment: function(){
      return this.i;
    },
    
    getPos: function(){
      return this.p;
    },
    
    getRot: function(){
      return this.rot;
    },
    
    getAngle: function(){
      return this.a;
    },
    
    getAngVel: function(){
      return this.w;
    },
    setAngVel: function(w){
      this.activate();
      this.w = w;
    },
    
    getTorque: function(){
      return this.t;
    },
    setTorque: function(t){
      this.activate();
      this.t = t;
    },
    
    getVelLimit: function(){
      return this.v_limit
    },
    setVelLimit: function(w){
      this.activate();
      this.v_limit = w;
    },

    getAngVelLimit: function(){
      return this.w_limit;
    },
    setAngVelLimit: function(w){
      this.activate();
      this.w_limit = w;
    },
    
    getUserData: function(){
      return this.data;
    },
    setUserData: function(w){
      this.activate();
      this.data = w;
    },

    getVel: function(){
      return this.v;
    },
    setVel: function(w){
      this.activate();
      this.v = w;
    },
    
    getForce: function(){
      return this.f;
    },
    setForce: function(w){
      this.activate();
      this.f = w;
    },
    
    
    componentRoot: function(){
      return this.node.root;
    },
    
    componentActivate: function(){
        if( !this.isSleeping() ){
          return;
        }
        assert.soft( !this.isRogue(), "Internal Error: ComponentActivate() called on a rogue body." );
        
        var space = this.space;
        var body = this;
        
        while(body){
          var next = body.node.next;
          body.node.root = undefined;
          body.node.next = undefined;
          space.activateBody(body);
          body = next;
        }
        arrays.deleteObj(space.sleepingComponents, this);
    },
    
    componentActive: function(threshold){
      for( var body = this; body; body = body.node.next ){
        if( body.node.idleTime < threshold ){
          return true;
        }
      }
      return false;
    },
    
    componentAdd: function(/* root, */ body){
      body.node.root = this;

      if(body !== this){
        body.node.next = this.node.next;
        this.node.next = body;
      }
    },
    
    floodFillComponent: function(body){
      var root = this;

      if( !body.isStatic() && !body.isRogue() ){
        var other_root = body.componentRoot();
        if( !other_root ){
          root.componentAdd(body);
          
          //#define CP_BODY_FOREACH_ARBITER(bdy, var)	for(cpArbiter *var = bdy->arbiterList; var; var = cpArbiterNext(var, bdy))
          //#define CP_BODY_FOREACH_CONSTRAINT(bdy, var) for(cpConstraint *var = bdy->constraintList; var; var = cpConstraintNext(var, bdy))
          for( var arb = body.arbiterList; arb; arb = arb.next(body) ){
            root.floodFillComponent( (body === arb.body_a)?arb.body_b:arb.body_a );
          }
          for( var constraint = body.constraintList; constraint; constraint = constraint.next(body) ){
            root.floodFillComponent( (body === constraint.a)?constraint.b:constraint.a );
          }

        }else{
          assert.soft(other_root === root, "Internal Error: Inconsistency dectected in the contact graph.");
        }
      }
    },
    
    pushArbiter: function(arb){
      assert.soft( !arb.threadForBody(this).next, "Internal Error: Dangling contact graph pointers detected. (A)" );
      assert.soft( !arb.threadForBody(this).prev, "Internal Error: Dangling contact graph pointers detected. (B)" );
      
      var next = this.arbiterList;
      assert.soft( !next || !next.threadForBody(this).prev, "Internal Error: Dangling contact graph pointers detected. (C)" );
      arb.threadForBody(this).next = next;
      if( next ){
        next.threadForBody(this).prev = arb;
      }
      this.arbiterList = arb;

    }
	
  };
  
  Body.Static = function(){
    var staticBody = new Body(Number.POSITIVE_INFINITY, Number.POSITIVE_INFINITY);
    staticBody.node.idleTime = Number.POSITIVE_INFINITY;
    return staticBody;
  };
  
  return Body;
});
