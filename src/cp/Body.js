/**
 * @namespace cp
 */
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

  /**
   * Chipmunk's rigid body struct.
   * @class Body
   * @param {number} mass
   * @param {number} moment
   */
  var Body = function(mass, moment){   
    /// Function that is called to integrate the body's velocity. (Defaults to cpBodyUpdateVelocity)
    this.velocity_func = this.updateVelocity;
    /// Function that is called to integrate the body's position. (Defaults to cpBodyUpdatePosition)
    this.position_func = this.updatePosition;
    
    /// Position of the rigid body's center of gravity.
    this.p = Vect.zero;
    /// Velocity of the rigid body's center of gravity.
    this.v = Vect.zero;
    /// Force acting on the rigid body's center of gravity.
    this.f = Vect.zero;
    
    /// Angular velocity of the body around it's center of gravity in radians/second.
    this._w = 0;
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
    
    this.node = new ComponentNode(undefined, undefined, 0);
  };
  
  /**
   * @namespace cp.Body.prototype
   */
  Body.prototype = {
    
    set w(val){
      if( val !== val ) debugger;
      this._w = val;
    },
    get w(){
      return this._w;
    },
    /**
     * Returns true if the body is sleeping.
     * @function isSleeping
     */
    isSleeping: function(){
      return !!this.node.root;
    },
    
    /**
     * Returns true if the body is static (not supposed to be moving).
     * @function isStatic
     */
    isStatic: function(){
      // patched the isStatic method to check for rogue+infinity mass+momentum
      return this.node.idleTime === Number.POSITIVE_INFINITY || 
        ( false && this.isRogue() && this.getMass() === Number.POSITIVE_INFINITY && this.getMoment() === Number.POSITIVE_INFINITY );
    },
    
    /**
     * Returns true if the body is rogue (not added to the space -> not affected by gravity).
     * @function isRogue
     */
    isRogue: function(){
      return !this.space; // not added to space == rogue
    },
    
    /**
     * Convert body relative/local coordinates to absolute/world coordinates.
     * @function local2World
     * @param {cp/Vect}
     */
    local2World: function(v){
      return this.p.add( v.rotate(this.rot) );
    },
    
    /**
     * Convert body absolute/world coordinates to  relative/local coordinates.
     * @function world2Local
     * @param {cp/Vect} v
     */
    world2Local: function(v){
      return v.sub(this.p).unrotate(this.rot);
    },
    
    /**
     * @function kineticEnergy
     * @return {number}
     */
    kineticEnergy: function(){
      var vsq = this.v.dot(this.v);
      var wsq = this.w * this.w;
      return (vsq ? vsq*this.m : 0) + (wsq ? wsq*this.i : 0);
    },
    
    /**
     * @function setMass
     * @param {number} mass
     */
    setMass: function(mass){
      this.activate();
      this.m = mass;
      this.m_inv = 1/mass;
    },
    /**
     * @function setMoment
     * @param {number} moment
     */
    setMoment: function(moment){
      this.activate();
      this.i = moment;
      this.i_inv = 1/moment;
    },
    /**
     * Adds a shape to this body.
     * @function addShape
     * @param {cp/Shape} shape
     */
    addShape: function(shape){
      var next = this.shapeList;
      if( next ){
        next.prev = shape;
      }
      shape.next = next;
      this.shapeList = shape;
    },
    /**
     * Removes a shape from the body
     * @function removeShape
     * @param {cp/Shape} shape
     */
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
    
    /**
     * @function removeConstraint
     * @param {cp/constraints/Constraint} constraint
     */
    removeConstraint: function(constraint){
      this.constraintList = this.filterConstraints( this.constraintList, constraint );
    },
    
    /**
     * @function setPos
     * @param {cp/Vect} pos
     */
    setPos: function(pos){
      this.activate();
      this.p = pos;
    },
    
    /**
     * Set the angle of this body in radians.
     * @function setAngle
     * @param {number} angle
     */
    setAngle: function(angle){
      this.activate();
      this.a = angle;
      this.rot = Vect.forangle(angle);
    },
    
    getSpace: function(){
      return this.space;
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
    
    /**
     * Zero both the forces and torques currently applied to the body.
     * @function resetForces
     */
    resetForces: function(){
      this.f = Vect.zero;
      this.t = 0;
    },
    
    /**
     * Add the force to body at a relative offset r from the center of gravity.
     * @function applyForce
     * @param {cp.Vect} force force to apply to the body
     * @param {cp.Vect} r offset of the impulse
     */
    applyForce: function(force, r){
      this.f = this.f.add(force);
      this.t += r.cross(force);
    },
    /**
     *  Add the impulse j to body at a relative offset r from the center of gravity.
     * @function applyImpulse
     * @param {cp.Vect} j impulse on the body
     * @param {cp.Vect} r offset of the impulse
     */
    applyImpulse: function( j, r ){
      this.activate();
      this.v = this.v.add( j.mult(this.m_inv) );
      this.w += this.i_inv * r.cross(j);
      //util.apply_impulse(this, j, r); 
    },
    
    /**
     * Iterate over all shapes of this body.
     * @function eachShape
     * @param {function} func a callback function. The callback should look like this: function(shape, data){}, "this" will refer to the body.
     * @param {object} data data to pass to the callbacks
     */
    eachShape: function( func, data ){
      var shape = this.shapeList;
      while(shape){
        var next = shape.next;
        func.call(this, shape, data);
        shape = next;
      }
    },
    /**
     * Iterate over all constraints of this body.
     * @function eachConstraint
     * @param {function} func a callback function. The callback should look like this: function(constraint, data){}, "this" will refer to the body.
     * @param {object} data data to pass to the callbacks
     */
    eachConstraint: function( func, data ){
      var constraint = this.constraintList;
      while( constraint ){
        var next = constraint.next(this);
        func.call(this, constraint, data);
        constraint = next;
      }
    },
    /**
     * Iterate over all shapes of this body.
     * @function eachArbiter
     * @param {function} func a callback function. The callback should look like this: function(arbiter, data){}, "this" will refer to the body.
     * @param {object} data data to pass to the callbacks
     */
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
      // TODO
      return;
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
    
    /**
     * @function getMass
     */
    getMass: function(){
      return this.m;
    },
    
    /**
     * @function getMoment
     */
    getMoment: function(){
      return this.i;
    },
    /**
     * @function getPos
     */
    getPos: function(){
      return this.p;
    },
    /**
     * @function getRot
     */
    getRot: function(){
      return this.rot;
    },
    /**
     * @function getAngle
     */
    getAngle: function(){
      return this.a;
    },
    /**
     * Returns the angular velocity of this body.
     * @function getAngVel
     */
    getAngVel: function(){
      return this.w;
    },
    /**
     * @function setAngVel
     * @param w
     */
    setAngVel: function(w){
      this.activate();
      this.w = w;
    },
    /**
     * @function getTorque
     */
    getTorque: function(){
      return this.t;
    },
    /**
     * @function setTorque
     * @param t
     */
    setTorque: function(t){
      this.activate();
      this.t = t;
    },
    /**
     * @function getVelLimit
     */
    getVelLimit: function(){
      return this.v_limit
    },
    /**
     * @function setVelLimit
     * @param w
     */
    setVelLimit: function(w){
      this.activate();
      this.v_limit = w;
    },
    /**
     * @function getAngVelLimit
     */
    getAngVelLimit: function(){
      return this.w_limit;
    },
    /**
     * @function setAngVelLimit
     * @param w
     */
    setAngVelLimit: function(w){
      this.activate();
      this.w_limit = w;
    },
    /**
     * @function getUserData
     * @return {object} object that is attached to this body
     */
    getUserData: function(){
      return this.data;
    },
    /**
     * @function setUserData
     * @param {object} data
     */
    setUserData: function(w){
      this.activate();
      this.data = w;
    },
    /**
     * @function getVel
     */
    getVel: function(){
      return this.v;
    },
    /**
     * @function setVel
     * @param v
     */
    setVel: function(w){
      this.activate();
      this.v = w;
    },
    /**
     * @function getForce
     */
    getForce: function(){
      return this.f;
    },
    /**
     * @function setForce
     * @param f
     */
    setForce: function(w){
      this.activate();
      this.f = w;
    },
    
    /**
     * @function getVelAtPoint
     * @param {cp.Vect} r
     */
    getVelAtPoint: function(r){
      return body.v.add( r.perp().mult(body.w) );
    },
    
    getVelAtWorldPoint: function(point){
      return this.getVelAtPoint( point.sub(this.p) );
    },
    
    getVelAtLocalPoint: function(point){
      return this.getVelAtPoint( point.rotate(this.rot) );
    },
    
    
    componentRoot: function(){} // TODO
  };

  /**  @namespace cp */
  /**
   * A static body (body that is not supposed to move in the space).
   * @class Body.Static
   */
  Body.Static = function(){
    var staticBody = new Body(Number.POSITIVE_INFINITY, Number.POSITIVE_INFINITY);
    staticBody.node.idleTime = Number.POSITIVE_INFINITY;
    return staticBody;
  };
  
  return Body;
});
