/**
 * @namespace cp
 */
define(['cp/Body', 'cp/Vect', 'cp/Shape', 'cp/Arbiter', 'cp/HashSet', 'cp/SpaceHash', 'cp/BBTree', 'cp/ContactBuffer', 'cp/constraints/util', 'cp/cpf', 'cp/Array', 'cp/CollisionHandler', 'cp/Hash', 'cp/assert'], 
    function(Body, Vect, Shape, Arbiter, HashSet, SpaceHash, BBTree, ContactBuffer, util,  cpf, arrays, CollisionHandler, hash_pair, assert){
  "use strict";

  // Default collision functions.
  var alwaysCollide = function(/* arb, */ space, data){ return true; }
  var nothing = function(space, data){}
  
  // cpCollisionHandler cpDefaultCollisionHandler = {0, 0, alwaysCollide, alwaysCollide, nothing, nothing, NULL};
  var defaultCollisionHandler = new CollisionHandler(0,0,alwaysCollide, alwaysCollide, nothing, nothing, undefined); 


  // callback from the spatial hash
  var shapeQueryHelper = function(b, context){
    if( (a.group && a.group === b.group) ||
        !(a.layers & b.layers) ||
        a === b ){
      return;
    }
    var contacts = [];
    // Shape 'a' should have the lower shape type. (required by cpCollideShapes() )
    if( a.type <= b.type ){
      a.collideShapes(b, contacts);
    }else{
      b.collideShapes(a, contacts);
      for(var i = 0; i < contacts.length; ++i){
        contacts[i].n = contacts[i].n.neg();
      }
    }
    if( contacts.length ){
      context.anyCollision = !(a.sensor || b.sensor);
      
      if( context.func ){
        var set = [];
        for( var i = 0; i < contacts.length; ++i ){
          set[i] = {};
          set[i].point = contacts[i].p;
          set[i].normal = contacts[i].n;
          set[i].dist = contacts[i].dist;
        }
        context.func.call(b, set, context.data);
      }
    }
  };
  

  /**
   * Basic Unit of Simulation in Chipmunk.
   * @class Space
   * @param {object} options
   */
  var Space = function(options){
    var defaultOptions = {};
    if( typeof options === 'object' ){
      defaultOptions = util.extend( defaultOptions, options );
    }
    
    /// Number of iterations to use in the impulse solver to solve contacts.
    this.iterations = 10;
    /// Gravity to pass to rigid bodies when integrating velocity.
    this.gravity = Vect.zero;
    /// Damping rate expressed as the fraction of velocity bodies retain each second.
    /// A value of 0.9 would mean that each body's velocity will drop 10% per second.
    /// The default value is 1.0, meaning no damping is applied.
    /// @note This damping value is different than those of cpDampedSpring and cpDampedRotarySpring.
    this.damping = 1;
    /// Speed threshold for a body to be considered idle.
    /// The default value of 0 means to let the space guess a good threshold based on gravity.
    this.idleSpeedThreshold = 0;
    /// Time a group of bodies must remain idle in order to fall asleep.
    /// Enabling sleeping also implicitly enables the the contact graph.
    /// The default value of INFINITY disables the sleeping algorithm.
    this.sleepTimeThreshold = Number.POSITIVE_INFINITY;
    /// Amount of encouraged penetration between colliding shapes..
    /// Used to reduce oscillating contacts and keep the collision cache warm.
    /// Defaults to 0.1. If you have poor simulation quality,
    /// increase this number as much as possible without allowing visible amounts of overlap.
    this.collisionSlop = 0.1;
    /// Determines how fast overlapping shapes are pushed apart.
    /// Expressed as a fraction of the error remaining after each second.
    /// Defaults to pow(1.0 - 0.1, 60.0) meaning that Chipmunk fixes 10% of overlap each frame at 60Hz.
    this.collisionBias = Math.pow(1.0 - 0.1, 60.0);
    /// Number of frames that contact information should persist.
    /// Defaults to 3. There is probably never a reason to change this value.
    this.collisionPersistence = 3;
    /// Rebuild the contact graph during each step. Must be enabled to use the cpBodyEachArbiter() function.
    /// Disabled by default for a small performance boost. Enabled implicitly when the sleeping feature is enabled.
    this.enableContactGraph = false;
    /// User definable data pointer.
    /// Generally this points to your game's controller or game state
    /// class so you can access it when given a cpSpace reference in a callback.
    this.data = undefined;
    
    /// The designated static body for this space.
    /// You can modify this body, or replace it with your own static body.
    /// By default it points to a statically allocated cpBody in the cpSpace struct.
    this.staticBody = new Body.Static();
    
    this.locked = 0;
    this.stamp = 0;
    
    this.curr_dt = 0;
    
    //*
    this.staticShapes = new BBTree( Shape.prototype.getBB, undefined );
    this.activeShapes = new BBTree( Shape.prototype.getBB, this.staticShapes ); 
    this.activeShapes.setVelocityFunc( Shape.prototype.velocityFunc ); 
    //*/
    
    
    this.bodies = [];
    this.sleepingComponents = [];
    this.rousedBodies = [];
    this.arbiters = [];
    //this.pooledArbiters = [];

    this.cachedArbiters = new HashSet(0, function(shapes){
      var arb = this;
      var a = shapes[0];
      var b = shapes[1];
      return ((a === arb.a && b === arb.b) || (b === arb.a && a === arb.b));
    }); 

    this.constraints = [];
    this.defaultHandler = defaultCollisionHandler;
    this.collisionHandlers = new HashSet(0, function(check, pair){ 
      return ((check.a === pair.a && check.b === pair.b) || (check.b === pair.a && check.a === pair.b));
    }); 
    
    this.collisionHandlers.setDefaultValue(defaultCollisionHandler);
    this.postStepCallbacks = undefined;
  };
 
 /**
  * @namespace cp.Space.prototype
  */
  Space.prototype = {

    // TODO check if I can kick out the destroy functions.
    destroy: function(){
      this.bodies = undefined;
      this.sleepingComponents = undefined;
      this.rousedBodies = undefined;
      this.constraints = undefined;
      this.cachedArbiters.free();
      this.arbiters = undefined;
    },
    free: function(){ // TODO: same as above, just kept for now.
      this.destroy();
    },
    
    /**
     * Set a collision handler to handle specific collision types. 
      The methods are called only when shapes with the specified collisionTypes collide.
      typeA and typeB should be the same object references set to cp.Shape.collisionType. They can be any uniquely identifying object.
     * @function addCollisionHandler
     * @param {number|string} a
     * @param {number|string} b
     * @param {function} begin a calback function that is called at the beginning of a collision. Could look like this: function(space, data){ return true; } In the callback the context ("this") is set to the arbiter that handles the collision. Returning false from the callback ignores the collision.
     * @param {function} presolve a calback function that is called before the collision is solved. Callback is the same as for begin
     * @param {function} postsolve a calback function that is called after the collision is solved. Could look like this: function(space, data){} In the callback the context ("this") is set to the arbiter that handles the collision. 
     * @param {function} seperate A callback that is called when the shape seperate again. Callback is the same as for postsolve
     * @param {object} [data=undefined] optional data object that will be passed to the callbacks.
     */
    addCollisionHandler: function(a, b, begin, presolve, postsolve, separate, data){
      // Remove any old function so the new one will get added.
      this.removeCollisionHandler(a,b);
      var handler = new CollisionHandler( a, b, begin||alwaysCollide, presolve||alwaysCollide, postsolve||nothing, separate||nothing, data); // TODO
      this.collisionHandlers.insert( hash_pair(a,b), handler, undefined, function(handler, unused){
        // Transformation function for collisionHandlers.
        return handler.clone();
      }); 
    },
    
    /**
     * Remove a collision handler for a given collision type pair.
     * @function removeCollisionHandler
     * @param {number|string} a
     * @param {number|string} b
     */
    removeCollisionHandler: function(a, b){
      var ids = {
        a: a,
        b: b
      };
      var oldHandler = this.collisionHandlers.remove( hash_pair(a,b), ids ); // TODO
    },
    
    /**
     * Register a default collision handler to be used when no specific collision handler is found. The space is given a default handler when created that returns true for all collisions in begin() and preSolve() and does nothing in the postSolve() and separate() callbacks.
     * @function setDefaultCollisionHandler
     * @param {function} begin a calback function that is called at the beginning of a collision. Could look like this: function(space, data){ return true; } In the callback the context ("this") is set to the arbiter that handles the collision. Returning false from the callback ignores the collision.
     * @param {function} presolve a calback function that is called before the collision is solved. Callback is the same as for begin
     * @param {function} postsolve a calback function that is called after the collision is solved. Could look like this: function(space, data){} In the callback the context ("this") is set to the arbiter that handles the collision. 
     * @param {function} seperate A callback that is called when the shape seperate again. Callback is the same as for postsolve
     * @param {object} [data=undefined] optional data object that will be passed to the callbacks.
     */
    setDefaultCollisionHandler: function(begin, preSolve, postSolve, separate, data){
      var handler = new CollisionHandler( 0, 0, begin || alwaysCollide, preSolve || alwaysCollide, postSolve||nothing, separate||nothing, data );
      this.defaultHandler = handler;
      this.collisionHandlers.setDefaultValue(this.defaultHandler);
    },
    
    /**
     * Add shape to the space. The add function cannot be called from within a callback other than a postStep() callback (which is different than a postSolve() callback!). Attempting to add or remove objects from the space while cpSpaceStep() is still executing will throw an assertion. 
     * The add functions return the thing being added so that you can create and add something in one line. 
     * @function addShape
     * @param {cp.Shape} shape
     * @return {cp.Shape} the input shape
     */
    addShape: function(shape){
      var body = shape.body;
      if( body.isStatic() ){
        return this.addStaticShape(shape);
      }
      body.activate();
      body.addShape(shape);
      shape.update( body.p, body. rot );
      this.activeShapes.insert( shape, shape.hashid );
      shape.space = this;
      return shape;
    },
    
    /**
     * Shapes attached to static bodies are automatically treated as static. There isn’t really a good reason to explicitly add static shapes anymore.
     * Used internally by addShape().
     * @function addStaticShape
     * @deprecated
     * @private
     */
    addStaticShape: function(shape){
      var body = shape.body;
      body.addShape(shape);
      shape.update( body.p, body.rot );
      this.staticShapes.insert(shape, shape.hashid);
      shape.space = this;
      return shape;
    },
    
    /**
     * Adds a body to the space.
     * @function addBody
     * @param {cp.Body} body 
     * @return {cp.Body} the input body
     */
    addBody: function(body){
      this.bodies.push(body);
      body.space = this;
      return body;
    },
    
    /**
     * Adds a Constraint to the space.
     * @function addConstraint
     * @param {cp.constraints.Constraint} constraint
     * @return {cp.constraints.Constraint} the input constraint
     */
    addConstraint: function(constraint){
      constraint.a.activate();
      constraint.b.activate();
      this.constraints.push(constraint);
      // Push onto the heads of the bodies' constraint lists
      var a = constraint.a;
      var b = constraint.b;
      constraint.next_a = a.constraintList;
      a.constraintList = constraint;
      constraint.next_b = b.constraintList;
      b.constraintList = constraint;
      constraint.space = this;
      return constraint;
    },
    
    filterArbiters: function(body, filter){
      var context = {
        space: this,
        body: body,
        shape: filter
      };
      this.cachedArbiters.filter( this.cachedArbitersFilter, context ); 
    },
    

    cachedArbitersFilter: function( arb, context ){
      var shape = context.shape;
      var body = context.body;
      
      // Match on the filter shape, or if it's NULL the filter body
      if( (body === arb.body_a && (shape === arb.a || !shape)) || 
          ( body === arb.body_b && (shape === arb.b || !shape)) ){
        // Call separate when removing shapes.
        if( shape && arb.state !== Arbiter.State.cached ){
          arb.callSeparate(context.space);
        }
        arb.unthread(); // TODO
        arrays.deleteObj( context.space.arbiters, arb );
        //context.space.pooledArbiters.push(arb);
        return false;
      }
      return true;
    },
    
    /**
     * Remove a shape from the space again.
     * @function removeShape
     * @param {cp.Shape} shape
     */
    removeShape: function(shape){
      var body = shape.body;
      if( body.isStatic() ){
        this.removeStaticShape(shape);
      }else{
        body.activate();
        body.removeShape(shape);
        this.filterArbiters(body, shape);
        this.activeShapes.remove( shape, shape.hashid );
        shape.space = undefined;
      }
    },
    
    /**
     * Removes a static shape from the space again. Deprecated since this is handled by removeShape() internally.
     * This function should be treated as private.
     * @function removeStaticShape
     * @param {cp.Shape} shape
     * @deprecated
     * @private
     */
    removeStaticShape: function(shape){
      var body = shape.body;
      if( body.isStatic() ){
        body.activateStatic(shape);
      }
      body.removeShape(shape);
      this.filterArbiters(body, shape);
      this.staticShapes.remove(shape, shape.hashid);
      shape.space = undefined;
    },

    /**
     * Removes a body from the space again.
     * @function removeBody
     * @param {cp.Body} body
     */
    removeBody: function(body){
      assert.hard( this.containsBody(body), 'Cannot remove a body that was not added to the space. (Removed twice maybe?)');
      body.activate();
      this.filterArbiters(body, undefined);
      arrays.deleteObj(this.bodies, body);
      body.space = undefined;
    },

    /**
     * Removes a constraint from the space again
     * @function removeConstraint
     * @param {cp.constraints.Constraint} constraint
     */
    removeConstraint: function(constraint){
      constraint.a.activate();
      constraint.b.activate();
      arrays.deleteObj(this.constraints, constraint);
      constraint.a.removeConstraint(constraint);
      constraint.b.removeConstraint(constraint);
      constraint.space = undefined;
    },
    
    /**
     * @function containsShape
     * @param {cp.Shape} shape
     */
    containsShape: function(shape){
      return (shape.space === this);
    },
    /**
     * @function containsBody
     * @param {cp.Body} body
     */
    containsBody: function(body){
      return (body.space === this);
    },
    /**
     * @function containsConstraint
     * @param {cp.constraints.Constraint} constraint
     */
    containsConstraint: function(constraint){
      return (constraint.space === this);
    },

    lock: function(){
      this.locked++;
    },
    unlock: function(runPostStep){
      this.locked--;
      if( !this.locked && runPostStep ){
        var waking = this.rousedBodies;
        for( var i =0 , count=waking.length; i < count; i++ ){
          this.activateBody(waking[i]);
        }
        waking.length =0;
        this.runPostStepCallbacks();
      }
    },
    
    /**
     * Call func for each body in the space also passing along your data pointer. Sleeping bodies are included, but static and rogue bodies are not as they aren’t added to the space.
     * @function eachBody
     * @param {function} func
     * @param {object} data
     */
    eachBody: function(func, data){
      this.lock();
      
      var bodies = this.bodies;
      for( var i = 0; i < bodies.length; ++i ){
        func(bodies[i], data);
      }
      var components = this.sleepingComponents;
      for( var i = 0; i < components.length; ++i ){
        var root = components[i];
        var body = root;
        while( body ){
          var next = body.node.next;
          func(body, data);
          body = next;
        }
      }
      
      this.unlock();
    },
    
    eachShapeIterator: function(/*shape, */context){
      context.func(this, context.data);
    },
    
    /**
     * Call func for each shape in the space also passing along your data pointer. Sleeping and static shapes are included.
     * @function eachShape
     * @param {function} func
     * @param {object} data
     */
    eachShape: function( func, data ){
      this.lock();
      var context = {
        func: func,
        data: data
      };
      this.activeShapes.each( this.eachShapeIterator, context ); 
      this.staticShapes.each( this.eachShapeIterator, context );      
      this.unlock(true);
    },  
    
    /**
     * Call func for each constraint in the space also passing along your data pointer.
     * @function eachConstraint
     * @param {function} func
     * @param {object} data
     */
    eachConstraint: function(func, data){
      this.lock();
      var constraints = this.constraints;
      for( var i = 0; i < constraints.length; ++i ){
        func(constraints[i], data);
      }
      this.unlock(true);
    },
    
    /**
     * Reindex all static shapes. Generally updating only the shapes that changed is faster.
     * @function reindexStatic
     */
    reindexStatic: function(){
      this.staticShapes.each( Shape.prototype.updateBBCache, undefined );
      this.staticShapes.reindex();
    },
    
    /**
     * Reindex a specific shape.
     * @function reindexShape
     * @param {cp.Shape} shape
     */
    reindexShape: function(shape){
      var body = shape.body;
      shape.update(body.p, body.rot);
      // attempt to rehash the shape in both hashes
      this.activeShapes.reindexObject(shape, shape.hashid);
      this.staticShapes.reindexObject(shape, shape.hashid);
    },
    
    /**
     *  Reindex all the shapes for a certain body.
     * @function reindexShapesForBody
     * @param {cp.Body} body
     */
    reindexShapesForBody: function(body){
      for(var var1 = body.shapeList; var1; var1 = var1.next){
        this.reindexShape(var1);
      }
    },
    
    /**
     * Switch the space to use a spatial hash instead of the bounding box tree.
     * The spatial hash data is fairly size sensitive. 
     * @function useSpatialHash
     * @param {number} dim the size of the hash cells. Setting dim to the average collision shape size is likely to give the best performance. Setting dim too small will cause the shape to be inserted into many cells, setting it too low will cause too many objects into the same hash slot.
     * @param {number} count the suggested minimum number of cells in the hash table. If there are too few cells, the spatial hash will return many false positives. Too many cells will be hard on the cache and waste memory. the Setting count to ~10x the number of objects in the space is probably a good starting point. Tune from there if necessary.
     */
    useSpatialHash: function(dim, count){
      var staticShapes = new SpaceHash(dim, count, Shape.prototype.getBB, undefined );
      var activeShapes = new SpaceHash(dim, count, Shape.prototype.getBB, staticShapes );
      
      if( this.staticShapes ){
        this.staticShapes.each( Shape.prototype.copyShapes, staticShapes );
      }
      if( this.activeShapes ){
        this.activeShapes.each( Shape.prototype.copyShapes, activeShapes );
      }
      this.staticShapes = staticShapes;
      this.activeShapes = activeShapes;
    },
    
    /**
     * Get the default static body for this space
     * @function getStaticBody
     */
    getStaticBody: function(){
      return this.staticBody;
    },
    /**
     * @function getCurrentTimeStep
     */
    getCurrentTimeStep: function(){
      return this.curr_dt;
    },
    /**
     * @function setIterations
     * @param {number} i
     */
    setIterations: function(i){
      this.iterations = i;
    },
    /**
     * @function getIterations
     */
    getIterations: function(){
      return this.iterations;
    },
    /**
     * @function setGravity
     * @param {cp.Vect} gravity
     */
    setGravity: function(i){
      this.gravity = i;
    },
    /**
     * @function getGravity
     */
    getGravity: function(){
      return this.gravity;
    },
    /**
     * @function setDamping
     * @param d
     */
    setDamping: function(d){
      this.damping = d;
    },
    /**
     * @function getDamping
     */
    getDamping: function(){
      return this.damping;
    },
    /**
     * @function setIdleSpeedThreshold
     * @param s
     */
    setIdleSpeedThreshold: function(s){
      this.idleSpeedThreshold = s;
    },
    /**
     * @function getIdleSpeedThreshold
     */
    getIdleSpeedThreshold: function(){
      return this.idleSpeedThreshold;
    },
    /**
     * @function setSleepTimeThreshold
     * @param {number} s
     */
    setSleepTimeThreshold: function(s){
      this.sleepTimeThreshold = s;
    },
    /**
     * @function getSleepTimeThreshold
     */
    getSleepTimeThreshold: function(){
      return this.sleepTimeThreshold;
    },
    /**
     * @function setCollisionSlop
     * @param s
     */
    setCollisionSlop: function(s){
      this.collisionSlop = s;
    },
    /**
     * @function getCollisionSlop
     */
    getCollisionSlop: function(){
      return this.collisionSlop;
    },
    /**
     * @function setCollisionBias
     * @param b
     */
    setCollisionBias: function(b){
      this.collisionBias = b;
    },
    /**
     * @function getCollisionBias
     */
    getCollisionBias: function(){
      return this.collisionBias;
    },
    /**
     * @function setCollisionPersistence
     * @param {number} p
     */
    setCollisionPersistence: function(p){
      this.collisionPersistence = p;
    },
    /**
     * @function getCollisionPersistence
     */
    getCollisionPersistence: function(){
      return this.collisionPersistence;
    },
    /**
     * @function setEnableContactGraph
     * @param {boolean} g
     */
    setEnableContactGraph: function(g){
      this.enableContactGraph = g;
    },
    /**
     * @function getEnableContactGraph
     * @return {boolean}
     */
    getEnableContactGraph: function(){
      return this.enableContactGraph;
    },
    /**
     * @function setUserData
     * @param {object} u data to attach to this space
     */
    setUserData: function(u){
      this.data = u;
    },
    /**
     * @function getUserData
     */
    getUserData: function(){
      return this.data;
    },
    
    isLocked: function(){
      return !!this.locked;
    },
    
    /**
     * @function step
     * @param {number} dt the timestep to use for this step
     */
    step: function(dt){ 
      if( dt === 0 ){ 
        return;
      }
      this.stamp++;
      
      var prev_dt = this.curr_dt;
      this.curr_dt = dt;
      
      // reset and empty the arbiter list.
      var bodies = this.bodies;
      var arbiters = this.arbiters;
      var constraints = this.constraints;
      for( var i = 0; i < arbiters.length; ++i ){
        var arb = arbiters[i];
        arb.state = Arbiter.State.normal; 
        // If both bodies are awake, unthread the arbiter from the contact graph
        if( !arb.body_a.isSleeping() && !arb.body_b.isSleeping() ){
          arb.unthread(); // TODO
        }
      }
      this.arbiters = arbiters = [];
      
      
      this.lock(); {
        // Integrate positions
        for( var i = 0; i < bodies.length; ++i ){
          var body = bodies[i];
          body.position_func(dt, true);
        }
        // Find colliding pairs.
        //this.pushFreshContactBuffer();
        this.activeShapes.each(Shape.prototype.updateFunc, undefined);
        this.activeShapes.reindexQuery(this.collideShapes, this);
      } 
      this.unlock(false);
      
      // rebuild the contact graph (and detect sleeping components if sleeping is enabled)
      this.processComponents(dt); // TODO: floodFillComponents causes infinite loop. Enable sleeping if bug is fixed
      
      this.lock();{
        // Clear out old cached arbiters and call separate callbacks
        this.cachedArbiters.filter( Space.prototype.arbiterSetFilter, this );
        // Prestep the arbiters and constraints.
        var slop = this.collisionSlop;
        var biasCoef = 1 - Math.pow( this.collisionBias, dt );
          
        for( var i = 0; i < arbiters.length; ++i ){ 
          arbiters[i].preStep(dt, slop, biasCoef);
        }
        for( var i = 0; i  < constraints.length; ++i ){
          var constraint = constraints[i];
          
          if( constraint.preSolve ){
            constraint.preSolve(this);
          }
          constraint.preStep(dt);
        }
        // Integrate velocities.
        var damping = Math.pow(this.damping, dt);
        var gravity = this.gravity;
        for( var i = 0; i < bodies.length; ++i ){
          var body = bodies[i];
          body.velocity_func(gravity, damping, dt);
        }
        // Apply cached impulses
        var dt_coef = (prev_dt === 0? 0 : dt/prev_dt);
        for( var i = 0; i < arbiters.length; ++i ){
          arbiters[i].applyCachedImpulse(dt_coef);
        }
        for( var i = 0; i < constraints.length; ++i ){
          var constraint = constraints[i];
          constraint.applyCachedImpulse(dt_coef);
        }
        // Run the impulse solver
        for( var i = 0; i < this.iterations; ++i ){
          for( var j = 0; j < arbiters.length; ++j ){
            arbiters[j].applyImpulse();
          }
          for( var j = 0; j < constraints.length; ++j ){
            var constraint = constraints[j];
            constraint.applyImpulse();
          }
        }
        // Run the constraint post-solve callbacks
        for( var i = 0; i < constraints.length; ++i ){
          var constraint = constraints[i];
          if( constraint.postSolve ){
              constraint.postSolve(space);
          }
        }
        // Run the post-solve callbacks
        for( var i = 0; i < arbiters.length; ++i ){
          var arb = arbiters[i];
          var handler = arb.handler;
          handler.postSolve.call(this, handler.data);
        }
      } 
      this.unlock(true);
    },

    // callback from the spatial hash.
    collideShapes: function(/* a, */ b, space){ // NOTE: this = a
      var a = this;
      // Reject any of the simple cases
      if( a.queryReject(b) ){
        return;
      }
      var handler = space.lookupHandler(a.collision_type, b.collision_type);
      var sensor = a.sensor || b.sensor;
      if( sensor && handler === defaultCollisionHandler ){ 
        return ;
      }
      
      // Shape 'a' should have the lower shape type. (required by cpCollideShapes() )
      if(a.type > b.type){ 
        var temp = a;
        a = b;
        b = temp;
      }
      // Narrow-phase collision detection.
      var contacts = [];
      var numContacts = a.collideShapes(b, contacts);
      if( !numContacts ){
        return; // Shapes are not colliding.
      }
      //space.pushContacts(numContacts);    
      
      // Get an arbiter from space->arbiterSet for the two shapes.
      // This is where the persistant contact magic comes from.
      var shapePair = [ a, b ];
      var arbHashID = hash_pair(a.hash, b.hash); 
      
      var arb = space.cachedArbiters.insert(arbHashID, shapePair, space, Space.prototype.arbiterSetTrans);
      arb.update(contacts, handler, a, b); 
      
      // Call the begin function first if it's the first step
      if( arb.state === Arbiter.State.firstColl && !handler.begin(arb, space, handler.data) ){
        arb.ignore(); // permanently ignore the collision until separation
      }
      if( (arb.state !== Arbiter.State.ignore) && // Ignore the arbiter if it has been flagged
          handler.preSolve(arb, space, handler.data) && // Call preSolve
          !sensor ){ // Process, but don't add collisions for sensors.
          //if( !arrays.contains( space.arbiters, arb ) ){
            space.arbiters.push(arb);
         // }
          
      }else{
        //space.popContacts(numContacts);
        arb.contacts = undefined;
        arb.numContacts = 0;
        // Normally arbiters are set as used after calling the post-solve callback.
        // However, post-solve callbacks are not called for sensors or arbiters rejected from pre-solve.
        if( arb.state !== Arbiter.State.ignore ){
          arb.state = Arbiter.State.normal;
        }
      }
      // Time stamp the arbiter so we know it was used recently.
      arb.stamp = space.stamp;
    },

    postStepCallbackSetIter: function(callback, space){
      callback.func.call(space, callback.obj, callback.data);
    },
    
    runPostStepCallbacks: function(){
      // Loop because post step callbacks may add more post step callbacks directly or indirectly.
      while( this.postStepCallbacks ){
        var callbacks = this.postStepCallbacks;
        this.postStepCallbacks = undefined;
        callbacks.each( this.postStepCallbackSetIter, this );
      }
    },
    
    lookupHandler: function(a, b){
      var types = [ a, b ];
      return this.collisionHandlers.find( hash_pair(a, b), types );
    },
    
    arbiterSetTrans:  function(shapes, space){ 
      var arb;
      if( false && space.pooledArbiters.length ){ // dead for now
        arb = space.pooledArbiters.pop();
        Arbiter.call( arb, shapes[0], shapes[1] );
      }else{
        arb = new Arbiter( shapes[0], shapes[1] );
      }

      return arb;
    },
    
    // Hashset filter func to throw away old arbiters.
    arbiterSetFilter: function(arb, space){
      var ticks = space.stamp - arb.stamp;
      var a = arb.body_a;
      var b = arb.body_b;
      // TODO should make an arbiter state for this so it doesn't require filtering arbiters for dangling body pointers on body removal.
      // Preserve arbiters on sensors and rejected arbiters for sleeping objects.
      if( (a.isStatic() || a.isSleeping()) && (b.isStatic() || b.isSleeping()) ){
        return true;
      }
      // Arbiter was used last frame, but not this one
      if( ticks >= 1 && arb.state !== Arbiter.State.cached ){
        arb.callSeparate(space);
        arb.state = Arbiter.State.cached;
      }
      if( ticks >= space.collisionPersistence ){
        arb.contacts = undefined;
        arb.numContacts = 0;
        //space.pooledArbiters.push(arb);
        return false;
      }
      return true;
    },
    
    uncacheArbiter: function(arb){
      var a = arb.a;
      var b = arb.b;
      
      var shape_pair = [ a, b ];
      
      var arbHashID = hash_pair(a.hash, b.hash);
      this.cachedArbiters.remove( arbHashID, shape_pair );
      arrays.deleteObj(this.arbiters, arb);
    },
    
    shapeQuery: function(shape, func, data){
      var body = shape.body;
      var bb = (body? shape.update(body.p, body.rot) : shape.bb);
      
      var context = {
        func: func,
        data: data,
        anyCollision: false
      };
      this.lock();
      this.activeShapes.indexQuery( shape, bb, shapeQueryHelper, context );
      this.staticShapes.indexQuery( shape, bb, shapeQueryHelper, context );
      this.unlock(true);
      
      return context.anyCollision;
    },
    
    processComponents: function(dt){
      var sleep = (this.sleepTimeThreshold !== Number.POSITIVE_INFINITY);
      var bodies = this.bodies;
      
      // Calculate the kinetic energy of all the bodies.
      if( sleep ){
        var dv = this.idleSpeedThreshold;
        var dvsq = dv ? dv*dv : (this.gravity.lengthsq() * dt * dt);
        
        // update idling and reset component nodes
        for( var i = 0, l = bodies.length; i < l; ++i ){
          var body = bodies[i];
          
          // Need to deal with infinite mass objects
          var keThreshold = dvsq ? body.m * dvsq : 0;
          body.node.idleTime = (body.kineticEnergy() > keThreshold) ? 0 : body.node.idleTime + dt;
        }
      }
      // Awaken any sleeping bodies found and then push arbiters to the bodies' lists.
      var arbiters = this.arbiters;
      for( var i = 0, count = arbiters.length; i < count; ++i ){
        var arb = arbiters[i];
        var a = arb.body_a;
        var b = arb.body_b;
        
        if( sleep ){
          if( (b.isRogue() && !b.isStatic()) || a.isSleeping() ){
            a.activate();
          }
          if( (a.isRogue() && !a.isStatic()) || b.isSleeping() ){
            b.activate();
          }
        }
        a.pushArbiter(arb);
        b.pushArbiter(arb);
      }
      
      if( sleep ){
        // Bodies should be held active if connected by a joint to a non-static rouge body.
        var constraints = this.constraints;
        for( var i = 0, l = constraints.length; i < l; ++i ){
          var constraint = constraints[i];
          var a = constraint.a;
          var b = constraint.b;
          
          if( b.isRogue() && !b.isStatic() ){
            a.activate();
          }
          if( a.isRogue() && !a.isStatic() ){
            b.activate();
          }
        }
        // Generate components and deactivate sleeping ones
        for( var i = 0; i < bodies.length; ){
          var body = bodies[i];
          
          if( !body.componentRoot() ){
            // Body not in a component yet. Perform a DFS to flood fill mark 
            // the component in the contact graph using this body as the root.
            body.floodFillComponent(body);
            
            // Check if the component should be put to sleep.
            if( !body.componentActive(this.sleepTimeThreshold) ){
              this.sleepingComponents.push(body);
              for( var other = body; other; other = other.node.next ){
                this.deactivateBody( other );
              }
              // cpSpaceDeactivateBody() removed the current body from the list.
              // Skip incrementing the index counter.
              continue;
            }
          }
          ++i;
          // Only sleeping bodies retain their component node pointers.
          body.node.root = undefined;
          body.node.next = undefined;
        }
      }
    },
    
    activateBody: function(body){
      assert.hard( !body.isRogue(), "Internal error: Attempting to activate a rouge body." );
      
      if( this.locked ){
        // cpSpaceActivateBody() is called again once the space is unlocked
        if( ! arrays.contains( this.rousedBodies, body ) ){
          this.rousedBodies.push(body);
        }
      }else{
        this.bodies.push(body);
        for( var shape = body.shapeList; shape; shape = shape.next ){
          this.staticShapes.remove( shape, shape.hashid );
          this.activeShapes.insert( shape, shape.hashid );
        }
        for( var arb = body.arbiterList; arb; arb = arb.next(body) ){
          var bodyA = arb.body_a;
          if( body === bodyA || bodyA.isStatic() ){
            var numContacts = arb.numContacts;
            var contacts = arb.contacts;
            // Reinsert the arbiter into the arbiter cache
            var a = arb.a;
            var b = arb.b;
            var shape_pair = [ a, b ];
            var arbHashID = hash_pair( a.hash, b.hash );
            this.cachedArbiters.insert( arbHashID, shape_pair, arb, undefined );
            // Update the arbiter's state
            arb.stamp = this.stamp;
            arb.handler = this.lookupHandler( a.collision_type, b.collision_type );
            this.arbiters.push( arb );
          }
        }
        for( var constraint = body.constraintList; constraint; constraint = constraint.next(body) ){
          var bodyA = constraint.a;
          if( body === bodyA || bodyA.isStatic() ){
            this.constraints.push(constraint);
          }
        }
      }
    },
    
    deactivateBody: function(body){
      assert.hard( !body.isRogue(), "Internal error: Attempting to deactivate a rouge body." );
      arrays.deleteObj( this.bodies, body );
      
      for( var shape = body.shapeList; shape; shape = shape.next ){
        this.activeShapes.remove( shape, shape.hashid );
        this.staticShapes.insert( shape, shape.hashid );
      }
      for( var arb = body.arbiterList; arb; arb = arb.next(body) ){
        var bodyA = arb.body_a;
        if( body === bodyA || bodyA.isStatic() ){
          this.uncacheArbiter(arb);
        }
      }
      for( var constraint = body.constraintList; constraint; constraint = constraint.next(body) ){
        var bodyA = constraint.a;
        if( body === bodyA || bodyA.isStatic() ){
          arrays.deleteObj( this.constraints, constraint );
        }
      }
    }
    
  };
 
  return Space;
});
