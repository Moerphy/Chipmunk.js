define(['cp/Body', 'cp/Vect', 'cp/Shape', 'cp/Arbiter', 'cp/HashSet', 'cp/SpaceHash', 'cp/ContactBuffer', 'cp/constraints/util', 'cp/cpf', 'cp/Array', 'cp/CollisionHandler'], 
    function(Body, Vect, Shape, Arbiter, HashSet, SpaceHash, ContactBuffer, util,  cpf, arrays, CollisionHandler){
  "use strict";

  var hash_pair = function(a, b){
    // #define CP_HASH_COEF (3344921057ul)
    // #define CP_HASH_PAIR(A, B) ((cpHashValue)(A)*CP_HASH_COEF ^ (cpHashValue)(B)*CP_HASH_COEF)
    var hash_coef = 3344921057;
    var arbHashID = ( ( a * hash_coef) ^ ( b * hash_coef) ) & 0x7FFFFFFF ;  // TODO:  find better port solution to this hashing thingy..
    return arbHashID;
  };
  
  // Default collision functions.
  var alwaysCollide = function(/* arb, */ space, data){ return true; }
  var nothing = function(space, data){}
  
  // cpCollisionHandler cpDefaultCollisionHandler = {0, 0, alwaysCollide, alwaysCollide, nothing, nothing, NULL};
  var defaultCollisionHandler = new CollisionHandler(0,0,alwaysCollide, alwaysCollide, nothing, nothing, undefined); 

  /// Basic Unit of Simulation in Chipmunk
  var Space = function(options){
    var defaultOptions = {
      hashCellDim: 50
    };
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
    
    // IMPORTANT: this is different from the reference implementation, because the BBTree is considerable more difficult to port in JS.
    //this.staticShapes = new BBTree( undefined, undefined ); // TODO
    //this.activeShapes = new BBTree( undefined, this.staticShapes ); // TODO: implement BBTree
    this.useSpatialHash(defaultOptions.hashCellDim, defaultOptions.hashCellDim); // TODO: make better config options.
    if( this.activeShapes.setVelocityFunc ){ // Spatial hashes do not have a VelocityFunc?
      this.activeShapes.setVelocityFunc( Shape.prototype.velocityFunc ); 
    }
    
    this.bodies = [];
    this.sleepingComponents = [];
    this.rousedBodies = [];
    this.arbiters = [];
    //this.pooledArbiters = [];
    this.contactBuffersHead = undefined;
    this.cachedArbiters = new HashSet(0, function(shapes, arb){
      var a = shapes[0];
      var b = shapes[1];
      return ((a === arb.a && b === arb.b) || (b === arb.a && a === arb.b));
    }); // TODO? arbiterSetEql

    this.constraints = [];
    this.defaultHandler = defaultCollisionHandler;
    this.collisionHandlers = new HashSet(0, function(check, pair){ 
      return ((check.a === pair.a && check.b === pair.b) || (check.b === pair.a && check.a === pair.b));
    }); 
    
    this.collisionHandlers.setDefaultValue(defaultCollisionHandler);
    this.postStepCallbacks = undefined;
  };
 
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
    
    addCollisionHandler: function(a, b, begin, presolve, postsolve, separate, data){
      // Remove any old function so the new one will get added.
      this.removeCollisionHandler(a,b);
      var handler = new CollisionHandler( a, b, begin||alwaysCollide, presolve||alwaysCollide, postsolve||nothing, separate||nothing, data); // TODO
      this.collisionHandlers.insert( hash_pair(a,b), handler, undefined, function(handler, unused){
        // Transformation function for collisionHandlers.
        return handler.clone();
      }); 
      
    },
    
    removeCollisionHandler: function(a, b){
      var ids = {
        a: a,
        b: b
      };
      var oldHandler = this.collisionHandlers.remove( hash_pair(a,b), ids ); // TODO
    },
    
    setDefaultCollisionHandler: function(begin, preSolve, postSolve, separate, data){
      var handler = new CollisionHandler( 0, 0, begin || alwaysCollide, preSolve || alwaysCollide, postSolve||nothing, separate||nothing, data );
      this.defaultHandler = handler;
      this.collisionHandlers.setDefaultValue(this.defaultHandler);
    },
    
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
    
    addStaticShape: function(shape){
      var body = shape.body;
      body.addShape(shape);
      shape.update( body.p, body.rot );
      //this.staticShapes.insert( shape, shape.hashid );
      this.staticShapes.push(shape);
      shape.space = this;
      return shape;
    },
    addBody: function(body){
      this.bodies.push(body);
      body.space = this;
      return body;
    },
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
      var arb = body.arbiterList;
      while( arb ){
        var next = arb.next(body);
        if( filter === undefined || filter === arb.a || filter === arb.b ){
          if( arb.state != Arbiter.State.cached ){
            arb.callSeparate(this);
          }
          arb.unthread();
          this.uncacheArbiter(arb);
          //this.pooledArbiters.push(arb);
        }
        arb = next;
      }
      // TODO see note at cpSpaceArbiterSetFilter()
      // When just removing the body, so we need to filter all cached arbiters to avoid dangling pointers.
      /*
      struct arbiterFilterContext {
        cpSpace *space;
        cpBody *body;
      };
      */
      if( filter === undefined ){
        var context = {
          space: space,
          body: body
        };
        this.cachedArbiters.filter( this.filterRemovedBody, context ); 
      }
    },
    
    filterRemovedBody: function( arb, context ){
      var body = context.body;
      if( body === arb.body_a || body === arb.body_b ){
        arrays.deleteObj( context.space.arbiters, arb );
        context.space.arbiters.push(arb);
        return false;
      }
      return true;
    },
    
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
    
    removeStaticShape: function(shape){
      var body = shape.body;
      body.activateStatic(shape);
      body.removeShape(shape);
      this.filterArbiters(body, shape);
      this.staticShapes.remove(shape, shape.hashid);
      shape.space = undefined;
    },
    
    removeBody: function(body){
      body.activate();
      this.filterArbiters(body, undefined);
      arrays.deleteObj(this.bodies, body);
      body.space = undefined;
    },
    
    removeConstraint: function(constraint){
      constraint.a.activate();
      constraint.b.activate();
      arrays.deleteObj(this.constraints, constraint);
      constraint.a.removeConstraint(constraint);
      constraint.b.removeConstraint(constraint);
      constraint.space = undefined;
    },
    
    // TODO: maybe point all three methods to the same function, since they behave the same
    containsShape: function(shape){
      return (shape.space === this);
    },
    containsBody: function(body){
      return (body.space === this);
    },
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
        waking.num =0;
        this.runPostStepCallbacks();
      }
    },
    
    eachBody: function(func, data){
      this.lock();
      
      var bodies = this.bodies;
      for( var i = 0; i < bodies.length; ++i ){
        func(bodies[i], data);
      }
      var components = this.sleepingComponents;
      for( var i = 0; i < components.length; ++i ){
        var root = components[i];
        for(var var1 = root; var1; var1 = var1.node.next){
          func(body, data);
        }
      }
      
      this.unlock();
    },
    eachShapeIterator: function(shape, context){
      context.func(shape, context.data);
    },
    
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
    
    eachConstraint: function(func, data){
      this.lock();
      var constraints = this.constraints;
      for( var i = 0; i < constraints.length; ++i ){
        func(constraints[i], data);
      }
      this.unlock(true);
    },
    
    reindexStatic: function(){
      this.staticShapes.each( Shape.prototype.updateBBCache, undefined );
      this.staticShapes.reindex();
    },
    
    reindexShape: function(shape){
      var body = shape.body;
      shape.update(body.p, body.rot);
      // attempt to rehash the shape in both hashes
      this.activeShapes.reindexObject(shape, shape.hashid);
      this.staticShapes.reindexObject(shape, shape.hashid);
    },
    
    reindexShapesForBody: function(body){
      for(var var1 = body.shapeList; var1; var1 = var1.next){
        this.reindexShape(var1);
      }
    },
    
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
    
    getStaticBody: function(){
      return this.staticBody;
    },
    getCurrentTimeStep: function(){
      return this.curr_dt;
    },
    
    setIterations: function(i){
      this.iterations = i;
    },
    getIterations: function(){
      return this.iterations;
    },
    setGravity: function(i){
      this.gravity = i;
    },
    getGravity: function(){
      return this.gravity;
    },
    setDamping: function(d){
      this.damping = d;
    },
    getDamping: function(){
      return this.damping;
    },
    setIdleSpeedThreshold: function(s){
      this.idleSpeedThreshold = s;
    },
    getIdleSpeedThreshold: function(){
      return this.idleSpeedThreshold;
    },
    setSleepTimeThreshold: function(s){
      this.sleepTimeThreshold = s;
    },
    getSleepTimeThreshold: function(){
      return this.sleepTimeThreshold;
    },
    setCollisionSlop: function(s){
      this.collisionSlop = s;
    },
    getCollisionSlop: function(){
      return this.collisionSlop;
    },
    setCollisionBias: function(b){
      this.collisionBias = b;
    },
    getCollisionBias: function(){
      return this.collisionBias;
    },
    setCollisionPersistence: function(p){
      this.collisionPersistence = p;
    },
    getCollisionPersistence: function(){
      return this.collisionPersistence;
    },
    setEnableContactGraph: function(g){
      this.enableContactGraph = g;
    },
    getEnableContactGraph: function(){
      return this.enableContactGraph;
    },
    setUserData: function(u){
      this.data = u;
    },
    getUserData: function(){
      return this.data;
    },
    
    step: function(dt){ // TODO: check that method again. Probably lots of errors
      if( dt === 0 ){ 
        return;
      }
      var space = this;
      var prev_dt = space.curr_dt;
      space.curr_dt = dt;
      
      // reset and empty the arbiter list.
      var arbiters = space.arbiters;
      for( var i = 0; i < arbiters.length; ++i ){
        var arb = arbiters[i];
        //arb.state = Arbiter.State.normal;
        // If both bodies are awake, unthread the arbiter from the contact graph
        if( !arb.body_a.isSleeping() && !arb.body_b.isSleeping() ){
          arb.unthread();
        }
      }
      space.arbiters = arbiters = [];
      
      // Integrate positions
      var bodies = space.bodies;
      for( var i = 0; i < bodies.length; ++i ){
        var body = bodies[i];
        body.position_func(dt);
      }
      // Find colliding pairs.
      space.lock(); {
        space.pushFreshContactBuffer();
        space.activeShapes.each(Shape.prototype.updateFunc, undefined);
        space.activeShapes.reindexQuery(this.collideShapes, space); // TODO ?
      } 
      space.unlock(false);
      
      if( space.sleepTimeThreshold !== Number.POSITIVE_INFINITY || space.enableContactGraph ){
        space.processComponents(dt);
      }
      space.cachedArbiters.filter( Space.prototype.arbiterSetFilter, space ); // TODO ?
      /*
      space.cachedArbiters.filter(function(arb){
        var ticks = space.stamp - arb.stamp;
        
        // was used last frame, but not this one
        if(ticks >= 1 && arb.state != Arbiter.State.Cached){
          arb.handler.separate(arb);
          arb.state = Arbiter.State.Cached;
        }
        
        if(ticks >= space.collisionPersistence){
          arb.contacts = [];
          return false;
        }
        
        return true;
      });
      */
      
      
      var slop = space.collisionSlop;
      var biasCoef = 1 - Math.pow( space.collisionBias, dt );
      
      for( var i = 0; i < arbiters.length; ++i ){ // TODO: arbiters type??
        arbiters[i].preStep(dt, slop, biasCoef);
      }
      var constraints = space.constraints;
      for( var i = 0; i  < constraints.length; ++i ){
        var constraint = constraints[i];
        constraint.preStep(dt);
      }
      // Integrate velocities.
      var damping = Math.pow(space.damping, dt);
      var gravity = space.gravity;
      for( var i = 0; i < bodies.length; ++i ){
        var body = bodies[i];
        body.velocity_func(gravity, damping, dt);
      }
      // Apply cached impulses
      var dt_coef = (space.stamp? dt/prev_dt : 0);
      for( var i = 0; i < arbiters.length; ++i ){
        arbiters[i].applyCachedImpulse(dt_coef);
      }
      for( var i = 0; i < constraints.length; ++i ){
        var constraint = constraints[i];
        constraint.applyCachedImpulse(dt_coef);
      }
      // Run the impulse solver
      for( var i = 0; i < space.iterations; ++i ){
        for( var j = 0; j <   arbiters.length; ++j ){
          arbiters[j].applyImpulse();
        }
        for( var j = 0; j < constraints.length; ++j ){
          var constraint = constraints[j];
          constraint.applyImpulse();
        }
      }
      // Run the post-solve callbacks
      space.lock();
      for( var i = 0; i < arbiters.length; ++i ){
        var arb = arbiters[i];
        var handler = arb.handler;
        handler.postSolve.call(space, handler.data);
      }
      space.unlock();
      space.stamp++;
    },
    
    
    
    // TODO: do I need contact buffers?
    pushFreshContactBuffer: function(){
      var stamp = this.stamp;
      var head = this.contactBuffersHead;
      if( !head ){
        this.contactBuffersHead = new ContactBuffer(stamp, undefined);
      }else if( head.next && ((stamp - head.next.stamp) > this.collisionPersistence) ){
        // The tail buffer is available, rotate the ring
        var tail = head.next;
        this.contactBuffersHead = ContactBuffer.call(tail, stamp, tail);
      }else{
        // Allocate a new buffer and push it into the ring
        var buffer = new ContactBuffer(stamp, head);
        this.contactBuffersHead = head.next = buffer;
      }
    },
    
    getArray: function(){
      if( false ){ // TODO: if(space->contactBuffersHead->numContacts + CP_MAX_CONTACTS_PER_ARBITER > CP_CONTACTS_BUFFER_SIZE){
        this.pushFreshContactBuffer();
      }
      var head = this.contactBuffersHead;
      return head.contacts; // TODO: not sure if that is right
    },
    
    pushContacts: function(count){
      this.contactBuffersHead.numContacts += count;
    },
    
    popContacts: function(count){
      this.contactBuffersHead.numContacts -= count;
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
      arb.update(contacts, /*numContacts,*/ handler, a, b); 
      
      // Call the begin function first if it's the first step
      if( arb.state === Arbiter.State.firstColl && !handler.begin(arb, space, handler.data) ){
        arb.ignore(); // permanently ignore the collision until separation
      }
      if( (arb.state != Arbiter.State.ignore) && // Ignore the arbiter if it has been flagged
          handler.preSolve(arb, space, handler.data) && // Call preSolve
          !sensor ){ // Process, but don't add collisions for sensors.
        space.arbiters.push(arb);
      }else{
        space.popContacts(numContacts);
        arb.contacts = undefined;
        arb.numContacts = 0;
        // Normally arbiters are set as used after calling the post-solve callback.
        // However, post-solve callbacks are not called for sensors or arbiters rejected from pre-solve.
        if( arb.state != Arbiter.State.ignore ){
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
    
    arbiterSetTrans:  function(shapes, space){ // NOTE: space not used right now.. because there is no object pooling (yet).
      // TODO: maybe do some caching too..
      return new Arbiter( shapes[0], shapes[1] );
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
    }

    //TODO:
    // uncacheArbiter
  };
 
  return Space;
});
