define(['cp/Vect', 'cp/constraints/util', 'cp/cpf', 'cp/Array'], function(Vect, util, cpf, arrays){
  "use strict";
  /// @defgroup cpArbiter cpArbiter
  /// The cpArbiter struct controls pairs of colliding shapes.
  /// They are also used in conjuction with collision handler callbacks
  /// allowing you to retrieve information on the collision and control it.
  var vdot = function(x1, y1, x2, y2){
    return x1*x2 + y1*y2;
  };
  
  var counter = 0;

  var MAX_CONTACTS_PER_ARBITER = 10;
  /// @private
  var ArbiterState = {
    // Arbiter is active and its the first collision.
    firstColl : 0,
    // Arbiter is active and its not the first collision.
    normal: 1,
    // Collision has been explicitly ignored.
    // Either by returning false from a begin collision handler or calling cpArbiterIgnore().
    ignore: 2,
    // Collison is no longer active. A space will cache an arbiter for up to cpSpace.collisionPersistence more steps.
    cached : 3
  };
  
  var Arbiter = function(a, b){
    if( a && b ){
      this.swappedColl = false;
      this.e = 0;
      this.u = 0;
      this.surface_vr = Vect.zero;
      this.numContacts = 0;
      
      this.a = a;
      this.body_a = a.body;
      this.b = b;
      this.body_b = b.body;
      
      this.counter = ++counter;

      
      this.stamp = 0;
      this.state = ArbiterState.firstColl;
      
      this.contacts = [];
      
      this.thread_a = {};
      this.thread_b = {};
    }
  };
 
  Arbiter.State = ArbiterState;
 
 /*
  * TODO
cpBool cpArbiterIsFirstContact(const cpArbiter *arb)
{
        return arb->CP_PRIVATE(state) == cpArbiterStateFirstColl;
}

int cpArbiterGetCount(const cpArbiter *arb)
{
        // Return 0 contacts if we are in a separate callback.
        return (arb->CP_PRIVATE(state) != cpArbiterStateCached ? arb->CP_PRIVATE(numContacts) : 0);
}
  * TODO: .numCount -> .getCount()
  */ 
  
 
  Arbiter.prototype = {
    getNormal: function(i){
      if( 0 <= i && i < this.numContacts ){
        var n = this.contacts[i].n;
        return this.swappedColl? n.neg() : n;
      }
    },
    
    getPoint: function(i){
      if( 0 <= i && i < this.numContacts ){
        return this.contacts[i].p;
      }
    },
    
    getDepth: function(i){
      if( 0 <= i && i < this.numContacts ){
        return this.contacts[i].dist;
      }
    },
    
    getContactPointSet: function(){
      var count = this.getCount();
      
      var set = { 
        count: count,
        points: []  
      };
      
      
      for( var i = 0; i < count; ++i ){
        set.points[i].point = this.contacts[i].p;
        set.points[i].normal = this.contacts[i].n;
        set.points[i].dist = this.contacts[i].dist;
      }
      
      return set;
    },
    
    totalImpulse: function(){
      var contacts = this.contacts;
      var sum = Vect.zero;
      
      for( var i = 0, count = this.numContacts; i < count; ++i ){
        var con = contacts[i];
        sum = sum.add( con.n.mult(con.jnAcc) );
      }
      return (this.swappedColl ? sum : sum.neg());
    },
    
    totalImpulseWithFriction: function(){
      var contacts = this.contacts;
      var sum = Vect.zero;
      for( var i = 0, count = this.numContacts; i < count; ++i ){
        var con = contacts[i];
        sum = sum.add( con.n.rotate( new Vect(con.jnAcc, con.jtAcc) ) );
      }
      return (this.swappedColl ? sum : sum.neg());
    },
    
    ignore: function(){
      this.state = ArbiterState.ignore;
    },
    
    update: function(contacts, handler, a, b ){
      // Arbiters without contact data may exist if a collision function rejected the collision.
      // note: probably not here, but just to be sure.
        // Iterate over the possible pairs to look for hash value matches.
      for( var i = 0; this.contacts && i < this.contacts.length; ++i ){
        var old = this.contacts[i];
        for( var j = 0; j < contacts.length; ++j ){
          var newContact = contacts[j];
          // This could trigger false positives, but is fairly unlikely nor serious if it does.
          if( newContact.hash === old.hash ){
            // Copy the persistant contact information.
            newContact.jnAcc = old.jnAcc;
            newContact.jtAcc = old.jtAcc;
          }
        }
      }
      this.contacts = contacts;
      this.numContacts = contacts.length;
      this.handler = handler;
      this.swappedColl = (a.collision_type != handler.a);
      this.e = a.e * b.e;
      this.u = a.u * b.u;
      this.surface_vr = a.surface_v.sub(b.surface_v);
      // For collisions between two similar primitive types, the order could have been swapped.
      this.a = a;
      this.body_a = a.body;
      this.b = b;
      this.body_b = b.body;
      // mark it as new if it's been cached
      if( this.state === ArbiterState.cached ){
        this.state = ArbiterState.firstColl;
      }
    },
    
    preStep: function(dt, slop, bias){
      var a = this.body_a;
      var b = this.body_b;
      for( var i = 0; this.contacts && i < this.contacts.length; ++i ){
        var con = this.contacts[i];
        // Calculate the offsets.
        con.r1 = con.p.sub(a.p);
        con.r2 = con.p.sub(b.p);
        // Calculate the mass normal and mass tangent.
        con.nMass = 1/util.k_scalar( a, b, con.r1, con.r2, con.n );
        con.tMass = 1/util.k_scalar( a, b, con.r1, con.r2, con.n.perp() );
        // Calculate the target bias velocity.
        con.bias = -bias*Math.min(0, con.dist + slop)/dt;
        con.jBias = 0;
        // Calculate the target bounce velocity.
        con.bounce = util.normal_relative_velocity( a, b, con.r1, con.r2, con.n )*this.e;
      }
    },
    
    applyCachedImpulse: function(dt_coef){
      if( this.isFirstContact() ){
        return;
      }
      var a = this.body_a;
      var b = this.body_b;
      for( var i =0 ; this.contacts && i < this.numContacts; ++i ){
        var con = this.contacts[i];
        if( con ){
          var j = con.n.rotate( new Vect(con.jnAcc, con.jtAcc) );
          util.apply_impulses( a, b, con.r1, con.r2, j.mult(dt_coef) );
        }
      }
    },

    applyImpulse: function(){
      //if (!this.contacts) { throw new Error('contacts is undefined'); }
      var a = this.body_a;
      var b = this.body_b;
      var surface_vr = this.surface_vr;
      var friction = this.u;

      for(var i=0; i<this.contacts.length; i++){
        var con = this.contacts[i];
        var nMass = con.nMass;
        var n = con.n;
        var r1 = con.r1;
        var r2 = con.r2;
        
        //var vr = relative_velocity(a, b, r1, r2);
        var vrx = b.v.x - r2.y * b.w - (a.v.x - r1.y * a.w);
        var vry = b.v.y + r2.x * b.w - (a.v.y + r1.x * a.w);
        
        //var vb1 = vadd(vmult(vperp(r1), a.w_bias), a.v_bias);
        //var vb2 = vadd(vmult(vperp(r2), b.w_bias), b.v_bias);
        //var vbn = vdot(vsub(vb2, vb1), n);

        var vbn = n.x*(b.v_bias.x - r2.y * b.w_bias - a.v_bias.x + r1.y * a.w_bias) + n.y*(r2.x*b.w_bias + b.v_bias.y - r1.x * a.w_bias - a.v_bias.y);

        var vrn = vdot(vrx, vry, n.x, n.y);
        //var vrt = vdot(vadd(vr, surface_vr), vperp(n));
        var vrt = vdot(vrx + surface_vr.x, vry + surface_vr.y, -n.y, n.x);
        
        var jbn = (con.bias - vbn)*nMass;
        var jbnOld = con.jBias;
        con.jBias = Math.max(jbnOld + jbn, 0);
        
        var jn = -(con.bounce + vrn)*nMass;
        var jnOld = con.jnAcc;
        con.jnAcc = Math.max(jnOld + jn, 0);
        
        var jtMax = friction*con.jnAcc;
        var jt = -vrt*con.tMass;
        var jtOld = con.jtAcc;
        con.jtAcc = cpf.clamp(jtOld + jt, -jtMax, jtMax);
        
        //apply_bias_impulses(a, b, r1, r2, vmult(n, con.jBias - jbnOld));
        var bias_x = n.x * (con.jBias - jbnOld);
        var bias_y = n.y * (con.jBias - jbnOld);
        util.apply_bias_impulse(a, new Vect(-bias_x, -bias_y), r1);
        util.apply_bias_impulse(b, new Vect(bias_x, bias_y), r2);

        //apply_impulses(a, b, r1, r2, vrotate(n, new Vect(con.jnAcc - jnOld, con.jtAcc - jtOld)));
        var rot_x = con.jnAcc - jnOld;
        var rot_y = con.jtAcc - jtOld;

        util.apply_impulses(a, b, r1, r2, new Vect(n.x*rot_x - n.y*rot_y, n.x*rot_y + n.y*rot_x) );
      }
    },
    
    
    
    getElasticity: function(){
      return this.e;
    },
    setElasticity: function(e){
      this.e = e;
    },
    getFriction: function(){
      return this.u;
    },
    setFriction: function(u){
      this.u = u;
    },
    getSurfaceVelocity: function(){
      return this.surface_vr;
    },
    setSurfaceVelocity: function(s){
      this.surface_vr = s;
    },
    
    getCount: function(){
      return this.contacts.length;
    },
    
    /// Return the colliding shapes involved for this arbiter.
    /// The order of their cpSpace.collision_type values will match
    /// the order set when the collision handler was registered.
    getShapes: function(){
      // A macro shortcut for defining and retrieving the shapes from an arbiter.
      // #define CP_ARBITER_GET_SHAPES(arb, a, b) cpShape *a, *b; cpArbiterGetShapes(arb, &a, &b);
      var shapes;
      if( this.swappedColl ){
        shapes = {
          a: this.b,
          b: this.a
        };
      }else{
        shapes = {
          a: this.a,
          b: this.b
        };
      }
      return shapes;
    },
    
    /// Return the colliding bodies involved for this arbiter.
    /// The order of the cpSpace.collision_type the bodies are associated with values will match
    /// the order set when the collision handler was registered.
    getBodies: function(){
      var shapes = this.getShapes();
      return {
        a: shapes.a.body,
        b: shapes.b.body
      };
    },
    
    /// Returns true if this is the first step a pair of objects started colliding.
    isFirstContact: function(){
      return this.state === ArbiterState.firstColl;
    },
    
    callSeparate: function(space){
      // The handler needs to be looked up again as the handler cached on the arbiter may have been deleted since the last step.
      var handler = space.lookupHandler(this.a.collision_type, this.b.collision_type);
      handler.separate(this, space, handler.data);
    },
    
    filterRemovedBody: function(context){
      var body = context.body;
      if( body === this.body_a || body === this.body_b){
        arrays.deleteObj( context.space.arbiters, this );
        context.space.pooledArbiters.push(this);
        return false;
      }
      return true;
    },

    totalKE: function(){
      var eCoef = (1 - this.e) / (1+this.e);
      var sum = 0;
      var contacts = this.contacts;
      
      for( var i = 0; i < contacts.length; ++i ){
        var con = contacts[i];
        var jnAcc = con.jnAcc;
        var jtAcc = con.jtAcc;
        sum += eCoef * jnAcc * jnAcc / con.nMass  +  jtAcc * jtAcc / con.tMass;
      }
      return sum;
    },
    
    threadForBody: function(body){
      return (this.body_a === body) ? this.thread_a : this.thread_b;
    },
    
    next: function(body){
      return (this.body_a === body ? this.thread_a.next : this.thread_b.next);
    },
    
    unthread: function(){
      unthreadHelper( this, this.body_a );
      unthreadHelper( this, this.body_b );
    }
  };
 
  var unthreadHelper = function(arb, body){
    var thread = arb.threadForBody(body);
    var prev = thread.prev;
    var next = thread.next;
    
    if( prev ){
      prev.threadForBody(body).next = next;
    }else{
      body.arbiterList = next;
    }
    
    if( next ){
      next.threadForBody(body).prev = prev;
    }
    
    thread.prev = undefined;
    thread.next = undefined;
  };

 
  return Arbiter;
});
