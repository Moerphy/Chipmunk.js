define(['cp/Vect', 'cp/BB', 'cp/Contact', 'cp/constraints/util'], function(Vect, BB, Contact, util){
  "use strict";
  var hash_pair = function(a, b){
    // #define CP_HASH_COEF (3344921057ul)
    // #define CP_HASH_PAIR(A, B) ((cpHashValue)(A)*CP_HASH_COEF ^ (cpHashValue)(B)*CP_HASH_COEF)
    var hash_coef = 3344921057;
    var arbHashID = (( a * hash_coef) ^ ( b * hash_coef))&0x7FFFFFFF;  // TODO:  find better port solution to this hashing thingy..
    return arbHashID;
  };

  // Add contact points for circle to circle collisions.
  // Used by several collision tests.
  var circle2circleQuery = function(p1, p2, r1, r2, con){ 
    var mindist = r1 + r2;
    var delta = p2.sub(p1);
    var distsq = delta.lengthsq();
    if( distsq >= mindist*mindist ){
      return 0;
    }
    var dist = Math.sqrt(distsq);
    // Allocate and initialize the contact.
    con.push( new Contact( p1.add( delta.mult( 0.5 + (r1 - 0.5*mindist)/(dist ? dist : Number.POSITIVE_INFINITY) ) ), 
                              (dist ? delta.mult(1.0/dist) : new Vect(1.0, 0.0)), 
                              dist-mindist, 
                              0 ) );
    return 1;
  };

  // Collide circle shapes.
  var circle2circle = function( circ1, circ2, arr ){
    return circle2circleQuery( circ1.tc, circ2.tc, circ1.r, circ2.r, arr );
  };

  // Collide circles to segment shapes.
  var circle2segment = function( circ, seg, con ){
    // Radius sum
    var rsum = circ.r + seg.r;
    // Calculate normal distance from segment.
    var dn = seg.tn.dot(circ.tc) - seg.ta.dot(seg.tn);
    var dist = Math.abs(dn) - rsum;
    if( dist > 0 ){
      return 0;
    }
    var dt = -seg.tn.cross(circ.tc);
    var dtMin = -seg.tn.cross(seg.ta);
    var dtMax = -seg.tn.cross(seg.tb);
    // Decision tree to decide which feature of the segment to collide with.
    if( dt < dtMin ){
      if( dt < (dtMin - rsum) ){
        return 0;
      }else{
        return circle2circleQuery(circ.tc, seg.ta, circ.r, seg.r, con);
      }
    }else{
      if( dt < dtMax ){
        var n = (dn < 0) ? seg.tn : seg.tn.neg();
        con.push( new Contact( circ.tc.add( n.mult(circ.r + dist*0.5) ), n, dist, 0 ) ); // TODO: keep an eye on this, i really fucked that port up.
        return 1;
      }else{
        if( dt < (dtMax + rsum) ){
          return circle2circleQuery( circ.tc, seg.tb, circ.r, seg.r, con );
        }else{
          return 0;
        }
      }
    }
    return 1;
  };

  // This one is complicated and gross. Just don't go there...
  // TODO: Comment me!
  var seg2poly = function( seg, poly, arr ){
    var axes = poly.tAxes;
    
    var segD = seg.tn.dot(seg.ta);
    var minNorm = poly.valueOnAxis(seg.tn, segD) - seg.r; 
    var minNeg = poly.valueOnAxis( seg.tn.neg(), -segD) - seg.r;
    if( minNeg > 0 || minNorm > 0 ){
      return 0;
    }
    var mini = 0;
    var poly_min = seg.valueOnAxis( axes[0].n, axes[0].d ); // TODO is axes[0] correctly ported?
    if( poly_min > 0 ){
      return 0;
    }
    for( var i = 0; i < poly.numVerts; ++i ){
      var dist = seg.valueOnAxis( axes[i].n, axes[i].d );
      if( dist > 0 ){
        return 0;
      }else if( dist > poly_min ){
        poly_min = dist;
        mini = i;
      }
    }
    var num = { num: 0 };
    var poly_n = axes[mini].n.neg();
    
    var va = seg.ta.add( poly_n.mult(seg.r) );
    var vb = seg.tb.add( poly_n.mult(seg.r) );
    if( poly.containsVert(va) ){
      arr.push( new Contact( va, poly_n, poly_min, hash_pair(seg.hashid, 0), Contact.nextPoint(arr, num) ) );
    }
    if( poly.containsVert(vb) ){
      arr.push( new Contact( vb, poly_n, poly_min, hash_pair(seg.hashid, 1),  Contact.nextPoint(arr, num) ) );
    }
    // Floating point precision problems here.
    // This will have to do for now.
    //	poly_min -= cp_collision_slop; // TODO is this needed anymore?
    if( minNorm >= poly_min || minNeg >= poly_min ){
      if( minNorm > minNeg ){
        Contact.findPointsBehindSeg(arr, num, seg, poly, minNorm, 1.0); 
      }else{
        Contact.findPointsBehindSeg(arr, num, seg, poly, minNeg, -1.0); 
      }
    }
    
    // If no other collision points are found, try colliding endpoints.
    if( num === 0 ){
      var poly_a = poly.tVerts[mini];
      var poly_b = poly.tVerts[(mini+1)%poly.numVerts];
      
      if(circle2circleQuery(seg.ta, poly_a, seg.r, 0, arr)){
        return 1;
      }
      if(circle2circleQuery(seg.tb, poly_a, seg.r, 0, arr)){
        return 1;
      }
      if(circle2circleQuery(seg.ta, poly_b, seg.r, 0, arr)){
        return 1;
      }
      if(circle2circleQuery(seg.tb, poly_b, seg.r, 0, arr)){
        return 1;
      }
    }
    return num.num;
  };

  // This one is less gross, but still gross.
  // TODO: Comment me!
  var circle2poly = function(circ, poly, con){
    var axes = poly.tAxes;
    var mini = 0; 
    var min = axes[0].n.dot(circ.tc) - axes[0].d - circ.r;
    for( var i = 0; i < poly.numVerts; ++i ){
      var dist = axes[i].n.dot(circ.tc) - axes[i].d - circ.r;
      if( dist > 0 ){
        return 0;
      }else if( dist > min ){
        min = dist;
        mini = i;
      }
    }
    var n = axes[mini].n;
    var a = poly.tVerts[mini];
    var b = poly.tVerts[(mini+1)%poly.numVerts];
    var dta = n.cross(a);
    var dtb = n.cross(b);
    var dt = n.cross(circ.tc);
    
    if( dt < dtb ){
      return circle2circleQuery(circ.tc, b, circ.r, 0, con);
    }else if( dt < dta ){
      con.push( new Contact( circ.tc.sub( n.mult(circ.r + min/2) ), n.neg(), min, 0 ) );
      return 1;
    }else{
      return circle2circleQuery( circ.tc, a, circ.r, 0, con);
    }
  };

  
  
  // Find the minimum separating axis for the give poly and axis list.
  var findMSA = function(poly, axes,  num, out){
    var min_index = 0;
    
    var min = poly.valueOnAxis(axes[0].n, axes[0].d);
    if( min > 0 ){
      return -1;
    }
    for( var i = 1; i < num; ++i ){
      var dist = poly.valueOnAxis(axes[i].n, axes[i].d);
      if( dist > 0 ){
        return -1;
      }else if( dist > min ){
        min = dist;
        min_index = i;
      }
    }
    
    out.min = min;
    return min_index;
  };
  
  // Add contacts for probably penetrating vertexes.
  // This handles the degenerate case where an overlap was detected, but no vertexes fall inside
  // the opposing polygon. (like a star of david)
  var findVertsFallback = function(arr, poly1, poly2, n, dist){
    var num = 0;
    
    for( var i = 0; i < poly1.numVerts; ++i ){
      var v = poly1.tVerts[i];
      if( poly2.containsVertPartial( v, n.neg() ) ){
        num++;
        arr.push( new Contact( v, n, dist, hash_pair(poly1.hashid, i)  ) );
      }
    }
    for( var i = 0; i < poly2.numVerts; ++i ){
      var v = poly2.tVerts[i];
      if( poly1.containsVertPartial(v, n) ) {
        num++;
        arr.push( new Contact( v, n, dist, hash_pair(poly2.hashid, i)  ) );
      }
    }
    
    return num;
  };

  
  // Add contacts for penetrating vertexes.
  var findVerts = function(arr, poly1, poly2, n, dist){
    var num = 0;
    
    for( var i = 0; i < poly1.numVerts; ++i ){
      var v = poly1.tVerts[i];
      if( poly2.containsVert(v) ){
        num++;
        arr.push( new Contact( v, n, dist, hash_pair(poly1.hashid, i) ) );
      }
    }
    for( var i = 0; i < poly2.numVerts; ++i ){
      var v = poly2.tVerts[i];
      if( poly1.containsVert(v) ){
        num++;
        arr.push( new Contact( v, n, dist, hash_pair(poly2.hashid, i) ) );
      }
    }
    
    return (num ? num : findVertsFallback(arr, poly1, poly2, n, dist));
  };
  
  var poly2poly = function( poly1, poly2, arr ){
    var min1 = {};
    var mini1 = findMSA(poly2, poly1.tAxes, poly2.numVerts, min1);
    if( mini1 === -1 ){
      return 0;
    }
    var min2 = {};
    var mini2 = findMSA( poly1, poly2.tAxes, poly2.numVerts, min2);
    if( mini2 === -1 ){
      return 0;
    }
    // There is overlap, find the penetrating verts
    if( min1.min > min2.min ){
      return findVerts( arr, poly1, poly2, poly1.tAxes[mini1].n, min1.min );
    }else{
      return findVerts( arr, poly1, poly2, poly2.tAxes[mini2].n.neg(), min2.min );
    }
  };



 var colfuncs = [
    circle2circle,
    undefined,
    undefined,
    circle2segment,
    undefined,
    undefined,
    circle2poly,
    seg2poly, 
    poly2poly, 
  ];


  var ShapeType = {
    CIRCLE_SHAPE: 0,
    SEGMENT_SHAPE: 1,
    POLY_SHAPE: 2,
    NUM_SHAPES: 3
  };
  
  var idCounter = 0;
  var hashCounter;

  var Shape = function(body){
    this.hashid = idCounter;
    this.hash = this.hashid; //  ~~(Math.random()*100000); //TODO: can i remove this, since it'S covered by hashid?
   
    idCounter = (idCounter + 49157)&0xFFFF;
    this.body = body;
    this.sensor = 0;
    this.e = 0;
    this.u = 0;
    this.surface_v = Vect.zero;
    this.collision_type = 0;
    this.group = 0; // CP_NO_GROUP
    this.layers = ~0;// CP_ALL_LAYERS
    
  };
  
  Shape.resetIdCounter = function(){
    idCounter = 0;
  };

  Shape.prototype = {
    setBody: function(body){
      this.body = body;
    },
    cacheBB: function(){
      var body = this.body;
      return this.update(body.p, body.rot);
    },
    update: function(pos, rot){
      return (this.bb = this.cacheData(pos, rot));
    },
    
    updateBBCache: function(){
      var body = this.body;
      this.update(body.p, body.rot);
    },  
    // function to get the estimated velocity of a shape for the cpBBTree.
    velocityFunc: function(){
      return this.body.v;
    },

    getBB: function(){
      return this.bb;
    },
    
    setSensor: function(s){
      this.body.activate();
      this.sensor = s;
    },
    getSensor: function(){
      return this.sensor;
    },
    
    setFriction: function(s){
      this.body.activate();
      this.u = s;
    },
    getFriction: function(){
      return this.u;
    },
    
    setSurfaceVelocity: function(s){
      this.body.activate();
      this.surface_v = s;
    },
    getSurfaceVelocity: function(){
      return this.surface_v;
    },
    
    setCollisionType: function(s){
      this.body.activate();
      this.collision_type = s;
    },
    getCollisionType: function(){
      return this.collision_type;
    },
    
    setGroup: function(s){
      this.body.activate();
      this.group = s;
    },
    getGroup: function(){
      return this.group;
    },
    
    setLayers: function(s){
      this.body.activate();
      this.layers = s;
    },
    getLayers: function(){
      return this.layers;
    },
    
    setElasticity: function(s){
      this.e = s;
    },
    getElasticity: function(){
      return this.e;
    },
    
    setUserData: function(s){
      this.data = s;
    },
    getUserData: function(){
      return this.data;
    },
    
    pointQuery: function(p){
      throw "Not implemented, don't use Shape class directly. Use a subclass (Circle, Segment or Polygon)";
    },
    segmentQuery: function(a,b){
      throw "Not implemented, don't use Shape class directly. Use a subclass (Circle, Segment or Polygon)"
    },
    destroy: function(){
      throw "Not implemented, don't use Shape class directly. Destroy is only implemented on POlygons."
    },
    cacheData: function(pos, rot){
      throw "Not implemented, don't use Shape class directly. Use a subclass (Circle, Segment or Polygon)"
    },
    
    copyShapes: function(index){
      index.insert(this, this.hashid);
    },
    
    updateFunc: function(unused){
      var body = this.body;
      this.update(body.p, body.rot);
    },
    
    collideShapes: function(b, arr){
      // Their shape types must be in order.
      if( this.type > b.type ){
        throw "Collision shapes passed to Shape.collideShapes() are not sorted.";
      }
      var cfunc = colfuncs[ this.type + b.type * ShapeType.NUM_SHAPES ];
      return cfunc?cfunc(this, b, arr) : 0;
    },
    
    queryReject: function(b){
      var a = this;
      return (
        // BBoxes must overlap
        !a.bb.intersects(b.bb) ||
        // Don't collide shapes attached to the same body.
        a.body === b.body ||
        // Don't collide objects in the same non-zero group
        (a.group && (a.group === b.group)) ||
        // Don't collide objects that don't share at least on layer.
        !(a.layers & b.layers)
      );
    }
    
  };
  
  
  Shape.Circle = function(body, radius, offset){
    // call super constructor
    Shape.call(this, body);
    this.c = offset;
    this.r = radius;
    this.type = ShapeType.CIRCLE_SHAPE;
  };
  
  Shape.Circle.momentFor = function(m, r1, r2, offset){
    return m*(0.5*(r1*r1 + r2*r2) + offset.lengthsq());
  };
  Shape.Circle.areaFor = function(r1, r2){
    return Math.PI*Math.abs(r1*r1 - r2*r2);
  };
  
  
  Shape.Circle.prototype = util.extend( new Shape(), {
    cacheData: function(p, rot){
      var c = this.tc = p.add( this.c.rotate(rot) );
      var r = this.r;
      return new BB( c.x-r, c.y-r, c.x+r, c.y+r );
    },
    
    pointQuery: function(p){
      return this.tc.near(p, this.r);
    },
    
    _circleSegmentQuery: function(center, r, a, b){
      // offset the line to be relative to the circle
      a = a.sub(center);
      b = b.sub(center);
      
      var qa = a.dot(a) - 2*a.dot(b) + b.dot(b);
      var qb = -2*a.dot(a) + 2*a.dot(b);
      
      var qc = a.dot(a) - r*r;
      var det = qb*qb - 4*qa*qc;
      
      if( det >= 0 ){
        var t = (-qb - Math.sqrt(det))/(2*qa);
        if( 0 <= t && t <= 1 ){
          return {
            shape: this,
            t: t,
            n: a.lerp(b, t).normalize()
          };
        }
      }
    },
    
    // wraps _circleSegmentQuery in the API for the shape
    segmentQuery: function(a, b){
      return this._circleSegmentQuery(this.tc, this.r, a, b);
    },
    
    getOffset: function(){
      return this.c;
    },
    setOffset: function(c){
      this.c = c;
    },
    
    getRadius: function(){
      return this.r;
    },
    setRadius: function(r){
      this.r = r;
    }
    
  });

  Shape.Segment = function(body, a, b, r){
    // call super constructor
    Shape.call(this, body);
    this.a = a;
    this.b = b;
    this.n = b.sub(a).normalize().perp();
    this.r = r;
    this.type = ShapeType.SEGMENT_SHAPE;
  };
  
  Shape.Segment.prototype = util.extend( new Shape(), {
    cacheData: function(p, rot){
      this.ta = p.add( this.a.rotate(rot) );
      this.tb = p.add( this.b.rotate(rot) );
      this.tn = this.n.rotate(rot);
      
      var l;
      var r;
      var b;
      var t;
      
      if( this.ta.x < this.tb.x ){
        l = this.ta.x;
        r = this.tb.x;
      }else{
        l = this.tb.x;
        r = this.ta.x;
      }
      
      if( this.ta.y < this.tb.y ){
        b = this.ta.y;
        t = this.tb.y;
      }else{
        b = this.tb.y;
        t = this.ta.y;
      }
      var rad = this.r;
      return new BB( l - rad, b - rad, r + rad, t + rad );
    },
    
    pointQuery: function(p){
      if( !this.bb.containsVect(p) ){
        return false;
      }
      // Calculate normal distance from segment.
      var dn = this.tn.dot(p) - this.ta.dot(this.tn);
      var dist = Math.abs(dn) - this.r;
      if( dist > 0 ){
        return false;
      }
      // Calculate tangential distance along segment.
      var dt = -this.tn.cross(p);
      var dtMin = -this.tn.cross(this.ta);
      var dtMax = -this.tn.cross(this.tb);
      
      // Decision tree to decide which feature of the segment to collide with.
      if(dt <= dtMin){
        if(dt < (dtMin - this.r)){
          return false;
        } else {
          return (this.ta.sub(p).lengthsq() < (this.r*this.r) );
        }
      } else {
        if(dt < dtMax){
          return true;
        } else {
          if(dt < (dtMax + this.r)) {
            return ( this.tb.sub(p).lengthsq() < (this.r*this.r) );
          } else {
            return false;
          }
        }
      }
      return true;	
    },
    
    segmentQuery: function(a, b){
      var n = this.tn;
      // flip n if a is behind the axis
      if( this.a.dot(n) < this.ta.dot(n) ){
        n = n.neg();
      }
      var an = a.dot(n);
      var bn = b.dot(n);
      
      if( an != bn ){
        var d = this.ta.dot(n) + this.r;
        var t = (d-an)/(bn-an);
        if( 0 < t && t < 1 ){
          var point = a.lerp(b, t);
          var dt = -this.tn.cross(point);
          var dtMin = -this.tn.cross( this.ta );
          var dtMax = -this.tn.cross( this.tb );
          if( dtMin < dt && dt < dtMax ){
            return { // return QueryInfo
              shape: this,
              t: t,
              n: n
            }; // don't continue on and check endcaps
          }
        }
      }
      if( this.r ){
        var info1 = {
          shape: undefined,
          t: 1,
          n: Vect.zero
        };
        var info2 = {
          shape: undefined,
          t: 1,
          n: Vect.zero
        };
        info1 = Shape.Circle.prototype._circleSegmentQuery.call(this, this.ta, this.r, a, b);
        info2 = Shape.Circle.prototype._circleSegmentQuery.call(this, this.tb, this.r, a, b);
        if( info1.t < info2.t ){
          return info1;
        }else{
          return info2;
        }
      }
    },
    
    setEndpoints: function(a, b){
      this.a = a;
      this.b = b;
      this.n = b.sub(a).normalize().perp();
    },
    
    setRadius: function(radius){
      this.r = radius;
    },
    getRadius: function(){
      return this.r;
    },
    
    getNormal: function(){
      return this.n;
    },
    getB: function(){
      return this.b;
    },
    getA: function(){
      return this.a;
    },
    
    valueOnAxis: function(n, d){
      var a = n.dot(this.ta) - this.r;
      var b = n.dot(this.tb) - this.r;
      return Math.min(a,b) - d;
    }
    
  });
  
  
  /**
   * @param body {object} a cpBody object
   * @param verts {array} array of cpVect objects (determines the length, drops the numVerts param)
   * @param offset {object} cpVect object
   */
  Shape.Polygon = function(body, verts, offset){
    if( !this.validate(verts) ){
      throw "Polygon is concave or has a reversed winding.";
    }
    this.setUpVerts(verts, offset);
    Shape.call(this, body);
    this.type = ShapeType.POLY_SHAPE;
  };
  Shape.Polygon.prototype = util.extend(new Shape(), {
    transformVerts: function(p, rot){
      var src = this.verts;
      var dst = this.tVerts;
      var l = Number.POSITIVE_INFINITY;
      var r = Number.NEGATIVE_INFINITY;
      var b = Number.POSITIVE_INFINITY;
      var t = Number.NEGATIVE_INFINITY;
      
      for( var i = 0; i < this.numVerts; ++i ){
        var v = p.add( src[i].rotate(rot) );
        dst[i] = v;
        l = Math.min(l, v.x);
        r = Math.max(r, v.x);
        b = Math.min(b, v.y);
        t = Math.max(t, v.y);
      }
      return new BB(l, b, r, t);
    },
    
    transformAxes: function(p, rot){
      var src = this.axes;
      var dst = this.tAxes;
      for( var i = 0; i < this.numVerts; ++i ){
        var n = src[i].n.rotate(rot);
        dst[i].n = n;
        dst[i].d = p.dot(n) + src[i].d;
      }
    },
    
    cacheData: function(p, rot){
      this.transformAxes(p, rot);
      var bb = this.bb = this.transformVerts(p, rot);
      return bb;
    },
    
    destroy: function(){ // TODO check if i could just drop this method
      this.verts = undefined;
      this.tVerts = undefined;
      this.axes = undefined;
      this.tAxes = undefined;
    },
    
    pointQuery: function(p){
      return this.bb.containsVect(p) && this.containsVert(p);
    },
    
    segmentQuery: function(a, b){
      var axes = this.tAxes;
      var verts = this.tVerts;
      var numVerts = this.numVerts;
      for( var i = 0; i < numVerts; ++i ){
        var n = axes[i].n;
        var an = a.dot(n);
        if( axes[i].d > an ){
          continue;
        }
        var bn = b.dot(n);
        var t = (axes[i].d - an)/(bn-an);
        if( t < 0 || 1 < t ){
          continue;
        }
        var point = a.lerp(b, t);
        var dt = -n.cross(point);
        var dtMin = -n.cross(verts[i]);
        var dtMax = -n.cross( verts[(i+1)%numVerts] );
        if( dtMin <= dt && dt <= dtMax ){
          return {
            shape: this,
            t: t,
            n: n
          };
        }
      }
    },
    validate: function(verts){
      var numVerts = verts.length;
      for( var i = 0; i < numVerts; ++i ){
        var a = verts[i];
        var b = verts[(i+1)%numVerts];
        var c = verts[(i+2)%numVerts];
        if( b.sub(a).cross( c.sub(b) ) > 0 ){
          return false;
        }
      }
      return true;
    },
    
    getNumVerts: function(){
      return this.numVerts;
    },
    
    getVert: function(idx){
      if( idx < 0 || idx >= this.getNumVerts() ){
        throw "Index out of range.";
      }
      return this.verts[idx];
    },
    setUpVerts: function(verts, offset){
      this.numVerts = verts.length;
      this.verts = [];
      this.tVerts = [];
      this.axes = [];
      this.tAxes = [];
      for( var i = 0; i < this.numVerts; ++i ){
        var a = offset.add( verts[i] );
        var b = offset.add( verts[(i+1)%this.numVerts] );
        var n = b.sub(a).perp().normalize();
        this.verts[i] = a;
        this.axes[i] = {
          n: n,
          d: n.dot(a)
        };
        this.tAxes[i] = {}; //TODO: is that right? :>
      }
    },
    setVerts: function(verts, offset){
      this.setUpVerts(verts, offset);
    },
    
    valueOnAxis: function(n, d){
      var verts = this.tVerts;
      var min = n.dot(verts[0]);
      
      for( var i = 1; i < this.numVerts; ++i ){
        min = Math.min(min, n.dot(verts[i]));
      }
      
      return min - d;
    },
    
    containsVert: function(v){
      var axes = this.tAxes;
      
      for( var i = 0; i < this.numVerts; ++i ){
        var dist = axes[i].n.dot(v) - axes[i].d;
        if( dist > 0 ){
          return false;
        }
      }
      
      return true;
    },
    
    containsVertPartial: function(v, n){
      var axes = this.tAxes;
      
      for( var i = 0; i < this.numVerts; ++i ){
        if( axes[i].n.dot(n) < 0 ){
          continue;
        }
        var dist = axes[i].n.dot(v) - axes[i].d;
        if( dist > 0 ){
          return false;
        }
      }
      
      return true;
    }
    
  });


  Shape.Box = function(body, widthOrBox, height){
    var box;
    if( arguments.length === 3 ){ // constructor: body, width, height
      var hw = widthOrBox / 2;
      var hh = height / 2;
      box = new BB( -hw, -hh, hw, hh );
    }else{
      box = widthOrBox;
    }
    var verts = [
      new Vect(box.l, box.b),
      new Vect(box.l, box.t),
      new Vect(box.r, box.t),
      new Vect(box.r, box.b)
    ];
    return new Shape.Polygon(body, verts, Vect.zero);
  };
  
  Shape.Box.momentFor = function(m, width, height){
    return m*(width*width + height*height)/12;
  };
  
  return Shape;
});




