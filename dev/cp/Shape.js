define(['cp/Vect', 'cp/BB', 'cp/Contact', 'cp/constraints/util', 'cp/Collision'], function(Vect, BB, Contact, util, colfuncs){
  "use strict";

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




