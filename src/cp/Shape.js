/** 
 * @namespace cp
 */
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

  /**
   * Opaque collision shape class. Used as baseclass for Circle-, Segment- and PolyShapes
   * @class Shape
   */
  var Shape = function(body){
    this.hashid = idCounter;
    this.hash = this.hashid; 
   
    idCounter = (idCounter + 49157)&0xFFFF;
    this.body = body;
    this.sensor = 0;
    this.e = 0;
    this.u = 0;
    this.surface_v = Vect.zero;
    this.collision_type = 0;
    this.group = 0; // CP_NO_GROUP
    this.layers = ~0;// CP_ALL_LAYERS
    
    this.next = undefined;
    this.prev = undefined;
    this.data = undefined;
    this.space = undefined;
  };
  
  Shape.resetIdCounter = function(){
    idCounter = 0;
  };
  /** @namespace cp.Shape.prototype */
  Shape.prototype = {
    /**
     * The rigid body the shape is attached to. Can only be set when the shape is not added to a space.
     * @function setBody
     * @param {cp.Body} body
     */
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
    
    active: function(){
      return this.prev || (this.body && this.body.shapeList === this);
    },

    /**
     * The bounding box of the shape. Only guaranteed to be valid after cpShapeCacheBB() or cpSpaceStep() is called. Moving a body that a shape is connected to does not update it’s bounding box. For shapes used for queries that aren’t attached to bodies, you can also use cpShapeUpdate().
     * @function getBB
     * @return {cp.BB}
     */
    getBB: function(){
      return this.bb;
    },
    /**
     * A boolean value if this shape is a sensor or not. Sensors only call collision callbacks, and never generate real collisions
     * @function setSensor
     * @param {boolean} s
     */
    setSensor: function(s){
      this.body.activate();
      this.sensor = s;
    },
    /**
     * A boolean value if this shape is a sensor or not. Sensors only call collision callbacks, and never generate real collisions
     * @function getSensor
     * @return {boolean}
     */
    getSensor: function(){
      return this.sensor;
    },
    /**
     * Friction coefficient. Chipmunk uses the Coulomb friction model, a value of 0.0 is frictionless. The friction for a collision is found by multiplying the friction of the individual shapes together. Tables of friction coefficients.
     * @function setFriction
     * @param {number} f
     */
    setFriction: function(s){
      this.body.activate();
      this.u = s;
    },
    /**
     * Friction coefficient. Chipmunk uses the Coulomb friction model, a value of 0.0 is frictionless. The friction for a collision is found by multiplying the friction of the individual shapes together. Tables of friction coefficients.
     * @function getFriction
     * @return {number} 
     */
    getFriction: function(){
      return this.u;
    },
    /**
     * The surface velocity of the object. Useful for creating conveyor belts or players that move around. This value is only used when calculating friction, not resolving the collision.
     * @function setSurfaceVelocity
     * @param {number} s
     */
    setSurfaceVelocity: function(s){
      this.body.activate();
      this.surface_v = s;
    },
    /**
     * The surface velocity of the object. Useful for creating conveyor belts or players that move around. This value is only used when calculating friction, not resolving the collision.
     * @function getSurfaceVelocity
     * @return {number}
     */
    getSurfaceVelocity: function(){
      return this.surface_v;
    },
    
    /**
     * You can assign types to Chipmunk collision shapes that trigger callbacks when objects of certain types touch.
     * @function setCollisionType
     * @param {object} c
     */
    setCollisionType: function(s){
      this.body.activate();
      this.collision_type = s;
    },
    /**
     * You can assign types to Chipmunk collision shapes that trigger callbacks when objects of certain types touch.
     * @function getCollisionType
     * @return {object}
     */
    getCollisionType: function(){
      return this.collision_type;
    },
    /**
     * Shapes in the same non-zero group do not generate collisions. Useful when creating an object out of many shapes that you don’t want to self collide. Defaults to 0.
     * @function setGroup
     * @param {number} g
     */
    setGroup: function(s){
      this.body.activate();
      this.group = s;
    },
    /**
     * Shapes in the same non-zero group do not generate collisions. Useful when creating an object out of many shapes that you don’t want to self collide. Defaults to 0.
     * @function setGroup
     * @return {number}
     */
    getGroup: function(){
      return this.group;
    },
    /**
     * Shapes only collide if they are in the same bit-planes. i.e. (a->layers & b->layers) != 0 By default, a shape occupies all bit-planes. Wikipedia has a nice article on bitmasks if you are unfamiliar with how to use them. Defaults to CP_ALL_LAYERS.
     * @function setLayers
     * @param {number} l
     */
    setLayers: function(s){
      this.body.activate();
      this.layers = s;
    },
    /**
     * Shapes only collide if they are in the same bit-planes. i.e. (a->layers & b->layers) != 0 By default, a shape occupies all bit-planes. Wikipedia has a nice article on bitmasks if you are unfamiliar with how to use them. Defaults to CP_ALL_LAYERS.
     * @function getLayers
     * @return {number}
     */
    getLayers: function(){
      return this.layers;
    },
    /**
     * @function setElasticity
     * @param {number} e
     */
    setElasticity: function(s){
      this.e = s;
    },
    /**
     * @function getElasticity
     * @return {number}
     */
    getElasticity: function(){
      return this.e;
    },
    
    /**
     * @function setUserData
     * @param {object} s
     */
    setUserData: function(s){
      this.data = s;
    },
    /**
     * @function getUserData
     * @return {object}
     */
    getUserData: function(){
      return this.data;
    },
    
    getSpace: function(){
      return this.space;
    },
    
    /**
     * @function pointQuery
     */
    pointQuery: function(p){
      throw "Not implemented, don't use Shape class directly. Use a subclass (Circle, Segment or Polygon)";
    },
    /**
     * @function segmentQuery
     */
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
  
  /** @namespace cp.Shape */
  
  /**
   * Fastest and simplest collision shape.
   * @class Circle
   * @implements {cp.Shape}
   */
  Shape.Circle = function(body, radius, offset){
    // call super constructor
    Shape.call(this, body);
    this.c = offset;
    this.r = radius;
    this.type = ShapeType.CIRCLE_SHAPE;
  };
  /**
   * @function Circle.momentFor
   * @param {number} mass
   * @param {number} radius1 inner diameter of circle (0 for solid circle)
   * @param {number} radius2 outer diameter of circle
   * @param {cp.Vect} offset
   */
  Shape.Circle.momentFor = function(m, r1, r2, offset){
    return m*(0.5*(r1*r1 + r2*r2) + offset.lengthsq());
  };
  /**
   * Area of a hollow circle
   * @function Circle.areaFor
   * @param {number} r1
   * @param {number} r2
   */
  Shape.Circle.areaFor = function(r1, r2){
    return Math.PI*Math.abs(r1*r1 - r2*r2);
  };
  
  /**
   * @namespace cp.Shape.Circle.prototype
   */
  Shape.Circle.prototype = util.extend( new Shape(), {
    cacheData: function(p, rot){
      var c = this.tc = p.add( this.c.rotate(rot) );
      return new BB.forCircle( c, this.r );
    },
    
    /**
     * @function pointQuery
     * @param {cp.Vect} p
     */
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
    
    /**
     * @function getOffset
     */
    getOffset: function(){
      return this.c;
    },
    /**
     * @function @setOffset
     * @param {cp.Vect} o
     */
    setOffset: function(c){
      this.c = c;
    },
    /**
     * @function getRadius
     * @return {number}
     */
    getRadius: function(){
      return this.r;
    },
    /**
     * @function setRadius
     * @param {number} r
     */
    setRadius: function(r){
      this.r = r;
    }
    
  });

  /**
   * Meant mainly as a static shape. They can be attached to moving bodies, but they don’t currently generate collisions with other line segments. Can be beveled in order to give them a thickness.
   * @namespace cp.Shape
   * @class Segment
   * @param {cp.Body} body
   * @param {cp.Vect} a point a of line segment
   * @param {cp.Vect} b point b of line segment
   * @param {number} r Thickness of line
   */
  Shape.Segment = function(body, a, b, r){
    // call super constructor
    Shape.call(this, body);
    this.a = a;
    this.b = b;
    this.n = b.sub(a).normalize().perp();
    this.r = r;
    this.type = ShapeType.SEGMENT_SHAPE;
    
    this.a_tangent = Vect.zero;
    this.b_tangent = Vect.zero;
  };
  /** @namespace cp.Shape.Segment.prototype */
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
      var d = this.ta.sub(a).dot(n);
      var r = this.r;
      
      var flipped_n = (d > 0? n.neg() : n );
      var seg_offset = flipped_n.mult(r).sub(a);

      // Make the endpoints relative to 'a' and move them by the thickness of the segment.
      var seg_a = this.ta.add(seg_offset);
      var seg_b = this.tb.add(seg_offset);
      var delta = b.sub(a);
      
      if( delta.cross(seg_a) * delta.cross(seg_b) <= 0 ){
        var d_offset = d + (d > 0? -r : r);
        var ad = -d_offset;
        var bd = delta.dot(n) - d_offset;
        
        if( ad * bd < 0 ){
          return {
            shape: this,
            t: ad/(ad-bd),
            n: flipped_n
          };
        }
      }else  if( r !== 0 ){
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
    /**
     * @function setEndpoints
     * @param {cp.Vect} a
     * @param {cp.Vect} b
     */
    setEndpoints: function(a, b){
      this.a = a;
      this.b = b;
      this.n = b.sub(a).normalize().perp();
    },
    /**
     * @function setRadius
     * @param {number} radius
     */
    setRadius: function(radius){
      this.r = radius;
    },
    /**
     * @function getRadius
     * @return {number}
     */
    getRadius: function(){
      return this.r;
    },
    /**
     * @function getNormal
     */
    getNormal: function(){
      return this.n;
    },
    /**
     * @function getB
     * @return {cp.Vect}
     */
    getB: function(){
      return this.b;
    },
    /**
     * @function getA
     * @return {cp.Vect}
     */
    getA: function(){
      return this.a;
    },
    
    valueOnAxis: function(n, d){
      var a = n.dot(this.ta) - this.r;
      var b = n.dot(this.tb) - this.r;
      return Math.min(a,b) - d;
    },
    
    setNeighbors: function(prev, next){
      this.a_tangent = prev.sub(this.a);
      this.b_tangent = next.sub(this.b);
    }
  });

  /**
   * @namespace cp.Shape.Segment
   * @function momentFor
   * @param {number} mass
   * @param {cp.Vect} a
   * @param {cp.Vect} b
   * @return {number} moment of this shape
   */
  Shape.Segment.momentFor = function(mass, a, b){
    var length = b.sub(a).length();
    var offset = a.add(b).mult( 0.5 );

    return mass*(length*length/12 + offset.lengthsq());
  };
  
  
  /**
   * @namespace cp.Shape
   * @class Polygon
   * @param {cp.Body} body a cpBody object
   * @param {array} verts array of cpVect objects (determines the length, drops the numVerts param)
   * @param {cp.Vect} cpVect offset
   */
  Shape.Poly = function(body, verts, offset){
    if( !this.validate(verts) ){
      throw "Polygon is concave or has a reversed winding.";
    }
    this.setUpVerts(verts, offset);
    Shape.call(this, body);
    this.type = ShapeType.POLY_SHAPE;
  };
  /**
   * @namespace cp.Shape.Poly.prototype
   */
  Shape.Poly.prototype = util.extend(new Shape(), {
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
    /**
     * Get number of verts of this polygon
     * @function getNumVerts
     * @return {number}
     */
    getNumVerts: function(){
      return this.numVerts;
    },
    /**
     * @function getVert
     * @param {number} index
     */
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
  
  /**
   * @namespace cp.Shape.Poly
   * @function momentFor
   * @param {number} mass
   * @param {array} verts array of cp.Vect objects
   * @param {cp.Vect} offset
   */
  Shape.Poly.momentFor = function(m, verts, offset){
    var sum1 = 0;
    var sum2 = 0;
    
    for( var i = 0, l = verts.length; i < l; ++i ){
      var v1 = verts[i].add(offset);
      var v2 = verts[(i+1)%l].add(offset);
      
      var a = v2.cross(v1);
      var b = v1.dot(v1) + v1.dot(v2) + v2.dot(v2);
      
      sum1 += a*b;
      sum2 += a;
    }
    return (m*sum1) / (6*sum2);
  };
  
  /**
   * @namespace cp.Shape.Poly
   * @function areaFor
   * @param {array} verts array of cp.Vect objects
   */
  Shape.Poly.areaFor = function(verts){
    var area = 0;
    
    for( var i = 0, l = verts.lengt; i < l; ++i ){
      area += verts[i].cross( verts[(i+1)%l] );
    }
    
    return - area / 2;
  };

  /**
   * @namespace cp.Shape.Poly
   * @function centroidFor
   * @param {array} verts array of cp.Vect objects
   */
  Shape.Poly.centroidFor = function(verts){
    var sum = 0;
    var vsum = Vect.zero;
    
    for( var i = 0, l = verts.lengt; i < l; ++i ){
      var v1 = verts[i];
      var v2 = verts[(i+1)%l];
      var cross = v1.cross(v2);
      
      sum += cross;
      vsum = vsum.add( v1.add(v2).mult(cross) );
    }
    
    return vsum.mult( 1 / (3*sum) );
  };


  /**
   * Because boxes are so common in physics games, Chipmunk provides shortcuts to create box shaped polygons. The boxes will always be centered at the center of gravity of the body you are attaching them to.
   * @namespace cp.Shape
   * @class Box
   * @param {cp.Body} body the body to attach this to
   * @param {number|cp.BB} widthOrBox either the width of the box or a cp.BB box
   * @param {number} [height=widthOrBox.height] height of the box. optional if second parameter was a bounding box.
   */
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
    return new Shape.Poly(body, verts, Vect.zero);
  };
  /**
   * @namespace cp.Shape.Box
   * @function momentFor
   * @param {number} m
   * @param {number} width
   * @param {height} height
   */
  Shape.Box.momentFor = function(m, width, height){
    return m*(width*width + height*height)/12;
  };
  
  define( 'cp/shape/Circle', function(){ return Shape.Circle; } );
  define( 'cp/shape/Segment', function(){ return Shape.Segment; } );
  define( 'cp/shape/Poly', function(){ return Shape.Poly; } );
  define( 'cp/shape/Box', function(){ return Shape.Box; } );
  
  return Shape;
});
