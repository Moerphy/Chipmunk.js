define(['cp/Vect', /*'chipmunk/private'*/], function(Vect, cpPrivate){
  "use strict";
  /**
   * Chipmunk's axis-aligned 2D bounding box type along with a few handy routines.
   */
  var BB = function(l,b,r,t){
    this.l = l;
    this.b = b;
    this.r = r;
    this.t = t;
  };
  
  BB.prototype = {
    
    /**
     * Returns true if a and b intersect.
     * @param a {object}
     * @param b {object}
     * @return {boolean}
     */
    intersects: function(b){
      return (this.l <= b.r && b.l <= this.r && this.b <= b.t && b.b <= this.t);
    },	

    /**
     * Returns true if other lies completely within bb.
     */
    containsBB: function( other ){
      return (this.l <= other.l && this.r >= other.r && this.b <= other.b && this.t >= other.t);
    },
    
    /**
     * Returns true if bb contains v.
     */
    containsVect: function(v){
      return (this.l <= v.x && this.r >= v.x && this.b <= v.y && this.t >= v.y);
    },
    
    /**
     * Returns a bounding box that holds both bounding boxes.
     */
    merge: function(b){
      return new BB(
        Math.min(this.l, b.l),
        Math.min(this.b, b.b),
        Math.max(this.r, b.r),
        Math.max(this.t, b.t)
      );
    },
    
    /**
     * Returns a bounding box that holds both c bb and c v.
     */
    expand: function( v ){
      return new BB(
        Math.min(this.l, v.x),
        Math.min(this.b, v.y),
        Math.max(this.r, v.x),
        Math.max(this.t, v.y)
      );
    },
    
    area: function(){
      return (this.r - this.l)*(this.t - this.b);
    },
    
    /**
     * Merges  a and  b and returns the area of the merged bounding box.
     */
    mergedArea: function(b){
      return (Math.max(this.r, b.r) - Math.min(this.l, b.l))*(Math.max(this.t, b.t) - Math.min(this.b, b.b));
    },
    
    /**
     * @function segmentQuery
     */
    segmentQuery: function(a, b){
      var idx = 1 / (b.x - a.x);
      var tx1 = (this.l === a.x ? -Number.POSITIVE_INFINITY : (this.l - a.x)*idx);
      var tx2 = (this.r === a.x ?  Number.POSITIVE_INFINITY : (this.r - a.x)*idx);
      var txmin = Math.min(tx1, tx2);
      var txmax = Math.max(tx1, tx2);
      
      var idy = 1/(b.y - a.y);
      var ty1 = (this.b === a.y ? -Number.POSITIVE_INFINITY : (this.b - a.y)*idy);
      var ty2 = (this.t === a.y ?  Number.POSITIVE_INFINITY : (this.t - a.y)*idy);
      var tymin = Math.min(ty1, ty2);
      var tymax = Math.max(ty1, ty2);
      
      if( tymin <= txmax && txmin <= tymax ){
        var min = Math.max(txmin, tymin);
        var max = Math.min(txmax, tymax);
        
        if( 0 <= max && min <= 1.0 ){
          return Math.max(min, 0);
        }
      }
      return Number.POSITIVE_INFINITY;
    },
    /**
     * @function intersectsSegment
     */
    intersectsSegment: function(a, b){
      return this.segmentQuery(a, b) !== Number.POSITIVE_INFINITY;
    },

    /**
     * Clamp a vector to a bounding box.
     */
    clampVect: function(v) {
      var x = Math.min(Math.max(this.l, v.x), this.r);
      var y = Math.min(Math.max(this.b, v.y), this.t);
      return new Vect(x, y);
    },
    
    wrapVect: function(v){
      var ix = Math.abs(this.r - this.l);
      var modx = (v.x - this.l) % ix;
      var x = (modx > 0) ? modx : modx + ix;
      
      var iy = Math.abs(this.t - this.b);
      var mody = (v.y - this.b) % iy;
      var y = (mody > 0) ? mody : mody + iy;
      
      return new Vect(x + this.l, y + this.b);
    }
  };
  /**
   * Constructs a cpBB for a circle with the given position and radius.
   * @param {cp.Vect} p
   * @param {number} r
   */
  BB.forCircle = function(p, r){
    return new BB(p.x - r, p.y - r, p.x + r, p.y + r);
  };
  
  return BB;
});
