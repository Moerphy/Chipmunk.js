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
     * Returns true if @c other lies completely within @c bb.
     */
    containsBB: function( other ){
      return (this.l <= other.l && this.r >= other.r && this.b <= other.b && this.t >= other.t);
    },
    
    /**
     * Returns true if @c bb contains @c v.
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
     * Returns a bounding box that holds both @c bb and @c v.
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
     * Merges @c a and @c b and returns the area of the merged bounding box.
     */
    mergedArea: function(b){
      return (Math.max(this.r, b.r) - Math.min(this.l, b.l))*(Math.max(this.t, b.t) - Math.min(this.b, b.b));
    },
    
    /**
     * Return true if the bounding box intersects the line segment with ends @c a and @c b.
     */
    intersectsSegment: function(a, b){
      var seg_bb = new BB(Math.min(a.x, b.x), Math.min(a.y, b.y), Math.max(a.x, b.x), Math.max(a.y, b.y));
      if(this.intersects(seg_bb)){
        var axis = new Vect(b.y - a.y, a.x - b.x);
        var offset = new Vect( (a.x + b.x - this.r - this.l), (a.y + b.y - this.t - this.b) );
        var extents = new Vect(this.r - this.l, this.t - this.b);
        
        return (Math.abs(axis.dot(offset)) < Math.abs(axis.x*extents.x) + Math.abs(axis.y*extents.y));
      }
      
      return false;
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
  
  return BB;
});
