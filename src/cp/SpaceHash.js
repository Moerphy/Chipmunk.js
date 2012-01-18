define(['cp/HashSet', 'cp/SpatialIndex', 'cp/constraints/util', 'cp/Prime'], function(HashSet, SpatialIndex, util, prime){
  "use strict";
  // Helper functions:
  
  // The hash function itself.
  var hash_func = function(x, y, n){
    return (x*1640531513 ^ y*2654435789) % n;
  };
  // Much faster than (int)floor(f)
  // Profiling showed floor() to be a sizable performance hog
  var floor_int = function(f) {
    var i = ~~f;
    return (f < 0.0 && f != i ? i - 1 : i);
  };
    
  var rehash_helper = function(hand, hash){
    hash.handle(hand, hash.bbfunc.call(hand.obj));
  };
  var segmentQuery_helper = function(hash, /* array */ bin_ptr, obj, func, data){
    var t = 1;
    
    // let's hope that this works...
    var inline = function(){
      for( var i = 0; bin_ptr && i < bin_ptr.length; ++i ){
        var hand = bin_ptr[i];
        var other = hand.obj;
        
        // Skip over certain conditions
        if( hand.stamp === hash.stamp ){
          continue;
        }else if( other ){
          t = Math.min(t, func(obj, other, data));
          hand.stamp = hash.stamp;
        }else{
          // The object for this handle has been removed
          // cleanup this cell and restart the query
          remove_orphaned_handles(hash, bin_ptr);
          inline(); // since we don't have goto try recursion..
          break;
        }
      }
    };
    inline();
    
    return t;
  };
  var eachHelper = function(hand, context){
    context.func.call(hand.obj, context.data);
  };
  var remove_orphaned_handles = function(hash, /* array */ bin_ptr){
    for( var i = 0; i < bin_ptr; ++i ){
      var hand = bin_ptr[i];

      if( !hand.obj ){ // TODO: check again after rewrite, no unlinking til then.
        // orphaned handle, unlink and recycle the bin
        //hash.recycleBin(bin);
        hand.release(hash.pooledHandles);
      }
    }
  };
  var query_helper = function(/* SpaceHash */ hash, /* array */ bin_ptr, obj, func, data){
    for( var i = 0; bin_ptr && i < bin_ptr.length; ++i ){
      var hand = bin_ptr[i];
      var other = hand.obj;
      
      if( hand.stamp === hash.stamp || obj === other ){
        continue;
      }else if( other ){
        func.call(obj, other, data);
        hand.stamp = hash.stamp;
      }else{
        // The object for this handle has been removed
        // cleanup this cell and restart the query
        remove_orphaned_handles(hash, bin_ptr);
        // no goto, do that with recursion..
        query_helper.apply(this, arguments);
        break;
      }
    }
  };
  var queryRehash_helper = function(hand, context){
    // typedef struct queryRehashContext {	cpSpaceHash *hash;	cpSpatialIndexQueryFunc func; 	void *data;}
    var hash = context.hash;
    var func = context.func;
    var data = context.data;
    var dim = hash.celldim;
    var n = hash.numcells;
    var obj = hand.obj;
    var bb = hash.bbfunc.call(obj);
    var l = floor_int(bb.l/dim);
    var r = floor_int(bb.r/dim);
    var b = floor_int(bb.b/dim);
    var t = floor_int(bb.t/dim);
    var table = hash.table;
    
    for(var i=l; i<=r; i++){
      for(var j=b; j<=t; j++){
        var idx = hash_func(i,j,n);
        var bin = table[idx];
        
        if( containsHandle(bin, hand) ){
          continue;
        }
        hand.retain(); // this MUST be done first in case the object is removed in func()
        query_helper(hash, bin, obj, func, data);
        
        var newBin = hash.getBin(idx);
        newBin.push(hand);
      }
    }
    // Increment the stamp for each object hashed.
    hash.stamp++;
  };
  
  /*
  struct cpHandle {
    void *obj;
    int retain;
    cpTimestamp stamp;
  };
  */
  var Handle = function(obj){
    this.obj = obj;
    this._retain = 0;
    this.stamp = 0;
  };
  Handle.prototype = {
    retain: function(){
      this._retain++;
    },
    release: function(pooledHandles){
      this._retain--;
      if( this._retain === 0 ){
        pooledHandles.push(this);
      }
    },
    setEql: function(obj){
      return obj === this.obj;
    }
  };
  Handle.setTrans = function(obj, hash){
    var hand = new Handle(obj);
    hand.retain();
    return hand;
  };
  
  var containsHandle = function(list, hand){
    for( var i = 0; list && i < list.length; ++i ){
      var bin = list[i];
      if( bin === hand ){
        return true;
      }
    }
    return false;
  };
 
  
  /*
    struct cpSpaceHash {
    cpSpatialIndex spatialIndex;
    
    int numcells;
    cpFloat celldim;
    
    cpSpaceHashBin **table;
    cpHashSet *handleSet;
    
    cpSpaceHashBin *pooledBins;
    cpArray *pooledHandles;
    cpArray *allocatedBuffers;
    
    cpTimestamp stamp;
  };
  */
  var SpaceHash = function(celldim, numcells, bbfunc, staticIndex){
    SpatialIndex.call(this, bbfunc, staticIndex);
    this.allocTable(prime.next());
    this.celldim = celldim;
    this.handleSet = new HashSet( 0, Handle.prototype.setEql  );
    this.pooledHandles = [];
    //this.pooledBins = undefined;
    this.allocatedBuffers = []; // TODO: probably can remove this. Keep for compat. reasons.
    this.stamp = 1;
  };
  
  SpaceHash.prototype = util.extend( new SpatialIndex(), {

    clearTableCell: function(idx){
      //var bin = this.table[idx];
      //bin.length = 0; // empty array
      this.table[idx] = undefined;
    },
    clearTable: function(){
      //for( var i = 0; i < this.numcells; ++i ){
      for( var i in this.table ){
        if( this.table.hasOwnProperty(i) ){
          this.clearTableCell(i);
        }
      }
    },
    getBin: function(idx){ // TODO: check if pooling (like C-Chipmunk does) brings any benefits here.
      var bin = this.table[idx];
      if( !bin ){
        bin = this.table[idx] = [];
      }
      return bin;
    },
    allocTable: function(numcells){
      this.numcells = numcells;
      this.table = {};
    },
    handle: function(hand, bb){
      // Find the dimensions in cell coordinates.
      var dim = this.celldim;
      var l = floor_int(bb.l/dim); // Fix by ShiftZ
      var r = floor_int(bb.r/dim);
      var b = floor_int(bb.b/dim);
      var t = floor_int(bb.t/dim);
      var n = this.numcells;
      for(var i=l; i<=r; i++){
        for(var j=b; j<=t; j++){
          var idx = hash_func(i,j,n);
          var bin = this.table[idx];
          // Don't add an object twice to the same cell.
          if(containsHandle(bin, hand)){
            continue;
          }
          hand.retain();
          // Insert a new bin for the handle in this cell.
          var newBin = this.getBin(idx);
          newBin.push(hand);
          //this.table[idx] = newBin;
        }
      }
    },


    insert: function(obj, hashid){
      var hand = this.handleSet.insert(hashid, obj, this, Handle.setTrans );
      this.handle(hand, this.bbfunc.call(obj));
    },
    
    rehashObject: function(obj, hashid){
      var hand = this.handleSet.remove(hashid, obj);
      if( hand ){
        hand.obj = undefined;
        hand.release(this.pooledHandles);
        this.insert(obj, hashid);
      }
    },
    
    rehash: function(){
      this.clearTable();
      this.handleSet.each(rehash_helper, this);
    },
    
    remove: function(obj, hashid){
      var hand = this.handleSet.remove(hashid, obj);
      if( hand ){
        hand.obj = undefined;
        hand.release(this.pooledHandles);
      }
    },
    
    each: function(func, data){
      // eachContext:
      // typedef struct eachContext { cpSpatialIndexIteratorFunc func; void *data;}
      var context = {
        func: func,
        data: data
      };
      this.handleSet.each(eachHelper, context);
    },
    
    pointQuery: function(point, func, data){
      var dim = this.celldim;
      var idx = hash_func(floor_int(point.x/dim), floor_int(point.y/dim), this.numcells);
      
      query_helper(this, this.table[idx], point, func, data);
      this.stamp++;
    },
    
    query: function(obj, bb, func, data){
      // Get the dimensions in cell coordinates.
      var dim = this.celldim;
      var l = floor_int(bb.l/dim);  // Fix by ShiftZ
      var r = floor_int(bb.r/dim);
      var b = floor_int(bb.b/dim);
      var t = floor_int(bb.t/dim);
      var n = this.numcells;
      
      var table = this.table;
      // Iterate over the cells and query them.
      for(var i=l; i<=r; i++){
        for(var j=b; j<=t; j++){
          query_helper(this, table[hash_func(i,j,n)], obj, func, data);
        }
      }
      this.stamp++;
    },
    
    reindexQuery: function(func, data){
      this.clearTable();
      // typedef struct queryRehashContext {	cpSpaceHash *hash; 	cpSpatialIndexQueryFunc func;	void *data; }
      var context = {
        hash: this,
        func: func,
        data: data
      };
      this.handleSet.each(queryRehash_helper, context);
      this.collideStatic(func, data);  // TODO: ?
    },
    // modified from http://playtechs.blogspot.com/2007/03/raytracing-on-grid.html
    segmentQuery: function(obj, a, b, t_exit, func, data){
      a = a.mult(1/this.celldim);
      b = b.mult(1/this.celldim);
      var cell_x = floor_int(a.x);
      var cell_y = floor_int(a.y);
      
      var t = 0;
      var x_inc;
      var y_inc;
      var temp_v;
      var temp_h;
      
      if( b.x > a.x ){
        x_inc = 1;
        temp_h = ~~(a.x + 1) - a.x;
      }else{
        x_inc = -1;
        temp_h = a.x - ~~(a.x);
      }
      if( b.y > a.y ){
        y_inc = 1;
        temp_v = ~~(a.y + 1) - a.y;
      }else{
        y_inc = -1;
        temp_v = a.y - ~~(a.y);
      }
      // Division by zero is *very* slow on ARM
      var dx = Math.abs(b.x - a.x);
      var dy = Math.abs(b.y - a.y);
      var dt_dx = (dx ? 1/dx: Number.INFINITY);
      var dt_dy = (dy ? 1/dy : Number.INFINITY);
      
      // fix NANs in horizontal directions
      var next_h = (temp_h ? temp_h*dt_dx : dt_dx);
      var next_v = (temp_v ? temp_v*dt_dy : dt_dy);
      
      var n = this.numcells;
      var table = this.table;
      
      while( t < t_exit ){
        var idx = hash_func(cell_x, cell_y, n);
        t_exit = Math.min(t_exit, segmentQuery_helper(this, table[idx], obj, func, data));
        
        if( next_v < next_h ){
          cell_y += y_inc;
          t = next_v;
          next_v += dt_dy;
        }else{
          cell_x += x_inc;
          t = next_h;
          next_h += dt_dx;
        }
      }
      this.stamp++;
    },
    
    resize: function(celldim, numcells){
     this.clearTable();
     this.celldim = celldim;
     this.allocTable(prime.next(numcells));
    },
    
    count: function(){
      return this.handleSet.count();
    },
    
    contains: function(obj, hashid){
      return this.handleSet.find(hashid, obj) != undefined;
    }

  });

  return SpaceHash;
});

