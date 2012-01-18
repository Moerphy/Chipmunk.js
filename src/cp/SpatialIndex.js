define(['cp/Prime'], function(prime){
  "use strict";
  // helper functions:
  var dynamicToStaticIter = function(/* obj, */ context){ // NOTE: obj = this
    context.staticIndex.query( this, context.bbfunc.call(this), context.queryFunc, context.data );
  };
  

  var SpatialIndex = function(bbfunc, staticIndex){
    this.bbfunc = bbfunc;
    this.staticIndex = staticIndex;
    if( staticIndex ){
      if( staticIndex.dynamicIndex ){
        throw "This static index is already is already associated with a dynamic index.";
      }
      staticIndex.dynamicIndex = this;
    }
  };
  
  SpatialIndex.prototype = {
    collideStatic: function(func, data){
      if( this.count() > 0 ){
        /*
        typedef struct dynamicToStaticContext {
          cpSpatialIndexBBFunc bbfunc;
          cpSpatialIndex *staticIndex;
          cpSpatialIndexQueryFunc queryFunc;
          void *data;
        } */
        var context = {
          bbfunc: this.bbfunc,
          staticIndex: this.staticIndex,
          queryFunc: func,
          data: data
        };
        this.each(dynamicToStaticIter, context);
      }
    }
  };
  
  return SpatialIndex;
});

