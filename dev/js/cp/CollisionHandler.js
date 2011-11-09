/*


/// @private
struct cpCollisionHandler {
	cpCollisionType a;
	cpCollisionType b;
	cpCollisionBeginFunc begin;
	cpCollisionPreSolveFunc preSolve;
	cpCollisionPostSolveFunc postSolve;
	cpCollisionSeparateFunc separate;
	void *data;
};

 */

define([], function(){
  "use strict";  
  var CollisionHandler = function(a, b, begin, preSolve, postSolve, separate, data){
    this.a = a;
    this.b = b;
    this.begin = begin;
    this.preSolve = preSolve;
    this.postSolve = postSolve;
    this.separate = separate;
    this.data = data;
  };
  
  CollisionHandler.prototype = {
    clone: function(){
      return new CollisionHandler( this.a, this.b, this.begin, this.preSolve, this.postSolve, this.separate, this.data );
    }
  };
  
  
  return CollisionHandler;
});


