define([], function(){
  "use strict";
  
  /**
   * @param {cp/Body} a
   * @param {cp/Body} b
   */
  var Constraint = function(a, b){
    /*
     * @field {cp/Body} a The first body connected to this constraint.
     */
    this.a = a;
    /// The second body connected to this constraint.
    this.b = b;
    
    /*
     * @field {cp/constraints/Constraint} next_a
     */
    this.next_a = undefined;
    this.next_b = undefined;
    
    /// The maximum force that this constraint is allowed to use.
    /// Defaults to infinity.
    this.maxForce = Number.POSITIVE_INFINITY;
    /// The rate at which joint error is corrected.
    /// Defaults to pow(1.0 - 0.1, 60.0) meaning that it will
    /// correct 10% of the error every 1/60th of a second.
    this.errorBias = Math.pow( 1.0 - 0.1, 60 );
    /// The maximum rate at which joint error is corrected.
    /// Defaults to infinity.
    this.maxBias = Number.POSITIVE_INFINITY;
    
    this.space = undefined;
    /// User definable data pointer.
    /// Generally this points to your the game object class so you can access it
    /// when given a cpConstraint reference in a callback.
    this.data = undefined;
    
    this.preSolve = undefined;
    this.postSolve = undefined;
  };
  
  Constraint.prototype = {
    activateBodies: function(){
      var a = this.a;
      if( a ){
        a.activate();
      }
      var b = this.b;
      if( b ){
        b.activate();
      }
    },
    
    getA: function(){
      return this.a;
    },
    getB: function(){
      return this.b;
    },
    
    getMaxForce: function(){
      return this.maxForce;
    },
    setMaxForce: function(value){
      this.activateBodies();
      this.maxForce = value;
    },
    getErrorBias: function(){
      return this.errorBias;
    },
    setErrorBias: function(value){
      this.activateBodies();
      this.errorBias = value;
    },
    getMaxBias: function(){
      return this.maxBias;
    },
    setMaxBias: function(value){
      this.activateBodies();
      this.maxBias = value;
    },
    getUserData: function(){
      return this.data;
    },
    setUserData: function(value){
      this.activateBodies();
      this.data = value;
    },
    setPreSolveFunc: function(func){
      this.preSolve = func;
    },
    getPreSolveFunc: function(func){
      return this.preSolve;
    },
    setPostSolveFunc: function(func){
      this.postSolve = func;
    },
    getPostSolveFunc: function(func){
      return this.postSolve;
    }
  };
  
  return Constraint;
});
