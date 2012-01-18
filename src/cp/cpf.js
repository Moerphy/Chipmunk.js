/** @namespace cp */
define([], function(){
  "use strict";
  /**
   * @object cpf
   */
  return {
    /** @namespace cp.cpf */
    /**
     * @function clamp
     * @param {number} f
     * @param {number} min
     * @param {number} max
     */
    clamp: function(f, min, max){
      return Math.min(Math.max(f, min), max);
    },
    /**
     * @function clamp01
     * @param {number} f
     */
    clamp01: function(f){
      return Math.max(0, Math.min(f,1));
    },
    /**
     * @function lerp
     * @param {number} f1
     * @param {number} f2
     * @param {number} t
     */
    lerp: function(f1, f2, t){
      return f1*(1-t) + f2*t;
    },
    /**
     * @function lerpconst
     * @param {number} f1
     * @param {number} f2
     * @param {number} d
     */
    lerpconst: function(f1, f2, d){
      return f1 + this.clamp(f2 - f1, -d, d);
    }
  };
  
});
