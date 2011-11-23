define([], function(){
  "use strict";
  return {
    clamp: function(f, min, max){
      return Math.min(Math.max(f, min), max);
    },
    
    clamp01: function(f){
      return Math.max(0, Math.min(f,1));
    },
    
    lerp: function(f1, f2, t){
      return f1*(1-t) + f2*t;
    },
    
    lerpconst: function(f1, f2, d){
      return f1 + this.clamp(f2 - f1, -d, d);
    }
  };
  
});
