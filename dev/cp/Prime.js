define([], function(){
  "use strict";
  var primes = [
    5,
    13,
    23,
    47,
    97,
    193,
    389,
    769,
    1543,
    3079,
    6151,
    12289,
    24593,
    49157,
    98317,
    196613,
    393241,
    786433,
    1572869,
    3145739,
    6291469,
    12582917,
    25165843,
    50331653,
    100663319,
    201326611,
    402653189,
    805306457,
    1610612741
  ];
  return {
    next: function(n){
      var i = 0;
      while(n > primes[i]){
        i++;
      }
      return primes[i];
    },
    
    last: function(){
      return primes[ primes.length - 1 ];
    }
  };
});
