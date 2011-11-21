define([], function(){
  var hash_pair = function(a, b){
    // #define CP_HASH_COEF (3344921057ul)
    // #define CP_HASH_PAIR(A, B) ((cpHashValue)(A)*CP_HASH_COEF ^ (cpHashValue)(B)*CP_HASH_COEF)
    var hash_coef = 3344921057;
    var arbHashID = ( ( a * hash_coef) ^ ( b * hash_coef) ) & 0x7FFFFFFF ;
    return arbHashID;
  };  
    
  return hash_pair;
});
  
