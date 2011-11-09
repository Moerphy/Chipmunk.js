define([], {
  hard: function(condition, message){
    if( !condition ){
      throw message;
    }
  }
});
