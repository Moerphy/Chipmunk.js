define([], {

  indexOf: function(array, obj){
    if( array.indexOf ){
      return array.indexOf(obj);
    }
    for (var i = 0; i < array.length; ++i) {  
      if( array[i] === obj ){
        return i;
      }
    }  
    return -1;  
  },

  deleteObj: function(array, obj){
    var pos = this.indexOf(array, obj);
    if( pos >= 0 ){
      array.splice(pos, 1);
    }
  },
  
  contains: function(array, obj){
    return this.indexOf(array, obj) >= 0;
  }
});



