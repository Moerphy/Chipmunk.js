define(['cp/Vect', 'cp/constraints/util', 'cp/cpf', 'cp/Array', 'cp/Prime'], function(Vect, util, cpf, arrays, prime){
  "use strict";
  var HashSetBin = {};
  
  var HashSet = function(size, eqlFunc){
    this.entries = 0;
    this.eql = eqlFunc;
    this.default_value = undefined;
    // type: HashSetBin
    this.table = {};
  };
  
  HashSet.prototype = {
    setDefaultValue: function(default_value){
      this.default_value = default_value;
    },
    
    isFull: function(){
      return false; //this.entries >= this.size;
    },
    
    count: function(){
      return this.entries;
    },
    
    insert: function(hash, ptr, data, trans){
      
      var bin = this.table[hash];
      if( !bin ){
        bin = this.table[hash] = [];
      }
      
      for( var i = 0; i < bin.length; ++i ){
        if( this.eql(ptr, bin[i].elt) ){
          return bin[i].elt;
        }
      }
      
      var obj = {
        elt: (trans ? trans(ptr, data) : data),
        hash: hash
      };
      bin.push(obj);
      this.entries++;

      return obj.elt;
    },
    
    remove: function(hash, ptr){
      var bin = this.table[hash];
      if( bin ){
        for( var i = bin.length - 1; i >= 0; --i ){
          var elt = bin[i].elt;
          if( this.eql.call(elt, ptr) ){
            bin.splice(i, 1);
            this.entries--;
            return elt;
          }
        }
      }
    },
    
    find: function(hash, ptr){
      var bin = this.table[hash];
      if( bin ){
        for( var i = 0; i < bin.length; ++i ){
          if( this.eql(ptr, bin[i].elt) ){
            return bin[i].elt;
          }
        }
      }
      return this.default_value;
    },
    
    each: function(func, data){
      for( var k in this.table ){
        if( this.table.hasOwnProperty(k) ){
          var bin = this.table[k];
          for( var i = 0; bin && i < bin.length; ++i ){
            func(bin[i].elt, data);
          }
        }
      }
    },
    
    filter: function(func, data){ 
      for( var k in this.table ){
        if( this.table.hasOwnProperty(k) ){
          var bin = this.table[k];
          if( bin ){
            for( var i = bin.length-1; i >= 0; --i ){
              if( func(bin[i].elt, data) ){
                
              }else{
                bin.splice(i, 1);
                this.entries--;
              }
            }
          }
        }
      }
    }
  };
  
  
  return HashSet;
});

