define(['cp/Vect', 'cp/constraints/util', 'cp/cpf', 'cp/Array', 'cp/Prime'], function(Vect, util, cpf, arrays, prime){
  "use strict";
  var HashSetBin = {};
  
  var HashSet = function(size, eqlFunc){
    this.size = prime.next(size);
    this.entries = 0;
    this.eql = eqlFunc;
    this.default_value = undefined;
    // type: HashSetBin
    this.table = {};//Array(this.size);
    this.pooledBins = undefined;
    this.allocatedBuffers = [];
  };
  
  HashSet.prototype = {
    setDefaultValue: function(default_value){
      this.default_value = default_value;
    },
    
    isFull: function(){
      return this.entries >= this.size;
    },
    
    resize: function(){
      // Get the next approximate doubled prime.
      var newSize = prime.next(this.size+1);
      // Allocate a new table.
      var newTable = {};
      // Iterate over the chains.
      for( var i = 0; i < this.size; i++ ){
        // Rehash the bins into the new table.
        var bin = this.table[i];
        while( bin ){
          var next = bin.next;
          var idx = bin.hash%newSize;
          bin.next = newTable[idx];
          newTable[idx] = bin;
          
          bin = next;
        }
      }
      this.table = newTable;
      this.size = newSize;
    },
    
    recycleBin: function(bin){
      bin.next = this.pooledBins;
      this.pooledBins = bin;
      bin.elt = undefined;
    },
    
    getUnusedBin: function(){
      var bin = this.pooledBins;
      if( bin ){
        this.pooledBins = bin.next;
        return bin;
      }else{
        // Pool is exhausted, make more
        var count = 100; // TODO: configurable
        var buffer = {}; // TODO
        this.allocatedBuffers.push(buffer);
        for( var i = 1; i < count; ++i ){
          this.recycleBin({});
        }
        return buffer;
      }
    },
    
    count: function(){
      return this.entries;
    },
    
    insert: function(hash, ptr, data, trans){
      var idx = hash%this.size;
      // Find the bin with the matching element.
      var bin = this.table[idx];
      while( bin && !this.eql(ptr, bin.elt) ){
        bin = bin.next;
      }
      // create it if necessary.
      if( !bin ){
        bin = this.table[idx] = {
          elt: (trans ? trans(ptr, data) : data),
          hash: hash,
          next: this.table[idx],
        };
        
        this.entries++;
        if(this.isFull()) this.resize();
        /*
        bin = this.getUnusedBin();
        bin.hash = hash;
        bin.elt = (trans ? trans(ptr, data) : data);
        bin.next = this.table[idx];
        this.table[idx] = bin;
        this.entries++;
        if( this.isFull() ){
          this.resize();
        }
        */
      }
      return bin.elt;
    },
    
    remove: function(hash, ptr){
      var idx = hash%set.size;
      var prev_ptr = this.table[idx];
      var bin = this.table[idx];
      
      // Find the bin
      while( bin && this.eql(ptr, bin.elt) ){
        prev_ptr = bin.next;
        bin = bin.next;
      }
      
      // Remove it if it exists
      if( bin ){
        // Update the previous linked list pointer
        prev_ptr = bin.ext;
        this.entries--;
        var elt = bin.elt;
        this.recycleBin(bin);
        return elt;
      }
    },
    
    find: function(hash, ptr){
      var idx = hash%this.size;
      var bin = this.table[idx];
      while( bin && !this.eql(ptr, bin.elt) ){
        bin = bin.next;
      }
      return (bin?bin.elt:this.default_value);
    },
    
    each: function(func, data){
      for( var i in this.table ){
        if( this.table.hasOwnProperty(i) ){
          var bin = this.table[i];
          while(bin){
            var next = bin.next;
            func(bin.elt, data);
            bin = next;
          }
        }
      }
    },
    
    filter: function(func, data){ // TODO: check if this works correctly (was some pointer magic)
      for( var i in this.table ){
        // The rest works similarly to cpHashSetRemove() above.
        var prev_ptr = undefined;
        var bin = this.table[i];
        while(bin){
          var next = bin.next;
          if( bin && bin.elt && func(bin.elt, data) ){
            prev_ptr = bin;
          }else{
            if( prev_ptr ){
              prev_ptr = next;
            }else{
              this.table[i] = next;
            }
            this.entries--;
           
            this.recycleBin(bin);
          }
          bin = next;
        }
      }
    }

    
  };
  
  
  return HashSet;
});

