/*


struct cpContactBufferHeader {
	cpTimestamp stamp;
	cpContactBufferHeader *next;
	unsigned int numContacts;
};

#define CP_CONTACTS_BUFFER_SIZE ((CP_BUFFER_BYTES - sizeof(cpContactBufferHeader))/sizeof(cpContact))
typedef struct cpContactBuffer {
	cpContactBufferHeader header;
	cpContact contacts[CP_CONTACTS_BUFFER_SIZE];
} cpContactBuffer;


 */

define([], function(){
  "use strict";
  var hash_pair = function(a, b){
    // #define CP_HASH_COEF (3344921057ul)
    // #define CP_HASH_PAIR(A, B) ((cpHashValue)(A)*CP_HASH_COEF ^ (cpHashValue)(B)*CP_HASH_COEF)
    var hash_coef = 3344921057;
    var arbHashID = ( a * hash_coef) ^ ( b * hash_coef);  // TODO:  find better port solution to this hashing thingy..
    return arbHashID;
  };
  var MAX_CONTACTS_PER_ARBITER = 10; // TODO: configurable
  
  var Contact = function(/* Vect */ p, /* Vect */ n, /* number */ dist, /* number */ hash, /* optional object*/ o){
    if( !o ){
      o = this;
    }
    
    o.p = p;
    o.n = n;
    o.dist = dist;
    
    o.jnAcc = 0;
    o.jtAcc = 0;
    o.jBias = 0;
    
    o.hash = hash;
    
    return o;
  };
  
  Contact.prototype = {
    nextPoint: function( /* object */ numPtr ){
      var index = numPtr.num;
      if( index < MAX_CONTACTS_PER_ARBITER ){
        numPtr.num = index+1;
        return this[index];
      }else{
        return this[MAX_CONTACTS_PER_ARBITER - 1];
      }
    },
    
    findPointsBehindSeg: function(/*number*/ num, /* SegmentShape */ seg, /* PolyShape */ poly, /* number */ pDist, /* number */ coef ){
      var dta = seg.tn.cross(seg.ta);
      var dtb = seg.tn.cross(seg.tb);
      var n = seg.tn.mult(coef);
      for( var i = 0; i < poly.numVerts; ++i ){
        var v = poly.tVerts[i];
        if( v.dot(n) < seg.tn.dot(seg.ta) * coef + seg.r ){
          var dt = seg.tn.cross(v);
          if( dta >= dt && dt >= dtb ){
            new Contact( v, n, pDist, hash_pair(poly.shape.hashid, i), this.nextPoint({num: num}) );
          }
        }
      }
    }
  };


  /*
   */

  return Contact;
});


