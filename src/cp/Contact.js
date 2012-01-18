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

define(['cp/Hash'], function(hash_pair){
  "use strict";
  var MAX_CONTACTS_PER_ARBITER = 100; // TODO: configurable
  
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
  };
  
  Contact.nextPoint = function( arr, /* object */ numPtr ){
    var index = numPtr.num;
    if( index < MAX_CONTACTS_PER_ARBITER ){
      numPtr.num = index+1;
      return arr[index];
    }else{
      return arr[MAX_CONTACTS_PER_ARBITER - 1];
    }
  };

  Contact.findPointsBehindSeg = function( /* array */ arr, /* object */ num, /* SegmentShape */ seg, /* PolyShape */ poly, /* number */ pDist, /* number */ coef ){
    var dta = seg.tn.cross(seg.ta);
    var dtb = seg.tn.cross(seg.tb);
    var n = seg.tn.mult(coef);
    for( var i = 0; i < poly.numVerts; ++i ){
      var v = poly.tVerts[i];
      if( v.dot(n) < seg.tn.dot(seg.ta) * coef + seg.r ){
        var dt = seg.tn.cross(v);
        if( dta >= dt && dt >= dtb ){
          arr.push( new Contact( v, n, pDist, hash_pair(poly.hashid, i), Contact.nextPoint(arr, num ) ) );
        }
      }
    }
  };



  /*
   */

  return Contact;
});


