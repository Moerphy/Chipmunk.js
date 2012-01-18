define(['cp/Vect'], function(Vect){
  "use strict";
  return {
    relative_velocity: function(a, b, r1, r2 ){
      var v1Sum = a.v.add( r1.perp().mult(a.w) );
      var v2Sum = b.v.add( r2.perp().mult(b.w) );
      return v2Sum.sub(v1Sum);
    },

    normal_relative_velocity: function(a, b, r1, r2, n){
      return this.relative_velocity(a, b, r1, r2).dot(n);
    },
    
    apply_impulse: function(body, j, r){
      body.v = body.v.add( j.mult(body.m_inv) );
      body.w += body.i_inv * r.cross(j);
    },
    
    apply_impulses: function(a, b, r1, r2, j){
      a.applyImpulse(j.neg(), r1);
      b.applyImpulse(j, r2); 
    },
    
    apply_bias_impulse: function(body, j, r){
      body.v_bias = body.v_bias.add( j.mult(body.m_inv) );
      body.w_bias += body.i_inv * r.cross(j);
    },
    
    apply_bias_impulses: function(a, b, r1, r2, j){
      this.apply_bias_impulse(a, j.neg(), r1);
      this.apply_bias_impulse(b, j, r2);
    },
    
    k_scalar_body: function(body, r, n){
      var rcn = r.cross(n);
      return body.m_inv + body.i_inv*rcn*rcn;
    },
    
    k_scalar: function( a, b, r1, r2, n ){
      var value = this.k_scalar_body( a, r1, n ) + this.k_scalar_body(b, r2, n);
      return value;
    },
    
    k_tensor: function(a, b, r1, r2, k1k2){
      var k11, k12, k21, k22;
      
      var m_sum = a.m_inv + b.m_inv;
      k11 = m_sum;
      k12 = 0;
      k21 = 0;
      k22 = m_sum;
      
      var a_i_inv = a.i_inv;
      var r1xsq = r1.x * r1.x * a_i_inv;
      var r1ysq =  r1.y * r1.y * a_i_inv;
      var r1nxy = -r1.x * r1.y * a_i_inv;
      k11 += r1ysq; 
      k12 += r1nxy;
      k21 += r1nxy; 
      k22 += r1xsq;
      
      // add the influnce from r2
      var b_i_inv = b.i_inv;
      var r2xsq =  r2.x * r2.x * b_i_inv;
      var r2ysq =  r2.y * r2.y * b_i_inv;
      var r2nxy = -r2.x * r2.y * b_i_inv;
      k11 += r2ysq; 
      k12 += r2nxy;
      k21 += r2nxy; 
      k22 += r2xsq;
      
      // invert
      var determinant = k11*k22 - k12*k21;
      
      var det_inv = 1/determinant;
      k1k2.k1 = new Vect( k22*det_inv, -k12*det_inv);
      k1k2.k2 = new Vect(-k21*det_inv,  k11*det_inv);
    },
    
    mult_k: function(vr, k1, k2){
      return new Vect( vr.dot(k1), vr.dot(k2) );
    },
    
    bias_coef: function(errorBias, dt){
      return 1 - Math.pow(errorBias, dt);
    },
    
    extend: function(object, extension){
      for( var i in extension ){
        if( extension.hasOwnProperty(i) ){
          object[i] = extension[i];
        }
      }
      return object;
    }
    
  };
  
});
