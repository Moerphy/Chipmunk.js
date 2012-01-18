define(['cp/Contact', 'cp/Hash', 'cp/Vect', 'cp/cpf'], function(Contact, hash_pair, Vect, cpf){
  // Add contact points for circle to circle collisions.
  // Used by several collision tests.
  var circle2circleQuery = function(p1, p2, r1, r2, con){ 
    var mindist = r1 + r2;
    var delta = p2.sub(p1);
    var distsq = delta.lengthsq();
    if( distsq >= mindist*mindist ){
      return 0;
    }
    var dist = Math.sqrt(distsq);
    // Allocate and initialize the contact.
    con.push( new Contact( p1.add( delta.mult( 0.5 + (r1 - 0.5*mindist)/(dist ? dist : Number.POSITIVE_INFINITY) ) ), 
                              (dist ? delta.mult(1.0/dist) : new Vect(1.0, 0.0)), 
                              dist-mindist, 
                              0 ) );
    return 1;
  };

  // Collide circle shapes.
  var circle2circle = function( circ1, circ2, arr ){
    return circle2circleQuery( circ1.tc, circ2.tc, circ1.r, circ2.r, arr );
  };

  // Collide circles to segment shapes.
  var circle2segment = function( circ, seg, con ){
    var seg_a = seg.ta;
    var seg_b = seg.tb;
    var center = circ.tc;
    
    var seg_delta = seg_b.sub(seg_a);
    var closest_t = cpf.clamp01( seg_delta.dot(center.sub(seg_a))/ seg_delta.lengthsq() );
    var closest = seg_a.add( seg_delta.mult(closest_t) );
    if( circle2circleQuery(center, closest, circ.r, seg.r, con) ){
      var n = con[0].n;
      // Reject endcap collisions if tangents are provided.
      if( (closest_t == 0 && n.dot(seg.a_tangent) < 0) ||
          (closest_t == 1 && n.dot(seg.b_tangent) < 0) ){
        return 0;
      }
		
      return 1;
    }else{
      return 0;
    }
  };

  // This one is complicated and gross. Just don't go there...
  // TODO: Comment me!
  var seg2poly = function( seg, poly, arr ){
    var axes = poly.tAxes;
    
    var segD = seg.tn.dot(seg.ta);
    var minNorm = poly.valueOnAxis(seg.tn, segD) - seg.r; 
    var minNeg = poly.valueOnAxis( seg.tn.neg(), -segD) - seg.r;
    if( minNeg > 0 || minNorm > 0 ){
      return 0;
    }
    var mini = 0;
    var poly_min = seg.valueOnAxis( axes[0].n, axes[0].d ); // TODO is axes[0] correctly ported?
    if( poly_min > 0 ){
      return 0;
    }
    for( var i = 0; i < poly.numVerts; ++i ){
      var dist = seg.valueOnAxis( axes[i].n, axes[i].d );
      if( dist > 0 ){
        return 0;
      }else if( dist > poly_min ){
        poly_min = dist;
        mini = i;
      }
    }
    var num = { num: 0 };
    var poly_n = axes[mini].n.neg();
    
    var va = seg.ta.add( poly_n.mult(seg.r) );
    var vb = seg.tb.add( poly_n.mult(seg.r) );
    if( poly.containsVert(va) ){
      arr.push( new Contact( va, poly_n, poly_min, hash_pair(seg.hashid, 0), Contact.nextPoint(arr, num) ) );
    }
    if( poly.containsVert(vb) ){
      arr.push( new Contact( vb, poly_n, poly_min, hash_pair(seg.hashid, 1),  Contact.nextPoint(arr, num) ) );
    }
    // Floating point precision problems here.
    // This will have to do for now.
    //	poly_min -= cp_collision_slop; // TODO is this needed anymore?
    if( minNorm >= poly_min || minNeg >= poly_min ){
      if( minNorm > minNeg ){
        Contact.findPointsBehindSeg(arr, num, seg, poly, minNorm, 1.0); 
      }else{
        Contact.findPointsBehindSeg(arr, num, seg, poly, minNeg, -1.0); 
      }
    }
    
    // If no other collision points are found, try colliding endpoints.
    if( num === 0 ){
      var poly_a = poly.tVerts[mini];
      var poly_b = poly.tVerts[(mini+1)%poly.numVerts];
      
      if(circle2circleQuery(seg.ta, poly_a, seg.r, 0, arr)){
        return 1;
      }
      if(circle2circleQuery(seg.tb, poly_a, seg.r, 0, arr)){
        return 1;
      }
      if(circle2circleQuery(seg.ta, poly_b, seg.r, 0, arr)){
        return 1;
      }
      if(circle2circleQuery(seg.tb, poly_b, seg.r, 0, arr)){
        return 1;
      }
    }
    return num.num;
  };

  // This one is less gross, but still gross.
  // TODO: Comment me!
  var circle2poly = function(circ, poly, con){
    var axes = poly.tAxes;
    var mini = 0; 
    var min = axes[0].n.dot(circ.tc) - axes[0].d - circ.r;
    for( var i = 0; i < poly.numVerts; ++i ){
      var dist = axes[i].n.dot(circ.tc) - axes[i].d - circ.r;
      if( dist > 0 ){
        return 0;
      }else if( dist > min ){
        min = dist;
        mini = i;
      }
    }
    var n = axes[mini].n;
    var a = poly.tVerts[mini];
    var b = poly.tVerts[(mini+1)%poly.numVerts];
    var dta = n.cross(a);
    var dtb = n.cross(b);
    var dt = n.cross(circ.tc);
    
    if( dt < dtb ){
      return circle2circleQuery(circ.tc, b, circ.r, 0, con);
    }else if( dt < dta ){
      con.push( new Contact( circ.tc.sub( n.mult(circ.r + min/2) ), n.neg(), min, 0 ) );
      return 1;
    }else{
      return circle2circleQuery( circ.tc, a, circ.r, 0, con);
    }
  };

  
  
  // Find the minimum separating axis for the give poly and axis list.
  var findMSA = function(poly, axes,  num, out){
    var min_index = 0;
    
    var min = poly.valueOnAxis(axes[0].n, axes[0].d);
    if( min > 0 ){
      return -1;
    }
    for( var i = 1; i < num; ++i ){
      var dist = poly.valueOnAxis(axes[i].n, axes[i].d);
      if( dist > 0 ){
        return -1;
      }else if( dist > min ){
        min = dist;
        min_index = i;
      }
    }
    
    out.min = min;
    return min_index;
  };
  
  // Add contacts for probably penetrating vertexes.
  // This handles the degenerate case where an overlap was detected, but no vertexes fall inside
  // the opposing polygon. (like a star of david)
  var findVertsFallback = function(arr, poly1, poly2, n, dist){
    var num = 0;
    
    for( var i = 0; i < poly1.numVerts; ++i ){
      var v = poly1.tVerts[i];
      if( poly2.containsVertPartial( v, n.neg() ) ){
        num++;
        arr.push( new Contact( v, n, dist, hash_pair(poly1.hashid, i)  ) );
      }
    }
    for( var i = 0; i < poly2.numVerts; ++i ){
      var v = poly2.tVerts[i];
      if( poly1.containsVertPartial(v, n) ) {
        num++;
        arr.push( new Contact( v, n, dist, hash_pair(poly2.hashid, i)  ) );
      }
    }
    
    return num;
  };

  
  // Add contacts for penetrating vertexes.
  var findVerts = function(arr, poly1, poly2, n, dist){
    var num = 0;
    
    for( var i = 0; i < poly1.numVerts; ++i ){
      var v = poly1.tVerts[i];
      if( poly2.containsVert(v) ){
        num++;
        arr.push( new Contact( v, n, dist, hash_pair(poly1.hashid, i) ) );
      }
    }
    for( var i = 0; i < poly2.numVerts; ++i ){
      var v = poly2.tVerts[i];
      if( poly1.containsVert(v) ){
        num++;
        arr.push( new Contact( v, n, dist, hash_pair(poly2.hashid, i) ) );
      }
    }
    
    return (num ? num : findVertsFallback(arr, poly1, poly2, n, dist));
  };
  
  var poly2poly = function( poly1, poly2, arr ){
    var min1 = {};
    var mini1 = findMSA(poly2, poly1.tAxes, poly2.numVerts, min1);
    if( mini1 === -1 ){
      return 0;
    }
    var min2 = {};
    var mini2 = findMSA( poly1, poly2.tAxes, poly2.numVerts, min2);
    if( mini2 === -1 ){
      return 0;
    }
    // There is overlap, find the penetrating verts
    if( min1.min > min2.min ){
      return findVerts( arr, poly1, poly2, poly1.tAxes[mini1].n, min1.min );
    }else{
      return findVerts( arr, poly1, poly2, poly2.tAxes[mini2].n.neg(), min2.min );
    }
  };



 return [
    circle2circle,
    undefined,
    undefined,
    circle2segment,
    undefined,
    undefined,
    circle2poly,
    seg2poly, 
    poly2poly 
  ];
});
  
