/** @namespace cp */
define(['cp/BB', 'cp/HashSet', 'cp/SpatialIndex', 'cp/constraints/util', 'cp/Prime', 'cp/assert'], function(BB, HashSet, SpatialIndex, util, prime, assert){
  "use strict";
 
  var leafSetEql = function(node){
    return (this === node.obj);
  }; 
  
  var leafSetTrans = function(obj, tree){
    return new Leaf( tree, obj, tree.bbfunc.call(obj) );
  };
  
  var voidQueryFunc = function(){};
  
  var each_helper = function(node, context){
    context.func.call(node.obj, context.data);
  };
 
  var cpfcompare = function(a, b){
    return (a < b ? -1 : (b < a ? 1 : 0 ) );
  }; 
  
  var proximity = function(a, b){
    return Math.abs(a.l + a.r - b.l - b.r) + Math.abs(a.b + b.t - b.b - b.t);
  };
 
  var MarkContext = function(tree, staticRoot, func, data){
    this.tree = tree;
    this.staticRoot = staticRoot;
    this.func = func;
    this.data = data;
  };


  var Node = function(tree, a, b){
    this.obj = undefined;
    this.bb = a.bb.merge(b.bb);
    this.parent = undefined;
    
    this.setA(a);
    this.setB(b);
  };
  
  var Leaf = function(tree, obj, bb){
    this.obj = obj;
    this.bb = tree.getBB(obj);
    
    this.parent = undefined;
    this.stamp = 0;
    this.pairs = undefined;
  };

  Node.prototype = Leaf.prototype = {
    setA: function(value){
      this.a = value;
      value.parent = this;
    },
    
    setB : function(value){
      this.b = value;
      value.parent = this;
    },
    
    isLeaf: function(){
      return !!this.obj;
    },
    
    other: function(child){
      return (this.a === child? this.b : this.a);
    },
    
    replaceChild: function(child, value, tree){
      assert.soft( !this.isLeaf(), "Cannot replace child of a leaf" );
      assert.soft( child === this.a || child === this.b, "Node is not a child of parent.");
      
      if( this.a === child ){
        //tree.nodeRecycle(this.a);
        this.setA(value);
      }else{
        //tree.nodeRecycle(this.b);
        this.setB(value);
      }
      for( var node = this; node && node.a && node.b; node = node.parent ){
        node.bb = node.a.bb.merge(node.b.bb);
      }
    },
    
    subtreeInsert: function(leaf, tree){
      if( this.isLeaf() ){
        return new Node(tree, leaf, this);
      }else{
        var cost_a = this.b.bb.area() + this.a.bb.mergedArea(leaf.bb);
        var cost_b = this.a.bb.area() + this.b.bb.mergedArea(leaf.bb);
        
        if( cost_a === cost_b ){
          cost_a = proximity( this.a.bb, leaf.bb );
          cost_b = proximity( this.b.bb, leaf.bb );
        }
        
        if( cost_b < cost_a ){
          var ins = leaf;
          if( this.b ){
            ins =this.b.subtreeInsert(leaf, tree);
          }
          this.setB( ins );
        }else{
          var ins = leaf;
          if( this.a ){
            ins = this.a.subtreeInsert(leaf, tree);
          }
          this.setA( ins );
        }
        
        this.bb = this.bb.merge(leaf.bb);
        return this;
      }
    },
    
    subtreeQuery: function(obj, bb, func, data){
      if( this.bb.intersects(bb) ){
        if( this.isLeaf() ){
          func.call(obj, this.obj, data);
        }else{
          this.a.subtreeQuery(obj, bb, func, data);
          this.b.subtreeQuery(obj, bb, func, data);
        }
      }
    },

    subtreeSegmentQuery: function(obj, a, b, t_exit, func, data){
      if( this.isLeaf() ){
        return func(obj, this.obj, data);
      }else{
        var t_a = this.a.bb.segmentQuery(a, b);
        var t_b = this.b.bb.segmentQuery(a, b);
        
        if( t_a < t_b ){
          if( t_a < t_exit ){
            t_exit = Math.min( t_exit, this.a.subtreeSegmentQuery(obj, a, b, t_exit, func, data) );
          }
          if( t_b < t_exit ){
            t_exit = Math.min( t_exit, this.b.subtreeSegmentQuery(obj, a, b, t_exit, func, data) );
          }
        }else{
          if( t_b < t_exit ){
            t_exit = Math.min( t_exit, this.b.subtreeSegmentQuery(obj, a, b, t_exit, func, data) );
          }
          if( t_a < t_exit ){
            t_exit = Math.min( t_exit, this.a.subtreeSegmentQuery(obj, a, b, t_exit, func, data) );
          }
        }
        return t_exit;
      }
    }, 
    
    subtreeRemove: function(leaf, tree){
      if( leaf === this ){
        return undefined;
      }else{
        var parent = leaf.parent;
        if( parent === this ){
          var other = this.other(leaf);
          other.parent = this.parent;
          //tree.nodeRecycle(this);
          return other;
        }else{
          parent.parent.replaceChild(parent, parent.other(leaf), tree);
          return this;
        }
      }
    },
    
    
    leafUpdate: function(tree){
      var root = tree.root;
      var bb = tree.bbfunc.call(this.obj);
      
      if( !this.bb.containsBB(bb) ){
        this.bb = tree.getBB(this.obj);
        
        root = root.subtreeRemove(this, tree);
        
        var ins = this;
        if( root ){
          ins = root.subtreeInsert(this, tree);
        }
        tree.root = ins;
        
        tree.pairsClear(this);
        this.stamp = tree.getStamp();
        
        return true;
      }
      return false;
    },
    
    addPairs: function(tree){
      var dynamicIndex = tree.dynamicIndex;
      if( dynamicIndex ){
        var dynamicRoot = dynamicIndex.root;
        if( dynamicRoot ){
          var dynamicTree = dynamicIndex.getTree();
          var context = new MarkContext(dynamicTree, undefined, undefined, undefined);
          
          dynamicRoot.markLeafQuery(this, true, context);
        }
      }else{
        var staticIndex = tree.staticIndex;
        if( staticIndex ){
          var staticRoot = staticIndex.root;
          var context = new MarkContext(tree,  staticRoot, voidQueryFunc,  undefined );
          this.markLeaf(context);
        }
      }
    },
    
    markLeafQuery: function(leaf, left, context){
      if( leaf.bb.intersects(this.bb) ){
        if( this.isLeaf() ){
          if( left ){
            leaf.pairInsert(this, context.tree);
          }else{
            if( this.stamp < leaf.stamp ){
              this.pairInsert(leaf, context.tree);
            }
            context.func.call(leaf.obj, this.obj, context.data);
          }
        }else{
          this.a.markLeafQuery(leaf, left, context);
          this.b.markLeafQuery(leaf, left, context);
        }
      }
    },
    
    markLeaf: function(context){
      var tree = context.tree;
      
      if( this.stamp === tree.getStamp() ){
        var staticRoot = context.staticRoot;
        if( staticRoot ){
          staticRoot.markLeafQuery(this, false, context);
        }
        
        for( var node = this; node.parent; node = node.parent ){
          if( node === node.parent.a ){
            node.parent.b.markLeafQuery(this, true, context);
          }else{
            node.parent.a.markLeafQuery(this, false, context);
          }
        }
      }else{
        var pair = this.pairs;
        
        while( pair ){
          if( this === pair.b.leaf ){
            context.func.call(pair.a.leaf.obj, this.obj, context.data);
            pair = pair.b.next;
          }else{
            pair = pair.a.next;
          }
        }
      }
    },

    markSubtree: function(context){
      if( this.isLeaf() ){
        this.markLeaf(context);
      }else{
        this.a.markSubtree(context);
        this.b.markSubtree(context);
      }
    },

    pairInsert: function(b, tree){
      var a = this;
      
      var nextA = a.pairs;
      var nextB = b.pairs;

      var pair = new Pair( new Thread(undefined, a, nextA), new Thread(undefined, b, nextB) );
      a.pairs = b.pairs = pair;
      
      if( nextA ){
        if( nextA.a.leaf === a ){
          nextA.a.prev = pair;
        }else{
          nextA.b.prev = pair;
        }
      }
      
      if( nextB ){
        if( nextB.a.leaf === b ){
          nextB.a.prev = pair;
        }else{
          nextB.b.prev = pair;
        }
      }
    }
  };

  var Thread = function(prev, leaf, next){
    this.prev = prev
    this.leaf = leaf;
    this.next = next;
  };
  
  Thread.prototype = {
    unlink: function(){
      var next = this.next;
      var prev = this.prev;

      if(next){
        if(next.a.leaf === this.leaf){
          next.a.prev = prev;
        }else{
          next.b.prev = prev;
        }
      }

      if(prev){
        if(prev.a.leaf === this.leaf){
          prev.a.next = next;
        }else{
          prev.b.next = next;
        }
      } else {
        this.leaf.pairs = next;
      }
    }
  };

  var Pair = function( a, b ){
    this.a = a;
    this.b = b;
  }; 
 
  /**
   * @namespace cp
   * @class BBTree
   * @private
   */
  var BBTree = function(bbfunc, staticIndex){
    SpatialIndex.call(this, bbfunc, staticIndex);
    
    this.velocityFunc = undefined;
    this.leaves = new HashSet( 0, leafSetEql );
    this.root = undefined;
    
    this.pooledNodes = undefined;
    
    this.stamp = 0;
  };
  
  /** @namespace cp.BBTree.prototype */
  BBTree.prototype = util.extend( new SpatialIndex(), {
    
    /**
     * @function setVelocityFunc
     * @param {function} func
     */
    setVelocityFunc: function(func){
      this.velocityFunc = func;
    },
    
    /**
     * @function getBB
     * @param {??} obj
     */
    getBB: function(obj){
      var bb = this.bbfunc.call(obj);
      
      var velocityFunc = this.velocityFunc;
      if( velocityFunc ){
        var coef = 0.1;
        var x = (bb.r - bb.l) * coef;
        var y = (bb.t - bb.b) * coef;
        
        var v = velocityFunc.call(obj).mult(0.1);
        return new BB( bb.l + Math.min(-x, v.x), bb.b + Math.min(-y, v.y), bb.r + Math.max(x, v.x), bb.t + Math.max(y, v.y) );
      }else{
        return bb;
      }
    },
    
    /**
     * @function getTree
     * @return {BBTree} this
     */
    getTree: function(){
      return this;
    },
    
    getRootIfTree: function(){
      return this.root;
    },
    
    /**
     * @function getStamp
     * @return {number} stamp of this tree
     */
    getStamp: function(){
      var dynamicIndex = this.dynamicIndex;
      if( dynamicIndex ){
        return dynamicIndex.stamp;
      }
      return this.stamp;
    },
    
    incrementStamp: function(){
      var dynamicIndex = this.dynamicIndex;
      var dynamicTree;
      if( dynamicIndex && (dynamicTree = dynamicIndex.getTree()) ){
        dynamicTree.stamp++;
      }else{
        this.stamp++;
      }
    },
    
    pairsClear: function(leaf){
      var pair = leaf.pairs;
      leaf.pairs = undefined;
      
      while( pair ){
        if( pair.a.leaf === leaf ){
          var next = pair.a.next;
          pair.b && pair.b.unlink && pair.b.unlink();
          //this.pairRecycle(pair);
          pair = next;
        }else{
          var next = pair.b.next;
          pair.a && pair.a.unlink && pair.a.unlink();
          //this.pairRecycle(pair);
          pair = next;
        }
      }
    },
  
    
    insert: function(obj, hashid){
      var leaf = this.leaves.insert(hashid, obj, this, leafSetTrans);
      
      var root = this.root;
      
      if( root ){
        this.root = root.subtreeInsert(leaf, this);
      }else{
        this.root = leaf;
      }

      leaf.stamp = this.getStamp();
      leaf.addPairs(this);
      this.incrementStamp();
    },
    
    remove: function(obj, hashid){
      var leaf = this.leaves.remove(hashid, obj);
      if( leaf ){
        this.root = this.root.subtreeRemove(leaf, this);
        leaf.pairsClear(this);
        //this.nodeRecycle(leaf);
      }
    },
    
    contains: function(obj, hashid){
      return !!this.leaves.find(hashid, obj);
    },
    
    reindexQuery: function(func, data){
      if( !this.root ){
        return;
      }
      // LeafUpdate() may modify tree->root. Don't cache it.
      this.leaves.each( function( leave, tree ){ 
        leave.leafUpdate(tree); 
      }, this );
      
      var staticIndex = this.staticIndex;
      var staticRoot = staticIndex && staticIndex.root;
      
      var context = new MarkContext(this, staticRoot, func, data);
      this.root.markSubtree(context);
      
      if( staticIndex && !staticRoot ){
        this.collideStatic(func, data);
      }
      this.incrementStamp();
    },
    
    reindex: function(){
      this.reindexQuery(voidQueryFunc, undefined);
    },
    
    reindexObject: function(obj, hashid){
      var leaf = this.leaves.find(hashid, obj);
      if( leaf ){
        if( leaf.leafUpdate(this) ){
          leaf.addPairs(this);
        }
        this.incrementStamp();
      }
    },
    
    pointQuery: function(point, func, data){
      var root = this.root;
      if( root ){
        root.subtreeQuery(point, new BB(point.x, point.y, point.x, point.y), func, data);
      }
    },
    
    segmentQuery: function(obj, a, b, t_exit, func, data){
      var root = this.root;
      if( root ){
        root.subtreeSegmentQuery(obj, a, b, func, data);
      }
    },
    
    query: function(obj, bb, func, data){
      if( this.root ){
        this.root.subtreeQuery(obj, bb, func, data);
      }
    },
    
    count: function(){
      return this.leaves.count();
    },
    
    each: function(func, data){
      var context = {
        func: func,
        data: data
      };
      this.leaves.each(each_helper, context);
    }

  });

  return BBTree;
});





/*

static void
fillNodeArray(Node *node, Node ***cursor){
	(**cursor) = node;
	(*cursor)++;
}

static Node *
partitionNodes(cpBBTree *tree, Node **nodes, int count)
{
	if(count == 1){
		return nodes[0];
	} else if(count == 2) {
		return NodeNew(tree, nodes[0], nodes[1]);
	}
	
	// Find the AABB for these nodes
	cpBB bb = nodes[0]->bb;
	for(int i=1; i<count; i++) bb = cpBBMerge(bb, nodes[i]->bb);
	
	// Split it on it's longest axis
	cpBool splitWidth = (bb.r - bb.l > bb.t - bb.b);
	
	// Sort the bounds and use the median as the splitting point
	cpFloat *bounds = (cpFloat *)cpcalloc(count*2, sizeof(cpFloat));
	if(splitWidth){
		for(int i=0; i<count; i++){
			bounds[2*i + 0] = nodes[i]->bb.l;
			bounds[2*i + 1] = nodes[i]->bb.r;
		}
	} else {
		for(int i=0; i<count; i++){
			bounds[2*i + 0] = nodes[i]->bb.b;
			bounds[2*i + 1] = nodes[i]->bb.t;
		}
	}
	
	qsort(bounds, count*2, sizeof(cpFloat), (int (*)(const void *, const void *))cpfcompare);
	cpFloat split = (bounds[count - 1] + bounds[count])*0.5f; // use the medain as the split
	cpfree(bounds);

	// Generate the child BBs
	cpBB a = bb, b = bb;
	if(splitWidth) a.r = b.l = split; else a.t = b.b = split;
	
	// Partition the nodes
	int right = count;
	for(int left=0; left < right;){
		Node *node = nodes[left];
		if(cpBBMergedArea(node->bb, b) < cpBBMergedArea(node->bb, a)){
//		if(cpBBProximity(node->bb, b) < cpBBProximity(node->bb, a)){
			right--;
			nodes[left] = nodes[right];
			nodes[right] = node;
		} else {
			left++;
		}
	}
	
	if(right == count){
		Node *node = NULL;
		for(int i=0; i<count; i++) node = SubtreeInsert(node, nodes[i], tree);
		return node;
	}
	
	// Recurse and build the node!
	return NodeNew(tree,
		partitionNodes(tree, nodes, right),
		partitionNodes(tree, nodes + right, count - right)
	);
}


void
cpBBTreeOptimize(cpSpatialIndex *index)
{
	if(index->klass != &klass){
		cpAssertWarn(cpFalse, "Ignoring cpBBTreeOptimize() call to non-tree spatial index.");
		return;
	}
	
	cpBBTree *tree = (cpBBTree *)index;
	Node *root = tree->root;
	if(!root) return;
	
	int count = cpBBTreeCount(tree);
	Node **nodes = (Node **)cpcalloc(count, sizeof(Node *));
	Node **cursor = nodes;
	
	cpHashSetEach(tree->leaves, (cpHashSetIteratorFunc)fillNodeArray, &cursor);
	
	SubtreeRecycle(tree, root);
	tree->root = partitionNodes(tree, nodes, count);
	cpfree(nodes);
}

*/
