define([
  './cp/Arbiter', './cp/BBTree', './cp/Contact', './cp/Shape', './cp/Array', './cp/Body', './cp/cpf', './cp/assert', 
  './cp/CollisionHandler',  './cp/Hash', './cp/SpaceHash', './cp/BB', './cp/Collision', './cp/HashSet', './cp/Space', 
  './cp/Constraints', './cp/SpatialIndex', './cp/ContactBuffer', './cp/Prime', './cp/Vect'
  ],function(Arbiter, BBTree, Contact, Shape, Array, Body, cpf, assert, CollisionHandler, Hash, SpaceHash, BB, Collision, HashSet, Space, Constraints, SpatialIndex, ContactBuffer, Prime, Vect ){
    return {
      'Arbiter' : Arbiter,
      'BBTree' : BBTree,
      'Contact' : Contact,
      'Shape' : Shape,
      'Array' : Array,
      'Body' : Body,
      'cpf' : cpf,
      'assert' : assert,
      'CollisionHandler' : CollisionHandler,
      'Hash' : Hash,
      'SpaceHash' : SpaceHash,
      'BB' : BB,
      'Collision' : Collision,
      'HashSet' : HashSet,
      'Space' : Space,
      'Constraints' : Constraints,
      'SpatialIndex' : SpatialIndex,
      'Prime' : Prime,
      'Vect': Vect
  };
});
