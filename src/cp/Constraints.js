define([
  './constraints/Constraint', './constraints/GearJoint', './constraints/PivotJoint', './constraints/SimpleMotor', 
  './constraints/DampedRotarySpring', './constraints/GrooveJoint', './constraints/RatchetJoint', './constraints/SlideJoint', 
  './constraints/DampedSpring', './constraints/PinJoint', './constraints/RotaryLimitJoint',  './constraints/util'
  ], function(Constraint, GearJoint, PivotJoint, SimpleMotor, DampedRotarySpring, GrooveJoint, RatchetJoint, SlideJoint, DampedSpring, PinJoint, RotaryLimitJoint){
    return {
      'Constraint' : Constraint,
      'GearJoint' : GearJoint,
      'PivotJoint' : PivotJoint,
      'SimpleMotor' : SimpleMotor,
      'DampedRotarySpring' : DampedRotarySpring,
      'GrooveJoint' : GrooveJoint,
      'RatchetJoint' : RatchetJoint,
      'SlideJoint' : SlideJoint,
      'DampedSpring' : DampedSpring,
      'PinJoint' : PinJoint,
      'RotaryLimitJoint': RotaryLimitJoint
    };
});
