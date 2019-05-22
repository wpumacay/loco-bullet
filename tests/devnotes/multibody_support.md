
# Some dev-notes regarding multibody-support

Finally, I'm back at trying to make this work :D. I already tried both using single
rigid bodies and appropriate constraints, and also tried making some tricks with
the constraints (duplicating geoms as bodies), but so far there are cases that
don't work properly :(.

My hope now is the featherstone implementation provided via btMultidbodyXYZ. That's
what roboschool is using, and what pybullet seems to be using as well (when creating
robots from sdf, urdf or mjcf). I've been looking at the code for the featherstone 
implementation, and so far it seems that in order to make both mujoco and bullet 
backends compatible I'll have to add an extra **setupCustom** method to support
multiple degrees of freedom, and add something similar to the spherical setup when
constructing the jacobian. I'll see how that goes (humanoid.xml and a few other 
models need multi-dof in some joints, so I think this feature is a must for full
compatibility).

### A trick for mjcf??? (5/22/19)

It seems that the way mjcf support is implemented in pybullet is via dummy links,
with some dummies per dof. For example, the hopper.xml model has only 4 bodies,
but it creates 10+1 links (1 extra for the base) when using the featherstone-based
implementation. It seems that the trick is to create a dummy link per mjcf "joint" 
(mujoco dof), which seems possible.

So far I've tested the case for a base with no mass (dummy base) and a link that
represents an actual body, and it seems to work ok. I'll try to implement the case
for a body with multi-dofs, specifically the following cases:

* The multi-dof base of the walker (make sure it keeps moving only in a plane)
* The upper shoulder from the humanoid, as it uses 2-dofs (2 revolutes intersecting 
  in a common axis).

I think I also have to implement some extra functionality regarding controling the 
torques or forces applied to the links of a multibody (in order to further test some 
stuffs). The approach should be as follows:

* Integrate a simple UI into the application tests (hard-coded design of the UI
  should be required, as each test might required different functionality).
* Check bullet's examples for references of the actuation implementation. Their code
  suggests that it's possible to directly actuate the multibody at each dof, I think
  I saw it in some place (they placed some code for enable motors or similar mechanisms
  for actuation).