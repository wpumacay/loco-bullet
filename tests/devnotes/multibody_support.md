
# Some dev-notes regarding multibody-support

Finally I'm back at trying to make this work :D. I already tried both using single
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