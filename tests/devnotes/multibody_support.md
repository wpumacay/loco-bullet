
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

There is a trick I could also use. There seems to be issues when using the trick
from bullet (mjcf import example). What avoided explosions in the movement of the
bodies seems to be the limits and the motors used. If I set everything to non-limited,
then everything start to work not that well (it seems that inertialless bodies don't
work that well). The trick I could use it's just to add the required setupMultidof
method in the btMultibody class and see if everything works ok. That way I could
just set multi-dofs to the appropriate links, and no dummy links needed :D. We'll
see tomorrow :(.

---

Ok, so far the trick seems to work (see commit 9e1f851b0cc31451ddfd7dc2cbff19e56fab1661).
I'll try to implement the other approach that does not use dummy links by adding
a custom setup to the btMultiBody class. So far, these is what we've got:

- [x] Walker torso-like setup working with bullet's approach for mcjf (see [test_multidof_setup_application.cpp](https://github.com/wpumacay/tysocBullet/blob/9e1f851b0cc31451ddfd7dc2cbff19e56fab1661/tests/test_multidof_setup_application.cpp#L106)).
- [x] Humanoid shoulder-like setup working with bullet's approach for mjcf (see [test_multidof_setup_application.cpp](https://github.com/wpumacay/tysocBullet/blob/9e1f851b0cc31451ddfd7dc2cbff19e56fab1661/tests/test_multidof_setup_application.cpp#L57)).
- [x] Simple UI integration for some tests (see [test_interface.cpp](https://github.com/wpumacay/tysocBullet/blob/9e1f851b0cc31451ddfd7dc2cbff19e56fab1661/tests/base/test_interface.cpp#L324)).
- [x] Simple tests for the motor constraints (see [test_interface.cpp](https://github.com/wpumacay/tysocBullet/blob/9e1f851b0cc31451ddfd7dc2cbff19e56fab1661/tests/base/test_interface.cpp#L625)).

### Testing the trick further, and implement a non-hacky option (5/23/19)

Ok, so I've already check the trick and it seems that this could actually work for
the bullet implementation of the wrappers (tysoc bullet-backend). The only issue
I see is the potential limitation of the actuations in the case that by using
motor constraints I can no longer access torques directly (via btMultiBody torques
setters or any other way). I'll try to test this and see how far can it get. Also,
the UI should change a bit to test all simmultibodies (just a few changes required,
so this should be quick).

Another approach I'll try to implement as well is to add a custom setup method for
multidof cases, such that I don't have to depend on dummy links for multidofs any
longer. I'll see how this goes, and if both work correctly for the requirements I
have (support for actuation models in the future) then I would consider allowing
the user to select from both.

Once I finish these key considerations I think I could finally use all the code
I made for tests for the actual implementation of the wrappers for tysocBullet
agents. So, these are the considerations I should check today:

* Check torques usage when using motor constraints for multibodies
* Improve the UI a bit to consider all multibodies and some other extra functionality.
* Test the approach of non-dummy objects and check with the shoulder and torso test cases.
* Wrap up tests with more tests on actuation limitations, and also add a simple
  model from mujoco (hard-coded) to see how everything goes.