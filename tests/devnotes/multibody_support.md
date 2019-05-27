
[//]: # (References)

<!-- IMAGES-GIFS -->
[gif_planar_walker_wip_1]: ../../imgs/gif_multibody_planar_walker_wip.gif

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

---

Ok, using dummies kind of worked out for a simple planar walker (mujoco model). Some
extra info can be found in commit 57ad3c0e930672e5c24a9be0238c1cf91416b809, which
adds the walker demo. Regarding the non-dummies approach, I think I will need more 
info/knowledge to implement this appropriately. The current implementation uses
single vectors Rvector|Evector for their implementation, and it seems this wasn't
designed to accommodate multidofs for a single link in different positions.

So far, this is what we've got:

- [x] Updated tests that take into account two different types of actuation, namely
      torques and motors (seem like PD controllers).
- [x] Added a small hack for the weird behaviour of massless dummy bodies. By adding
      a bit of mass and inertia it seems to stabilize (I wonder if this is similar
      to the armature that each mjcf model has).
- [x] Updated UI to take into account these actuation options.
- [x] Added reference frames to the bodies to check the links positions/orientations.
- [x] Extracted the links world frames from the multibody, as we will need them for
      our wrapper implementation (the base is the body, and everything else is defined
      w.r.t. the body).
- [x] Added a working planar walker demo as a proof of concept of the buller backend.

![planar-walker-wip-1][gif_planar_walker_wip_1]

### Implementing the wrappers for bullet as backend (5/24/19)

Hopefully everything will work out. I've made the tests and they seem fine, so it
looks like the implementation of thw agent_wrappers for bullet should go smoothly
(with some issues here and there, but ultimately everything should work just fine).

The idea is to implement the bullet agent wrappers using the multibody implementation
and tests made so far. Previously we tried to implement it using non-multibody resources,
like rigid bodies and constraints, but this didn't work out (issues with planar constraints,
with multidofs, etc.). It could have worked out, but I think I was just forcing it too much
(the walker was right, it just wasn't constraint to the plane, which could have been fixed
forcing even more the implementation).

For today (and perhaps tomorrow as well) I will be implementing these agent wrappers.
The only issue that I anticipate is the fact that bodies have resources attached to them
in the kintree (collisions, visuals, dofs), which adds an extra reference frame to take into
account. The planar walker did not have any orientations mismatches between bodies, dofs and
geoms, so everything was just translations. Other models do have this mismatch, so I'd have
to take this into account. Hopefully, btMultiBodys allow you to grab the frame info in
worldspace, so we can just use this and call it a day (we'll see though). So, these
seem to be the key updates I'll have to make today+tomorrow:

* Implement the agent-wrappers for bullet, and test and compare the implementation
  to the mujoco implementation (they whould be very similar).
* Add support for torques as default actuation (from mjcf, motor-actuators are
  torques). Should map also gears, and link it to the UI for tests.

**Note**: current bullet version is fd161fa061539257036dd8f651bda3d3bf079556