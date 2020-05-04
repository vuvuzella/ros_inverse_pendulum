## Notes in URDF modeling

### Aligning the origins

* Origins stack up based on its parent's origin. The value put in a joint's origin is relative to the connecting links' origin based.

* A link's origin is based on how it should rotate or trnaslate relative to the joint it is connected to, where the link is a child of that joint.

* The <visual> tag helps in visualizing how the objects stack up

* Use Rviz's TF to visualize how the links are interconnected, how they rotate in relation to each other

* A box's, sphere's and cylinder's origin starts from the center
