Leo
- [x] Fix force feedback
  - Found that when the end effector is pushed, the base joint moves regardless if the trigger is held still, making the UR move as well.
    - Force feedback won't work untill we do a hardware re-design where ball bearings elminate jiggle
- [ ] Fix gripper urcap
- [ ] make safety plane ground more lenient 
- [ ] up the threshold for collision
- [ ] Feel which joints need to be strengthened for hardware redesign
- [ ] Gravity comp: motor's weak or faulty URDF? 
- [ ] Hardware redesign 

Sri
- [] Add joint normalization to leader arms when deploying script so that we don't have to rotate all joints to match the initial joint position
- [] Ensure smooth teleop of the right arm, right now there's some very weird jerky motion for some reason
- [] Revert the norm change of the joint position deltas before follower movement so that we can ensure that it's below some threshold
- [] Implement force feedback on the right arm
- [] Implement gravity compensation on the right arm