Leo
- [x] Fix force feedback
  - Found that when the end effector is pushed, the base joint moves regardless if the trigger is held still, making the UR move as well.
    - Force feedback won't work untill we do a hardware re-design where ball bearings elminate jiggle
- [X] Fix gripper urcap
- [X] make safety plane ground more lenient 
- [X] up the threshold for collision
- [X] Feel which joints need to be strengthened for hardware redesign
- [X] Gravity comp: motor's weak or faulty URDF? 
- [X] Hardware redesign 
- [ ] Collect and make sure we have all the parts
- [ ] Build the leader arms
- [ ] Make a new URDF of the new design
- [ ] Print out wrist cams

Sri

priority: figure out root cause of issue with motor 2, i think we need to definitely swap it out for a new xc330, not sure why it's so unstable. 

- [x] Add joint normalization to leader arms when deploying script so that we don't have to rotate all joints to match the initial joint position
- [x] Ensure smooth teleop of the right arm, right now there's some very weird jerky motion for some reason
- [x] Revert the norm change of the joint position deltas before follower movement so that we can ensure that it's below some threshold
- [x] Implement force feedback on the right arm
- [x] Implement gravity compensation on the right arm
- [ ] Collision avoidance
- [ ] Sync with Pranav on data collection pipeline
