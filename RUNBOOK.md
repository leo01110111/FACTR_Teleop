If this is your first time using the UR arms, please contact Leo (leokingw@andrew.cmu.edu) for an in-person tutoring session.
There are many details that would be best communicated in-person so this runbook doesn't serve as a replacement for the in-person sessions.

The summary of what to do:

- Get training for how the UR arms work with the pendant.
- Create a user account on the robot computer.
- In your user account, give yourself the right permissions to use the usb ports:
  - sudo usermod -aG dialout,input "$USER" 
    - log out and back in, then verify:
  - id -nG  
    - must list both dialout and input
- run ./run_factr_left.sh and/or ./run_factr_right.sh Turn on the teleop program on their respective pendants when asked in the terminal.
  - These scripts allow you to use the leader arms to teleoperate
- Create a dataset in raw_data/[dataset name] (create the raw_data dir if this is your first time)
- run ./record_data_left.sh or ./record_data_left.sh 
  - I haven't designed the system to record data for both arms at the same time. If you run both it probably will still work, but it's untested so check to see if logging works as intended.

