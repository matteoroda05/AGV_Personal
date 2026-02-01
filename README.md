# AGV_Personal
Personal repository of trials and tests for the Devnut X PMDS AGV project 

## Summary of files added/edited recently
Reverse chronological order of last update
- Folder: my_first_simulation – 1 Feb 2025
    - Created a a 'first simulation' in webots following the tutorial on the documentation.
    - Added simple world following tutorial 1 (https://cyberbotics.com/doc/guide/tutorial-1-your-first-simulation-in-webots?tab-language=python) in python (since it's the language we will use for the project) and added/modified some bits from the base tutorial to better familiarize myself with webots. Specifically: 
        - changed the view from mountains to "twilight_cloud_empty" for personal liking;
        - added a wooden crate (object subject to no forces, by default sticks to ground); 
        - added three beers (objects subject to physics and gravity) each one with a different weight to see the different behaviours when falling/being pushed by the robot;
        - added a simple epuck robot and changed its controller to a custom one [File: epuck_go_forward.py] that makes the robot go forward for 5s and then starts turning with left wheel speed unchanged (50% of max speed) and right wheel speed at 1% of max speed, making the robot go in circles.
- File: client_trial1.py – 1 Feb 2025
    - Created a simple python client code to test socket communication with a C++ server.
    - Client code was supposed to be used to connect to a C++ server code that would then be the actual controller of the AGV in webots, however we found out we could directly use a python controller in webots therefore client_trial1.py is just a test file and will not be used and is useless by its own.