# momentum-flight

Private repository for our Momentum Team for purely storage purposes to store copies of working versions of our code when we reach our milestones

No need to learn git commands for this project; use copy/paste

### To Add File ###
Go to the correct Draft folder

Click 'Add file' > 'Create new file'

Title as name-date.py

Copy paste file

Click 'Commit new file'

### Tasks ###
Goal for Progress Report: Super basic reactionary alg: if middle sensors are close to obstacle, go up. else, go straight

1. dis_pointing_down(): given lidar reading data, use "orientation" parameter and rotation matrices to figure out which index is pointing down and this distance. Remember we only need to look along the middle sensors. If this turns out to be too difficult, we can just use the maximum distance given

2a. Code: Update while loop in reactionary code, so that it has an if else statement (if middle sensors are close to obstacle, go up. else, go straight. also might need to go down if all the obstacles are far)

2b. Numbers: We currently have random numbers for how much forward or up drone should go. Need to look into the units and put in more reasonable numbers

3. Test: Run our programs on remote terminal and take a video for progress report. Do this by opening vscode and saving our files; if you save in the same location as demo_mission.py (home/Momentum etc) you can run it the same way as the Google Docs

### To Do ###
1. Read the [Momentum Code Notes](https://docs.google.com/document/d/190yfrauW1Njj7F8keZMDoK98A8mNPiVudUl6i1_pmwU/edit?usp=sharing) to understand how to interact with the software in Python. You can skip page 1 

If you have questions compile them at the top of the Google Docs

2. (If time) Look at the two python files located in the folders above. I write at the top which lines are important 

### Resources ###
[Momentum Code Notes Google Docs](https://docs.google.com/document/d/190yfrauW1Njj7F8keZMDoK98A8mNPiVudUl6i1_pmwU/edit?usp=sharing)

[LM Github](https://github.com/katabeta/lm-mit-momentum)
