# M2 Marking Instructions 
You will need to demonstrate your SLAM on the physical robot during the Week 5 lab session.

**Please familiarise yourselves with these steps to ensure the demonstrators can finish marking your team in the allocated time**
- [Marking steps](#marking-steps)
- [Marking checklist](#marking-checklist)


Each team will have a **STRICT** 15 minutes total time limit to perform the live robot demo for marking according to a random schedule. The 15min marking slot starts when you are called to the marking arena and are asked to start downloading your code submission from Moodle. You will need to submit the map(s) generated during the live demo marking to Moodle **before** the 15min runs out. You may open up the marking checklist, which is a simplified version of the following steps to remind yourself of the marking procedures. Each team will also be doing their vivas right after their demonstration, with 10 minutes of viva split across the team.

**Further clarifications:**
- In addition to collision penalty, an out-of-arena penalty is added. The robot exceeding the arena borders will be considered the same as colliding with a marker, i.e., each time it exceeds the border there is a 5pt penalty, and the maximum number of allowed collision/out-of-bound is three times in a run. The fourth time your robot collides with a marker OR exceeds the arena boundry that run will be terminated.
- You are allowed to perform wheel and/or camera calibration again during the live demo marking, and replace the calibration parameter files under the "param" folder in your downloaded Moodle submission with the newly calibrated parameters or take in these new parameters as command line arguments. However, these are the only parts of the downloaded submission that you are allowed to change. Also, if you decide to perform calibration during your 15min marking time slot, the marking timer will continue while you perform the calibration.
- The 5min countdown clock in the operate.py GUI is only for reference. Your individual SLAM runs can be shorter or longer than 5min. The only time limit enforced is the 15min total time limit.
- [EDIT] Please check out the [marking order Google sheet (under the M2 tab)](TODO) for when your group's turn is and which marking arena you will be performing the live demo in. The first groups of each lab session will start their live demo marking 30min after that lab session starts.

### Evaluation scheme
To allow for the best performance of your SLAM module, you will be marked based on finding the 10 ARUCO markers, *the RMSE after alignment* between your estimations and the true locations of these markers during a live demonstration conducted in a **NEW MAP** in week 5. After the live demo, the map generated will be marked against the ground-truth map using [SLAM_eval.py](SLAM_eval.py). 5pt will be deducted for each marker the robot has collided into during the live demo, with a max of 3 collisions allowed per run. Your M2 mark is computed as:

slam_score = ((0.12 - Aligned_RMSE)/(0.12 - 0.02)) x 80

**Total M2 mark = slam_score + (NumberOfFoundMarkers x 2) - (NumberOfCollidedMarkers x 5)**

**Note:** If your Aligned_RMSE value goes above the upper bound 0.12, your slam_score will be 0. If the Aligned_RMSE value goes below the lower bound 0.02, your slam_score will be 80, i.e., 0 ≤ slam_score ≤ 80

There will also be an ITP metrics survey that will be sent out to address M2 and M3 together. A viva will be conducted during the marking process for each individual within the group, which will be a short oral assessment to gauge your understanding of the code and how you troubleshooted the issues for each milestone. The ITP value and viva will be applied to your final M2 and M3 scores by a multiplicative formula (ie. M2 score * ITP * viva).


### Marking steps
#### Step 1:
**Do this BEFORE your lab session**
Zip your **whole Week02-04 folder** (including the subfolders, such as util, pics, slam, and calibration, everything that your demo requires) and submit to the Moodle submission box according to your lab session. Each group only needs one submission. This submission is due by the starting time of the lab session, which means you should **submit your script BEFORE you come to the lab**. 

**Tips:** 
- You may also include a text file in the zip file with a list of commands to use, if you don't know all the commands by heart.
- **Please practise** the marking steps (eg. unzipping your code and running it) to ensure there are no issues during marking.
- Make sure your robot is fully charged before coming to the lab session.


#### Step 2: 
**Do this BEFORE the demonstrator come to mark your team**

1. Close all the windows/applications in your development environment (VM/native/WSL2, whichever one you use)

2. Use any team member's account to log in Moodle and navigate to the M2 submission box, so that you are ready to download your submitted code when the demonstrator arrives

3. Have an **empty** folder named "LiveDemo" ready in your home directory, ie. it is at ```~/LiveDemo/```. This folder should remain open at all time during marking (Does not apply to WSL2 users)

4. Please turn on your robot before the demonstrator comes to you. DO NOT connect to its hotspot yet. 

#### Step 3:
**During marking**
Note: within the 15min marking time limit, you may rerun SLAM as many times as you want. The attempt with the highest score will be your final score. 

1. The demonstrator will set up the marking arena containing the 10 ARUCO markers at the beginning of the session. Note that each lab session will get a slightly different map layout. **You are not allowed near the marking arena unless it is your team's turn for marking**

2. When the demonstrator ask you to come to the marking arena, download your submitted zip file from Moodle and unzip its content to the "LiveDemo" folder.

3. Activate your Python venv, then navigate to the "Week02-04" folder in your unzipped submission which contains the operate.py script

4. Connect to the robot and run the script with ```python3 operate.py --ip 192.168.50.1 --port 8080```

5. Demonstrate SLAM by **strategically** driving the robot around arena and search for all 10 markers
    - You may stop a run at anytime by pressing ```s``` to save the map generated by the current run. This implies that you can only generate one SLAM map per run.
    - You may rerun this step as many times as you want within your marking time limit. **Remember that the map submission has to be done within the time limit**, so make sure you leave enough time for the submission. The 15min marking timer also continues when you are getting ready in between runs (resetting the robot to the start location and resetting any markers that may have moved due to collision).
    - **A visible effort has to be made to try and locate all 10 markers.**
    - **You must immediately start your SLAM with the robot at the centre of the arena, facing the direction indicated by the arrow on the ground, before teleoperating the robot**
    - **The maximum number of collision allowed in a run is 3 times**. The fourth time your robot collides with a marker, you will be asked to terminate that run, save the map at that point, and restart a new run if time permits and if you want to
    - Note that the provided skeleton code saves the map under the same name (i.e., the later run would replace map generated by the earlier run), so you'll need to manually rename the maps in between runs, or modify the map-saving codes when developing your solution.
    - While you are not allowed to change anything in the downloaded submission of your implementation, your code can take in arguments at the time of execution. For example, if you plan to do some last minute fine-tuning of the wheel calibration parameters, you may write your code in a way that it takes in ```baseline``` and ```scale``` as command line arguments, and give these values at the time of running the demo. You are NOT allowed to give inputs related to the arena setup or object locations as command line arguments, an extreme example would be typing in the whole map manually as command line arguments (please don't). 

6. Submit all the generated maps as a zip folder to the Moodle map submission box

---

### Marking checklist
**BEFORE the lab session**
- [ ] Submit your code to Moodle
- [ ] Charge the robot

**BEFORE the marking**
- [ ] Close all programs and folders
- [ ] Log in Moodle and navigate to the submission box
- [ ] Open an empty folder named "LiveDemo" (located at ```~/LiveDemo/```)
- [ ] Turn on the robot (DO NOT connect to its hotspot yet)

**During the marking**
- [ ] Demonstrator will ask you to download your submission and unzip it to "LiveDemo"
- [ ] Connect to the robot
- [ ] Demonstrate SLAM (rename the ```lab_output/slam.txt``` file(s)) and good luck!
- [ ] zip and submit the map(s) to Moodle
