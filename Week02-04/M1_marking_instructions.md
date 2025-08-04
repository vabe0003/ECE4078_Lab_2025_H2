# M1 Marking Instructions 

**Please familiarise yourselves with these steps to ensure the demonstrators can finish marking your team in the allocated time**
- [Marking steps](#marking-steps)
- [Marking checklist](#marking-checklist)


Each team will have a **STRICT** 15 minutes total time limit to perform the live robot demo for marking according to a random schedule. When it is your group's turn, you will be called to the marking arena and will download the Moodle submission. After setting up your LiveDemo folder, the 15min marking slot will start when your robot and code are ready (the 15 minutes include calibration time). You will need to submit the map(s) generated during the live demo marking to Moodle **before** the 15min runs out. You may open up the marking checklist, which is a simplified version of the following steps to remind yourself of the marking procedures. 

- You are allowed to perform wheel and/or camera calibration again during the live demo marking, and replace the calibration parameter files under the "param" folder in your downloaded Moodle submission with the newly calibrated parameters or take in these new parameters as command line arguments. However, these are the only parts of the downloaded submission that you are allowed to change. Also, if you decide to perform calibration during your 15min marking time slot, the marking timer will continue while you perform the calibration.
- The 5min countdown clock in the operate.py GUI is only for reference. Your individual SLAM runs can be shorter or longer than 5min. The only time limit enforced is the 15min total time limit.
- Please check out the [marking order Google sheet (under the M1 tab)](https://docs.google.com/spreadsheets/d/1X3cr0gBKZy2VaotczIgovOc5Q4cgQa3NROwMl_u-cKw/edit?usp=drive_link) for when your group's turn is and which marking arena you will be performing the live demo in.

### Evaluation scheme
To allow for the best performance of your SLAM module, you will be marked based on finding the 10 ARUCO markers, *the RMSE after alignment* between your estimations and the true locations of these markers during a live demonstration conducted in a **NEW MAP** in week 6. After the live demo, the map generated will be marked against the ground-truth map using [SLAM_eval.py](SLAM_eval.py). 5pt will be deducted for each marker the robot has collided into and/or each time the robot has collided with the cage walls during the live demo, with a max of 3 collisions allowed per run. Your M1 mark is computed as:

slam_rating = ((0.2 - Aligned_RMSE)/(0.2 - 0.02))

slam_score = (base^slam_rating - 1)/(base -1) * 80 where base = 16

**Total M1 mark = (slam_score + (NumberOfFoundMarkers x 2) - (NumberOfCollisions x 5)) * NumberOfFoundMarkers/NumberOfMarkers**

**Note:** If your Aligned_RMSE value goes above the upper bound 0.2, your slam_score will be 0. If the Aligned_RMSE value goes below the lower bound 0.02, your slam_score will be 80, i.e., 0 ≤ slam_score ≤ 80

If multiple slam maps were submitted, the best performing map was used as the team's M1 mark. 

We will also be conducting a viva (a short interview), to confirm each team member's understanding of the project milestones. Each member of the team will be individually interviewed about what has been developed. You will be required to answer questions based primarily on the software components you personally worked on. However, you are also expected to demonstrate some understanding of the other components developed by the group.

It is therefore important to be thoroughly prepared to explain your own contributions and have a basic understanding of how the other parts of the code fit into the overall project.

Total project marks will be scaled based on the level of understanding demonstrated in the Viva, with the full marks being no change and a reduction as the category code drops (see the table below for the marking criteria).

If you miss (do not attend) your project interview, you may receive a 0.

Viva Marking Criteria
---------------------

| Category Description     | Detailed Description                                                                                                                                                  |
|--------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| Complete Understanding   | The student is clearly prepared and can answer questions concisely and correctly with little to no prompting about the parts they personally worked on. They also show a reasonable understanding of the rest of the codebase and how it connects to their contribution. |
| Tolerable Understanding  | The student has prepared and can answer mostly correct responses about their own work, though some clarification or prompting may be needed. They demonstrate limited understanding of the other parts of the code. |
| Selective Understanding  | The student can answer questions about some parts of their own work, but is clearly unprepared for other areas, either within their contribution or the broader system. Their preparation is insufficient or incomplete. |
| Trivial Understanding    | The student demonstrates only vague or superficial knowledge of their contribution, and is unable to engage with questions meaningfully. There is little or no awareness of the rest of the team's code. |
| No Understanding         | The student appears completely unprepared, shows no knowledge of the code, and cannot answer basic questions, even with assistance. May indicate they have not reviewed the work at all. |
| Absent                   | The student did not attend the viva interview for the project.                                                                                                         |

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

1. Close all the windows/applications in your development environment.

2. Use any team member's account to log in Moodle and navigate to the M1 submission box, so that you are ready to download your submitted code when the demonstrator arrives

3. Please turn on your robot before the demonstrator comes to you. DO NOT connect to its hotspot yet. 

#### Step 3:
**During marking** you may rerun SLAM as many times as you want. The attempt with the highest score will be your final score. 

1. The demonstrator will set up the marking arena containing the 10 ArUco markers at the beginning of the session. Note that each lab session will get a slightly different map layout. **You are not allowed near the marking arena unless it is your team's turn for marking**

2. When the demonstrator ask you to come to the marking arena, download your submitted zip file from Moodle and unzip its content to the "`~/LiveDemo`" folder. If working with the dockerised container, you will be asked to copy the folder from your local device into the dockerised container. **You must not have a pre-existing LiveDemo folder in your container**.

3. Navigate to the "Week02-04" folder in your unzipped submission which contains the operate.py script

4. Connect to the robot and run the script with ```python3 operate.py```

5. Demonstrate SLAM by **strategically** driving the robot around arena and search for all 10 markers
    - You may stop a run at anytime by pressing ```s``` to save the map generated by the current run 
    - You may rerun this step as many times as you want within your marking time limit. **Remember that the map submission has to be done within the time limit**, so make sure you leave enough time for the submission. The 15min marking timer also continues when you are getting ready in between runs (resetting the robot to the start location and resetting any markers that may have moved due to collision).
    - **The maximum number of collision allowed in a run is 3 times**. The fourth time your robot collides with a marker or arena wall, you will be asked to terminate that run, save the map at that point, and restart a new run if you want to (time will continue to tick)
    - Note that the provided skeleton code saves the map under the same name (i.e., later runs will overwrite the map generated by the earlier run), so you must manually rename the maps in between runs, or modify the map-saving functionality in your code to mitigate this concern.
    - While you are not allowed to change anything in the downloaded submission of your implementation, your code can take in arguments at the time of execution. For example, if you plan to do some last minute fine-tuning of the wheel calibration parameters, you may write your code in a way that it takes in ```baseline``` and ```scale``` as command line arguments, and give these values at the time of running the demo. You are NOT allowed to give inputs related to the arena setup or object locations as command line arguments, an extreme example would be typing in the whole map manually as command line arguments. 

6. Submit all the generated maps as a zip folder to the Moodle map submission box

---

### Marking checklist
**BEFORE the lab session**
- [ ] Submit your code to Moodle
- [ ] Charge the robot

**BEFORE the marking**
- [ ] Close all programs and folders
- [ ] Log in Moodle and navigate to the submission box
- [ ] Turn on the robot (DO NOT connect to its hotspot yet)
- [ ] Demonstrator will ask you to download your submission and unzip it to "LiveDemo"
- [ ] Connect to the robot

**During the marking**
- [ ] Demonstrate SLAM (rename the ```lab_output/slam.txt``` file(s)) and good luck!
- [ ] zip and submit the map(s) to Moodle
