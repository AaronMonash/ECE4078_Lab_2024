# Final Demo

## Introduction
The task for the final demo is the same as your trial run in [M5](../Week09-10#introduction).

## Marking
The final demo evaluation schemes are the same as your trial run in [M5](../Week09-10/M5_marking.md#marking-schemes). Please note that the final demo accounts for 60% of your unit mark.

## Marking procedure
The final demo marking procedure is the same as your trial run in [M5](../Week09-10/M5_marking.md#marking-steps). However, we have highlighted key differences between M5 and final demo.

---

## Revised Final Demo marking scheme

**1. Demo duration increase**
  - The total duration of your Final Demo is increased to **30min**.
  - Within this 30min, you need to download and unzip your submitted codes, perform the demo, reset arena in between runs if needed, and submit your generated SLAM and targets maps with the required format and file names.

**2. Qualified navigation run and penalty scheme**
  - To achieve a qualified navigation run, you need to navigate to **3 out of the 5** targets in the order specified in the shopping list, and for each of them stop within 1m of the target (you will only get marks for successfully navigating to a target if the robot is within 0.5m of that target).
  - After the robot has navigated to 3 targets, you may stop the run at any time either manually or as your program stops by itself and keep the marks you've earned so far. For example, if your robot successfully navigated to the 1st and 2nd target without penalty, and you stopped it within 1m radius but outside of 0.5m radius of the 3rd target in a full auto run, you will receive 16pt for that run (a qualified run with marks containing two successful targets).
    - You are not allowed to manually reset the robot or the arena during a semi or full auto navigation run, such as moving aside a marker block that the robot gets stuck behind.
    - Manual interference to code execution during a full auto run after the run is launched with your one command will immediately terminate that run
    - The penalty will not exceed the total mark of the run: 0 ≤ (score of a semi auto run) ≤ 15pt, 0 ≤ (score of a full auto run) ≤ 40pt

**3. Individual contributions to team and mark scaling**
  - The 3rd and last ITP survey will be open from 6pm 18th Oct to 6pm 23rd Oct. The results will be used to inform the individual scaling factors applied to M5 and Final Demo's team scores.
  - We may conduct individual interview and code reviews as part of the final assessment to understand an individual's contribution to the team. The interview results may be used to adjust the individual scaling factor. The teaching team will email individuals during Swot Voc to arrange this interview if needed.

**4. Other**
  - The Final Demo will be video recorded. We may record the arena, the robot's behaviours inside the arena, your computer screens or keyboard actions. We will not record any people or faces. Any accidental recordings of personal information irrelevant to the Final Demo or the unit will be deleted.
  - We will prepare a small number of back-up robots with calibrated wheel and camera parameters ready to use with them. During the Final Demo, if a team has unexpected hardware issues they may switch to use these back-up robots. While switching and reconnecting to the back-up robot the demo timer will be paused.

---

## Further tips  and FAQs
Below are some tips that teams could use:
1. HAVE A BACK-UP PLAN!!! For example, implement a command line argument for switching between running semi or full auto navigation in case the full auto navigation crashes on the day. Also have a back-up driver and laptop.
2. Parameter tuning: check your wheel and camera parameters, YOLO confidence, SLAM covariance, radius around markers and objects for creating occupancy map, etc. We recommand recalibrating the wheel and camera during the Week 12 labs and check to make sure that your calibrated parameters are close to the [default parameters](../Week02-04/calibration/param/).
3. Try different map layouts and pay attention to when collisions might happen due to path finding not optimised or occupancy map radius not being able to handle inaccurate maps
4. Make sure to submit the right version of your codes containing all required components. Test to ensure that the codes work as expected before clicking "submit"
5. We are still seeing map syntax and naming errors in the submitted 'slam.txt' and 'targets.txt' which might result in 0pt mapping scores. Please make sure to check the maps generated and submit the generated maps that you want to be marked on.
6. Some groups had their generated SLAM map rotated or flipped on the x/y axis, please check this with practice arenas and sim maps. With the robot's camera facing left when positioned in the middle of an arena, the left half of the arena will have positive x coordinates, and the bottom half of the arena will have positive y coordinates
7. Practice your runs and discuss driving strategies with your teammates to reduce operator errors (e.g., pressing wrong buttons or giving wrong commands)
8. In a navigation run, distance of the 0.5m radius for successful navigation and 1m radius for qualified navigation is measured from the centre of the target, and the **entire** robot has to be within this radius
