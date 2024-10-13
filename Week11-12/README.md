# Final Demo

## Introduction
The task for the final demo is the same as your trial run in [M5](../Week09-10#introduction).

## Marking
The final demo evaluation schemes are the same as your trial run in [M5](../Week09-10/M5_marking.md#marking-schemes). Please note that the final demo accounts for 60% of your unit mark.

## Marking procedure
The final demo marking procedure is the same as your trial run in [M5](../Week09-10/M5_marking.md#marking-steps). However, we have highlighted key differences between M5 and the final demo. Make sure to read through the original marking instructions and the changes here carefully. 

---

## Revised Final Demo marking scheme

**1. Demo duration increase**
  - The total duration of your Final Demo is increased to **30min**.
  - Within this 30min, you need to download and unzip your submitted codes, perform the demo, reset arena in between runs if needed, and submit your generated SLAM and targets maps with the required format and file names.

**2. Changes to marking calculations and clarifying map submission details [NEW]**
  - Target pose estimation will now be evaluated using an updated formula (Changing upper bound to 30cm instead of 1m), mapping_eval.py has been changed to account for this (lines 274 and 279):
  ~~~
  target_score[object] = (0.3 - estimation_error[object])/(0.3-0.025) x 3
  target_est_score = sum(target_score)
  0 ≤ target_est_score ≤ 30
  
  Line 274 changed to: if est_err > 0.3:
  Line 279 changed to: target_score = (0.3-est_err)/(0.3-0.025) * 8.0
  ~~~
  - All other marking calculations will remain the same.
  - Additionally, you must submit your files in following file structure:
  ~~~
  M5_maps
  ├── slam_0.txt
  ├── slam_1.txt
  ├── targets_0.txt
  ├── targets_1.txt
  ⋮
  ~~~
  - You may receive 0 marks for mapping if you fail to follow this file structure. We will also not be accepting maps in different formats (such as combined formats), these maps will automatically receive a 0.
  - You must also attend your Viva in week 12 unless you have special consideration, failure to do so will result in you receiving 0 marks for the final demonstration.

**3. Individual contributions to team and mark scaling**
  - The 3rd and last ITP survey will be open from 6pm 18th Oct to 6pm 23rd Oct. The results will be used to inform the individual scaling factors applied to the Final Demo's team scores (Edit: This previously mentioned that it would apply to both M5 and Final Demo, this will only apply to the Final Demo).
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
