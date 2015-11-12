#!/usr/bin/env python
import rosbag

max_msgs = 10
num_msgs = max_msgs
nanosecs = 10.0**9
time_index = 98.62
delta_time = 0.1
prev_time = time_index - delta_time

i_start = 17 # Right arm Joint # 1 Index
i_end = 23 # Right arm Joint # 7 Index


#for topic, msg, t in rosbag.Bag('circular_forward_trajectory.bag').read_messages():
for topic, msg, t in rosbag.Bag('bag_files/LR_upperWave.bag').read_messages():
    start_time = (float(str(t)) / (nanosecs)) 
    print msg
    break

# print "id time x -x y z"
# with rosbag.Bag('output.bag', 'w') as outbag:	
# #    for topic, msg, t in rosbag.Bag('circular_forward_trajectory.bag').read_messages():
#     for topic, msg, t in rosbag.Bag('demo_bags/lift_box_4.bag').read_messages():		
# 		# Convert current time in seconds
#         cur_time = (float(str(t)) / (nanosecs)) 

#         if (msg.id == 4):
#             print msg.id, "%.3f" % (cur_time-start_time), msg.pose.position.x, -msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
# #      print msg

#         # Ensure that we start at the desired time index
#         # and ensure that we space by 0.1 secs
#     	if (cur_time < time_index) or ((cur_time - prev_time) < delta_time) : 
#     		continue

#         # Print time and joint names once
#     	# if num_msgs == max_msgs: 
# 	    # 	print "Time", msg.name[i_start:i_end+1]

#      #    if num_msgs < 1:
#      #        break
#      #    num_msgs -= 1
        
#      #    # Obtain positions. Truncate to 5 decimals
#      #    positions = ["%.5f" % position for position in msg.position[i_start:i_end+1]]
#      #    print cur_time, positions

#      #    prev_time = cur_time # Update time constraint

#         #print "%.3f" % (float(str(t)) / (nanosecs)) , ["%.5f" % position for position in msg.position[i_start:i_end+1]]
        
#         #for i in range(len(msg.name)):
#         # 	print i, msg.name[i]
#         #17 r_upper_arm_roll_joint
# 		#18 r_shoulder_pan_joint
# 		#19 r_shoulder_lift_joint
# 		#20 r_forearm_roll_joint
# 		#21 r_elbow_flex_joint
# 		#22 r_wrist_flex_joint
# 		#23 r_wrist_roll_joint

# 		# print topic, msg, t
# #        print msg.name, msg.position

#        #outbag.write(topic, msg, t)