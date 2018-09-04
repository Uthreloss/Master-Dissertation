import os, sys

# Variables for the tree creation
PositionRange = 3
Sets = 4
ItRange = 6
Participants = 20

#For each participant, set and pose create a directories tree accordingly
for k in range(Participants):
    for i in range(Sets):
        for j in range(PositionRange):
            #path = "/home/generic/ros_workspaces/coach_demo/src/master_dissertation/experiment_bags/participant_" +str(k+1)+ "/set_" + str(i+1) +"/P_"+ str(j+1)
            path = "/home/pepper/catkin_ws/src/master_dissertation/experiment_bags/participant_" +str(k+1)+ "/set_" + str(i+1) +"/P_"+ str(j+1)

            os.makedirs(path)
