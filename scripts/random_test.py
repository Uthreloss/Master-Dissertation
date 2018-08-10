import os, sys

# Path to be created

PositionRange = 3
Sets = 4
ItRange = 6
Participants = 10
for k in range(Participants):
    for i in range(Sets):
        for j in range(PositionRange):
            path = "/home/pepper/catkin_ws/src/master_dissertation/experiment_bags/participant_" +str(k+1)+ "/set_" + str(i+1) +"/P_"+ str(j+1)
            os.makedirs(path)
