#!/usr/bin/env python
import numpy as np
#This function return the average of TXT files in a new txt file

#Core variables
itNumber = 6 #Number of iterations
Positions = 3
Participants = 2
Master_Diss_Path = "/home/generic/ros_workspaces/coach_demo/src"
for w in range(Positions):
    for p in range(Participants):
        text2copy = open(Master_Diss_Path + "/master_dissertation/experiment_bags/participant_" + str(p+1) + "/P_" + str(w+1) + "_Cluster.txt","r") #Create a text file too
        if p==0:
            text = open(Master_Diss_Path + "/master_dissertation/experiment_bags/participants_P_" + str(w+1) + "_Big_Cluster.txt","w") #Create a text file too
        #Paste the first two lines
            for idx, line in enumerate(text2copy):
                text.write(line)
            text.close()
        else:
            text = open(Master_Diss_Path + "/master_dissertation/experiment_bags/participants_P_" + str(w+1) + "_Big_Cluster.txt","a") #Append the text file
            #Paste everything but the first two lines
            for idx, line in enumerate(text2copy):
                if idx == 0 or idx == 1:
                    Yeah=1
                else:

                    text.write(line)
            text.close()
        text2copy.close()
