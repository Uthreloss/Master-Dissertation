#!/usr/bin/env python
import numpy as np
#This function return a TXT file including all poses 1, 2 or 3 gathered for each participant a new txt file representing all the participant

#Core variables
itNumber = 6 #Number of iterations
Positions = 3
Participants = 10
Master_Diss_Path = "/home/generic/ros_workspaces/coach_demo/src"
#For each participant (10 available per participant) copy and paste into a new txt file representing the population
for w in range(Positions):
    for p in range(Participants):
        ##Open the text to be copied and pasted
        text2copy = open(Master_Diss_Path + "/master_dissertation/experiment_bags/participant_" + str(p+1) + "/P_" + str(w+1) + "_Cluster.txt","r") #Create a text file too

        if p==0:
            text = open(Master_Diss_Path + "/master_dissertation/experiment_bags/participants_P_" + str(w+1) + "_Big_Cluster.txt","w") #Create a text file too
        #Paste the first whole participant data
            for idx, line in enumerate(text2copy):
                text.write(line)
            text.close()
        else:
            text = open(Master_Diss_Path + "/master_dissertation/experiment_bags/participants_P_" + str(w+1) + "_Big_Cluster.txt","a") #Append the text file
        #Paste everything but the first two lines for the rest
            for idx, line in enumerate(text2copy):
                if idx == 0 or idx == 1:
                    DoNotWrite=1
                else:

                    text.write(line)
            text.close()
        text2copy.close()
