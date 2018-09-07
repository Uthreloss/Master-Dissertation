#!/usr/bin/env python
import numpy as np
#This function return the average of TXT files samples in a new txt file

#Core variables
Participants = 10 # Default = 10
Sets = 4 # Default = 4
Master_Diss_Path = "/home/generic/ros_workspaces/coach_demo/src"


#Read the first document to get number of columns
with open(Master_Diss_Path + "/master_dissertation/experiment_bags/participant_2/set_1_Rest.txt") as f:
    file = f.read()

fileS = file.split('\n') #Rows
data = fileS[2:-1] #Exclude description lines (Only numbers)
elements = int(data[0].split('\t')[-1]) #Last number indicates the number of columns per .txt
coordinates = 3 #XYZ
reference = 6 #Joint number to act as the origin before averaging

Attributes = [
    "joint_position_head",
    "joint_position_neck",
    "joint_position_shoulder", #Always 0 by default coincides with camera frame
    "joint_position_spine_top",
    "joint_position_spine_mid",
    "joint_position_spine_bottom", #Origin frame
    "joint_position_left_shoulder",
    "joint_position_left_elbow",
    "joint_position_left_hand",
    "joint_position_right_shoulder",
    "joint_position_right_elbow",
    "joint_position_right_hand"]

# Initialise a matrix to contains ALL XYZ coordinates to be extracted from the TXT file and later on averaged
file_text = np.zeros((Sets, elements*coordinates))

#Per participant, set, pose and iteration access a file and extract XYZ from stream recording to do an average
for p in range(Participants):
    for s in range(Sets):
        #Data accessing
        #                                                                                       Participant            Set
        with open(Master_Diss_Path + "/master_dissertation/experiment_bags/participant_" + str(p+1) + "/set_" + str(s+1) + "_Rest.txt") as f:
            fileX = f.read() #Read raw text
        fileS = fileX.split('\n') #Separate rows
        data = fileS[2:-1] #Eliminate description (Strings)
        rows=len(data) #Count rows
        # Extraction and averaging
        compressed = np.zeros(elements*coordinates) #Initialisation of the array containing the averaged values
        #For each group of coordinates and row
        for i in range(elements):
            for j in range(rows):
                #Save the origina frame in a variable
                ref = data[j].split('\t')[reference-1]
                xref = float(ref.split(',')[0])
                yref = float(ref.split(',')[1])
                zref = float(ref.split(',')[2])

                #Separate the group of XYZ coordinates
                tri = data[j].split('\t')[i] #"i" is after split is coincides with a pack of XYZ data

                #Add up correspondingly to X, Y and Z
                compressed[i*coordinates]= compressed[i*coordinates] + float(tri.split(',')[0]) - xref # X float addition
                compressed[(i*coordinates)+1]= compressed[(i*coordinates)+1] + float(tri.split(',')[1]) - yref # Y float addition
                compressed[(i*coordinates)+2]= compressed[(i*coordinates)+2] + float(tri.split(',')[2]) - zref # Z float addition
        #Average calculation
        if not rows==0:
            compressed = compressed/rows #Final step to calculate the average
        file_text[s,:]=compressed


#Copy AND Paste

    #Open any old document to copy the first two lines (Where the description is)
    old_text = open(Master_Diss_Path + "/master_dissertation/experiment_bags/participant_" + str(p+1) + "/set_1_Rest.txt","r") #Create a text file too
    text = open(Master_Diss_Path + "/master_dissertation/experiment_bags/participant_" + str(p+1) + "/set_AV_Rest.txt","w") #Create a text file too
    copy = False
    #Paste first two lines
    for idx, line in enumerate(old_text):
            if idx == 0 or idx == 1:
                text.write(line)
    #Paste the values in the compressed file
    for i in range(file_text.shape[0]):
        for j in range(elements):
            text.write("%f,%f,%f\t" % (file_text[i][j*3], file_text[i][j*3+1], file_text[i][j*3+2]))
        text.write("\n")
    old_text.close()
    text.close()
