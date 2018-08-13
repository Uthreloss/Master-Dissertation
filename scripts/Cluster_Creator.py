#!/usr/bin/env python
import numpy as np
#This function return the average of TXT files in a new txt file

#Core variables
itNumber = 6 #Number of iterations
Positions = 3
Participants = 10
Set = 4
#Read the first document to get number of columns
with open("/home/pepper/catkin_ws/src/master_dissertation/experiment_bags/participant_1/set_1/P_1/P_1_1.txt") as f:
    file = f.read()
fileS = file.split('\n') #Rows
data = fileS[2:-1]
elements = int(data[0].split('\t')[-1]) #Number of columns per .txt
coordinates = 3
file_text = np.zeros((itNumber, elements*coordinates))
for p in range(Participants):
    for s in range(Sets):
        for w in range(Positions):
            for z in range(itNumber):
                #                                                                                       Participant            Set                       Position Number            Iteration
                with open("/home/pepper/catkin_ws/src/master_dissertation/experiment_bags/participant_" + str(p+1) + "/set_" + str(s+1) + "/P_" + str(w+1) + "/P_" + str(w+1)+ "_" + str(z+1) + ".txt") as f:
                    fileX = f.read()
                fileS = fileX.split('\n') #Separate rows
                data = fileS[2:-1]
                rows=len(data)
                #elements = int(data[0].split('\t')[-1]) #Separate columns and give las value for the first row
                #print elements
                compressed = np.zeros(elements*coordinates) #Array to contain the averaged values
                for i in range(elements):
                    for j in range(rows):
                        #Average calculation
                        tri = data[j].split('\t')[i] #"i" is after split is coincides with a pack of XYZ data
                        compressed[i*coordinates]= compressed[i*coordinates] + float(tri.split(',')[0]) # X figure addition
                        compressed[(i*coordinates)+1]= compressed[(i*coordinates)+1] + float(tri.split(',')[1]) # Y figure addition
                        compressed[(i*coordinates)+2]= compressed[(i*coordinates)+2] + float(tri.split(',')[2]) # Z figure addition
                compressed = compressed/rows #Final step to calculate the average
                file_text[z,:]=compressed

            #Debugging lines
                #print file_text
                # print numbers
                # print "hey"
                #print singles

        #Copy AND Paste

            #Open any old document to copy the first two lines
            old_text = open("/home/pepper/catkin_ws/src/master_dissertation/experiment_bags/participant_" + str(p+1) + "/set_" + str(s+1) + "/P_" + str(w+1) + "/P_" + str(w+1)+ "_" + str(z+1) + ".txt","r") #Create a text file too
            text = open("/home/pepper/catkin_ws/src/master_dissertation/experiment_bags/participant_" + str(p+1) + "/set_" + str(s+1) + "/P_" + str(w+1) + "_Av.txt","w") #Create a text file too
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


