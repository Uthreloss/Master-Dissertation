#!/usr/bin/env python
print 1234/10
# import numpy as np
# with open("/home/pepper/catkin_ws/src/master_dissertation/experiment_bags/11P1.txt") as f:
#     file = f.read()
# fileS = file.split('\n')
# data = fileS[2:-1]
# elements = int(data[0].split('\t')[-1])
# coordinates = 3
# file_text = np.zeros((3,elements*coordinates))
# for z in range(3):
#     with open("/home/pepper/catkin_ws/src/master_dissertation/experiment_bags/11P"+ str(z+1) +".txt") as f:
#         file = f.read()
#     fileS = file.split('\n') #Separate rows
#     data = fileS[2:-1]
#     rows=len(data)
#     #elements = int(data[0].split('\t')[-1]) #Separate columns and give las value for the first row
#     #print elements
#     compressed = np.zeros(elements*coordinates)
#     for i in range(elements):
#         for j in range(rows):
#             tri = data[j].split('\t')[i]
#             compressed[i*coordinates]= compressed[i*coordinates] + float(tri.split(',')[0])
#             compressed[(i*coordinates)+1]= compressed[(i*coordinates)+1] + float(tri.split(',')[1])
#             compressed[(i*coordinates)+2]= compressed[(i*coordinates)+2] + float(tri.split(',')[2])
#     compressed = compressed/rows
#     file_text[z,:]=compressed
# print file_text
#     # print numbers
#     # print "hey"
#     #print singles
# old_text = open("/home/pepper/catkin_ws/src/master_dissertation/experiment_bags/11P1.txt","r") #Create a text file too
# text = open("/home/pepper/catkin_ws/src/master_dissertation/experiment_bags/11PC.txt","w") #Create a text file too
# copy = False
# for idx, line in enumerate(old_text):
#         if idx == 0 or idx == 1:
#             text.write(line)
#
# for i in range(file_text.shape[0]):
#     for j in range(elements):
#         text.write("%f,%f,%f\t" % (file_text[i][j*3], file_text[i][j*3+1], file_text[i][j*3+2]))
#     text.write("\n")
