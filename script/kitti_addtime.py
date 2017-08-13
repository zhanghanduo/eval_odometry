#!/usr/bin/python
"""
The Kinect provides the color and depth images in an un-synchronized way. This means that the set of time stamps from the color images do not intersect with those of the depth images. Therefore, we need some way of associating color images to depth images.

For this purpose, you can use the ''associate.py'' script. It reads the time stamps from the rgb.txt file and the depth.txt file, and joins them by finding the best matches.
"""

import argparse
import sys
import os
import numpy

def read_file_list(filename1, filename2):
    """
    Reads trajectory from 2 text files. 
    
    File format:
    The file format is "d1 d2 d3 ...", where "d1 d2 d3.." is arbitary data (e.g., a 3D position and 3D orientation) not associated to this timestamp. 
    The file format is "stamp", where stamp denotes the time stamp (to be matched). 
    
    Input:
    filename -- File name
    
    Output:
    list
    
    """
    file1 = open(filename1, 'r').readlines()
    file2 = open(filename2, 'r').readlines()
    N=len(file1)
    listc=[]
    for i in range(0,N):
        list=[]
        w1=file1[i].split("\n")
        w2=file2[i].split(" ")
        l1=w1[0]
        time = float(l1)
        w3=w2
        for k in range(0,12):
            w3[k] = float(w2[k])
        list.append(time)
        for v in range(0,12):
            list.append(w3[v])
        listc.append(list)
    return listc


if __name__ == '__main__':
    print 1
    # parse command line
    parser = argparse.ArgumentParser(description='''ground truth to txt file with timestamps''')
    parser.add_argument('timestampfile', help='input timestamp')
    parser.add_argument('posefile', help='input poses')
    parser.add_argument('outputtxt', help='out text file')
    args = parser.parse_args()

    print "Processing bag file:"
    print "  in:",args.timestampfile
    print "  in:",args.posefile
    print "  out:",args.outputtxt

    # second_list = read_pose_list(args.posefile)
    cc_list = read_file_list(args.timestampfile, args.posefile)
    outtxt = open(args.outputtxt,'w')

    for t1 in cc_list:
        for cc in range(0,12):
            outtxt.write(str.format("{0:.9f} ", t1[cc]))
        
        outtxt.write(str.format("{0:.9f}\n", t1[12]))