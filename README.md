# eval_odometry
## executable file: eval_odometry can evaluate kitti format trajectory file
   -- with ground truth and estimated poses together with timestamp. So poses are aligned by time, not by distance.
    
 Usage: ./eval_odometry [optional: -num=1 -prefix=try1 -offset=0 -maxdiff=0.02 -inithead=120]
where:
  num:      number of evaluation
  prefix:   name prefix of groundtruth and slam pose file, default: ""
  offset:   offset during alignment of two files, default = 0
  maxdiff:  maximum value difference between aligned frames, default=0.03
  inithead: initial heading with reference to North, default=120
  
  
  
## executable file: eval_kitti
  Original kitti evaluation c++ file
