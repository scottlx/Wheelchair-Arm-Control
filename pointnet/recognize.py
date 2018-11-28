from inference import evaluate

#label and their number
# 0  ceiling
# 1  floor
# 2  wall
# 3  beam
# 4  column
# 5  window
# 6  door
# 7  table
# 8  chair
# 9  sofa
# 10 bookcase
# 11 board
# 12 clutter


#----------------------------------------------------------------------------
# this is usage example of pointnet recognition in gazebo with kinect
#----------------------------------------------------------------------------


location, std = evaluate(label_to_detect=12, x_offset=0.35,y_offset=0.137, z_offset=0.1)
#you will have mean location std of the object belong to clutter class
#if you run this on tensorflow-cpu, it will be slow when doing
#neural network inference, proberly 60 secs.
print(location)
print(std)
