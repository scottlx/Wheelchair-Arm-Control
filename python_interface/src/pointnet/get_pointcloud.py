import numpy as np
from subscriber import topic2array

def sample_data(data, num_sample):
    """ data is in N x ...
        we want to keep num_samplexC of them.
        if N > num_sample, we will randomly keep num_sample of them.
        if N < num_sample, we will randomly duplicate samples.
    """
    N = data.shape[0]
    if (N == num_sample):
        return data
    elif (N > num_sample):
        sample = np.random.choice(N, num_sample)
        return data[sample, ...]
    else:
        sample = np.random.choice(N, num_sample-N)
        dup_data = data[sample, ...]
        return np.concatenate([data, dup_data], 0)

def room2blocks(data, num_point, block_size=0.2, stride=0.2,random_sample=False, sample_num=None, sample_aug=1):
    """ Prepare block training data.
    Args:
        data: N x 6 numpy array, 012 are XYZ in meters, 345 are RGB in [0,1]
            assumes the data is shifted (min point is origin) and aligned
            (aligned with XYZ axis)
        num_point: int, how many points to sample in each block
        block_size: float, physical size of the block in meters
        stride: float, stride for block sweeping
        random_sample: bool, if True, we will randomly sample blocks in the room
        sample_num: int, if random sample, how many blocks to sample
            [default: room area]
        sample_aug: if random sample, how much aug
    Returns:
        block_datas: K x num_point x 6 np array of XYZRGB, RGB is in [0,1]

    TODO: for this version, blocking is in fixed, non-overlapping pattern.
    """
    assert(stride<=block_size)

    limit = np.amax(data, axis=0)[0:3]

    # Get the corner location for our sampling blocks
    xbeg_list = []
    ybeg_list = []
    if not random_sample:
        num_block_x = int(np.ceil((limit[0] - block_size) / stride)) + 1
        num_block_y = int(np.ceil((limit[1] - block_size) / stride)) + 1
        for i in range(num_block_x):
            for j in range(num_block_y):
                xbeg_list.append(i*stride)
                ybeg_list.append(j*stride)
    else:
        num_block_x = int(np.ceil(limit[0] / block_size))
        num_block_y = int(np.ceil(limit[1] / block_size))
        if sample_num is None:
            sample_num = num_block_x * num_block_y * sample_aug
        for _ in range(sample_num):
            xbeg = np.random.uniform(-block_size, limit[0])
            ybeg = np.random.uniform(-block_size, limit[1])
            xbeg_list.append(xbeg)
            ybeg_list.append(ybeg)

    # Collect blocks
    block_data_list = []
    idx = 0
    for idx in range(len(xbeg_list)):
       xbeg = xbeg_list[idx]
       ybeg = ybeg_list[idx]
       xcond = (data[:,0]<=xbeg+block_size) & (data[:,0]>=xbeg)
       ycond = (data[:,1]<=ybeg+block_size) & (data[:,1]>=ybeg)
       cond = xcond & ycond
       if np.sum(cond) < 100: # discard block if there are less than 100 pts.
           continue

       block_data = data[cond, :]


       # randomly subsample data
       block_data_sampled = sample_data(block_data, num_point)
       block_data_list.append(np.expand_dims(block_data_sampled, 0))


    return np.concatenate(block_data_list, 0)

def room2blocks_plus_normalized(data, num_point,block_size=0.2):
    """ room2block, with input filename and RGB preprocessing.
        for each block centralize XYZ, add normalized XYZ as 678 channels
    """
    data = data[:,0:6]
    data[:,3:6] /= 255.0
    max_room_x = max(data[:,0])
    max_room_y = max(data[:,1])
    max_room_z = max(data[:,2])
    max_room = [max_room_x,max_room_y,max_room_z]


    data_batch= room2blocks(data, num_point)
    new_data_batch = np.zeros((data_batch.shape[0], num_point, 9))
    for b in range(data_batch.shape[0]):
        new_data_batch[b, :, 6] = data_batch[b, :, 0]/max_room_x
        new_data_batch[b, :, 7] = data_batch[b, :, 1]/max_room_y
        new_data_batch[b, :, 8] = data_batch[b, :, 2]/max_room_z
        minx = min(data_batch[b, :, 0])
        miny = min(data_batch[b, :, 1])
        data_batch[b, :, 0] -= (minx+block_size/2)
        data_batch[b, :, 1] -= (miny+block_size/2)
    new_data_batch[:, :, 0:6] = data_batch
    return new_data_batch, max_room

def align_shift(input_array):
    # print(input_array.shape)
    #
    # test_out = open('test_out.obj', 'w')
    # for point_idx in range(input_array.shape[0]):
    #     test_out.write('v %f %f %f\n' % (input_array[point_idx,0], input_array[point_idx,1],input_array[point_idx,2]))
    # test_out.close()

    # from kinect to gazebo coordination
    # x = z, y=-x, z=-y
    x = input_array[:,2].copy()
    y = -input_array[:,0].copy()
    z = -input_array[:,1].copy()
    input_array[:,0] = x
    input_array[:,1] = y
    input_array[:,2] = z


    min_xyz = np.amin(input_array,axis=0)[0:3]
    input_array[:,0] -= min_xyz[0]
    input_array[:,1] -= min_xyz[1]
    input_array[:,2] -= min_xyz[2]
    sorted_array = input_array[input_array[:,0].argsort()]
    shift_history = [min_xyz[0],min_xyz[1],min_xyz[2]]

    # new_out = open('new.obj', 'w')
    # for point_idx in range(sorted_array.shape[0]):
    #     new_out.write('v %f %f %f\n' % (sorted_array[point_idx,0], sorted_array[point_idx,1],sorted_array[point_idx,2]))
    # new_out.close()
    #
    # exit()

    return sorted_array, shift_history

def pointcloud_wrapper(topic='/camera/depth/points'):
    pc_arr = topic2array(topic)
    if pc_arr is False:
        print("something error when subscribing topic, try:")
        print("rostopic list")
        print("in terminal and make sure you see /camera/depth/points")
        exit()
    pointclouds_np, shift_history = align_shift(pc_arr)
    new_data_batch, max_room = room2blocks_plus_normalized(pointclouds_np,num_point=4096)
    return new_data_batch, max_room,shift_history


if __name__ == "__main__":
    pointclouds_np = topic2array(topic)
    print(pointclouds_np.shape)
    sampled_clouds = room2blocks_plus_normalized(pointclouds_np,num_point=4096)
    print(sampled_clouds.shape)
