import os
import sys
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(BASE_DIR)
sys.path.append(BASE_DIR)
from model import *
from get_pointcloud import pointcloud_wrapper
NUM_CLASSES = 13
g_classes = [x.rstrip() for x in open(os.path.join(BASE_DIR, 'class_names.txt'))]
g_class2color = {'ceiling':	[0,255,0],
                 'floor':	[0,0,255],
                 'wall':	[0,255,255],
                 'beam':        [255,255,0],
                 'column':      [255,0,255],
                 'window':      [100,100,255],
                 'door':        [200,200,100],
                 'table':       [170,120,200],
                 'chair':       [255,0,0],
                 'sofa':        [200,100,100],
                 'bookcase':    [10,200,100],
                 'board':       [200,200,200],
                 'clutter':     [50,50,50]}
g_label2color = {g_classes.index(cls): g_class2color[cls] for cls in g_classes}


def evaluate(label_to_detect=NUM_CLASSES-1, BATCH_SIZE=1,NUM_POINT=4096,MODEL_PATH='ckpt/model.ckpt', x_offset=0.35,y_offset=0.137, z_offset=0.1,VISU = True):
    is_training = False

    pointclouds_pl, labels_pl = placeholder_inputs(BATCH_SIZE, NUM_POINT)
    is_training_pl = tf.placeholder(tf.bool, shape=())

    # simple model
    pred = get_model(pointclouds_pl, is_training_pl)

    # Add ops to save and restore all the variables.
    saver = tf.train.Saver()

    # Create a session
    config = tf.ConfigProto()
    config.allow_soft_placement = True
    config.log_device_placement = True
    sess = tf.Session(config=config)

    # Restore variables from disk.
    saver.restore(sess, MODEL_PATH)
    print("Model restored!")

    ops = {'pointclouds_pl': pointclouds_pl,
           'is_training_pl': is_training_pl,
           'pred': pred}

    out_data_label_filename = "output_prediction.txt"
    location, std = eval_one_epoch(label_to_detect, sess, ops, out_data_label_filename,BATCH_SIZE,NUM_POINT,VISU)


    x = location[2] + x_offset
    y = -location[0] + y_offset
    z = -location[1] + z_offset
    print("location based on global coordination = "+str([x,y,z]))
    print("standard deviation along x y z axis: "+str(std))
    return [x,y,z], std



def eval_one_epoch(label_to_detect, sess, ops, out_data_label_filename,BATCH_SIZE,NUM_POINT, VISU=True):
    is_training = False

    if VISU:
        fout = open('visualization_pred.obj', 'w')
    #fout_data_label = open(out_data_label_filename, 'w')

    print("getting point cloud data from rostopic...")

    # current_data,max_room, shift_history = pointcloud_wrapper()
    # np.save("current_data",current_data)
    # np.save("max_room",max_room)
    # np.save("shift_history",shift_history)
    # exit()

    current_data = np.load("current_data.npy")
    max_room = np.load("max_room.npy")
    shift_history  = np.load("shift_history.npy")


    print("point cloud shape is "+str(current_data.shape))
    print("pointnet inferencing...")
    current_data = current_data[:,0:NUM_POINT,:]
    # Get room dimension..
    max_room_x = max_room[0]
    max_room_y = max_room[1]
    max_room_z = max_room[2]
    file_size = current_data.shape[0]
    num_batches = file_size // BATCH_SIZE
    #print("file size = "+str(file_size))

    position_label = np.array([[],[],[]]).T
    for batch_idx in range(num_batches):
        start_idx = batch_idx * BATCH_SIZE
        end_idx = (batch_idx+1) * BATCH_SIZE
        cur_batch_size = end_idx - start_idx

        feed_dict = {ops['pointclouds_pl']: current_data[start_idx:end_idx, :, :],
                     ops['is_training_pl']: is_training}
        pred_val = sess.run( ops['pred'],feed_dict=feed_dict)


        # if no_clutter:
        #     pred_label = np.argmax(pred_val[:,:,0:12], 2) # BxN
        # else:
        pred_label = np.argmax(pred_val, 2) # BxN
        # Save prediction labels to OBJ file
        # create numpy array with xyz and label

        for b in range(BATCH_SIZE):
            pts = current_data[start_idx+b, :, :]
            pts[:,6] *= max_room_x
            pts[:,7] *= max_room_y
            pts[:,8] *= max_room_z
            pts[:,3:6] *= 255.0
            pred = pred_label[b, :]

            #numpy array [x,y,z,label]


            for i in range(NUM_POINT):
                color = g_label2color[pred[i]]
                if VISU:
                    fout.write('v %f %f %f %d %d %d\n' % (pts[i,6], pts[i,7], pts[i,8], color[0], color[1], color[2]))

                #fout_data_label.write('%f %f %f %d %d %d %f %d\n' % (pts[i,6], pts[i,7], pts[i,8], pts[i,3], pts[i,4], pts[i,5], pred_val[b,i,pred[i]], pred[i]))
                if pred[i]==label_to_detect:
                    position_label = np.append(position_label,[[pts[i,6], pts[i,7], pts[i,8]]],axis=0)


    #fout_data_label.close()
    if VISU:
        fout.close()
    if position_label.shape[0] > 0:
        print("found "+str(position_label.shape[0])+" points belong to the object")
        mean_location = np.mean(position_label,axis=0)
        std = np.std(position_label,axis=0)
        print("before shift location = "+str(mean_location))
        #print("shift_history = "+str(shift_history))
        shifted_location = mean_location + np.asarray(shift_history)
        #print("shifted_location = "+str(shifted_location))
        return shifted_location, std.tolist()
    else:
        print("no point recognized as "+str(label_to_detect))
        exit()

if __name__=='__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--batch_size', type=int, default=1, help='Batch Size during training [default: 1]')
    parser.add_argument('--num_point', type=int, default=4096, help='Point number [default: 4096]')
    parser.add_argument('--model_path', default='ckpt/model.ckpt', help='model checkpoint file path')
    # parser.add_argument('--no_clutter', action='store_true', help='If true, donot count the clutter class')
    FLAGS = parser.parse_args()

    BATCH_SIZE = FLAGS.batch_size
    NUM_POINT = FLAGS.num_point
    MODEL_PATH = FLAGS.model_path




    with tf.Graph().as_default():
        evaluate()
