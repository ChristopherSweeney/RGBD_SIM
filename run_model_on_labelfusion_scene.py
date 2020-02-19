import cv2
import matplotlib.pyplot as plt
import os
from scipy import misc
import numpy as np
import imageio
from RGBDCNN import network, data
import glob
import scipy
import time


#method for organizing files to process
def gen_samples(directory,shuffle = True,filter_files=None):
    samples = []
    dirs = os.listdir(directory)
    for i in dirs:
        if filter_files and i in filter_files:
            path = os.path.join(directory, i)+"/"
            if os.access(path, os.R_OK):
                gt_depth = sorted(glob.glob(path+"*_depth_*"))
                depth = sorted(glob.glob(path+"*_depth.png"))
                rgb = sorted(glob.glob(path+"*rgb.png"))
                samples.extend(zip(gt_depth,depth,rgb))
    if shuffle:
        random.shuffle(samples)
    return samples 

def save_image_uint16(name, im):
    array_as_uint16 = im.astype(np.uint16)
    imageio.imwrite(name, array_as_uint16)

def save_image_uint8(name, im):
    array_as_uint8 = im.astype(np.uint8)
    imageio.imwrite(name, array_as_uint8)


def save_image_colormap(name, im):
    plt.imsave(name, im, cmap=plt.cm.inferno)


def save_depth_colormap(name, im, near, far):
    cmapped = plt.cm.jet((far - im)/(far - near))
    zero_range_mask = im < near
    cmapped[:, :, 0][zero_range_mask] = 0
    cmapped[:, :, 1][zero_range_mask] = 0
    cmapped[:, :, 2][zero_range_mask] = 0
    imageio.imwrite(name, (cmapped*255.).astype(np.uint8))

#viz method for prediction from directory
def viz_predicted_depth1(path,model_path,sleep =.1,filter_files= None,img_height=480,img_width=640,save_dir = None,viz=True): #add file filter for specific logs
    model = network.load_trained_model(weights_path = model_path)
    samples = gen_samples(path,False,filter_files=filter_files)

    stack = np.zeros((1,img_height,img_width,1))

    max_depth = 3500.

    #threshold where NDP probabilities greater than it will be classified as NDP, 
    #NDP probabilities lower will be instiatiated with correlated noise process
    threshold = .5

    ax1 = plt.subplot(1,4,1)
    ax2 = plt.subplot(1,4,2)
    ax3 = plt.subplot(1,4,3)
    ax4 = plt.subplot(1,4,4)

    im1 = ax1.imshow(misc.imread(samples[0][1]))
    im2 = ax2.imshow(misc.imread(samples[0][0]))
    im3 = ax3.imshow(misc.imread(samples[0][0])/max_depth)
    im4 = ax4.imshow(misc.imread(samples[0][2]))
    plt.ion()

    for i in range(len(samples)):
        #read images
        rgb = misc.imread(samples[i][2])
        depth = misc.imread(samples[i][1])
        gtdepth = misc.imread(samples[i][0])/max_depth
        stack[0,:,:,0] = gtdepth
        gt_copy = np.copy(gtdepth)

        # predict NDP
        predicted_prob_map = model.predict_on_batch(stack)

        # apply NDP to 'perfect sim'
        network.apply_mask(predicted_prob_map, gtdepth, threshold)

        im1.set_data(depth)
        im2.set_data(gtdepth*max_depth)
        im3.set_data(gt_copy)
        im4.set_data(rgb)

        if save_dir:
            save_image_uint8(save_dir+"%010d_rgb.png" % i, rgb)
            save_depth_colormap(save_dir + "%010d_depth.png" % i, depth, 0, max_depth)
            save_depth_colormap(save_dir+"%010d_gtdepth.png" % i, gt_copy*max_depth, 0, max_depth)
            save_depth_colormap(save_dir+"%010d_predicted_depth.png" % i, gtdepth*max_depth, 0, max_depth)
            save_image_colormap(save_dir+"%010d_mask.png" % i, predicted_prob_map[0, :, :, 0])
        plt.pause(sleep)

    plt.ioff() # due to infinite loop, this gets never called.
    plt.show()

    
if __name__ == '__main__':
    save_dir = "./"
    viz_predicted_depth1(sleep =.05,filter_files = "2017-06-16-24",path = "./",model_path = "DepthSim/python/models/net_depth_seg_v1.hdf5",save_dir="2017-06-16-24-render/")