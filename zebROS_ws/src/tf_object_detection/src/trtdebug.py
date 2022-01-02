#! /usr/bin/env python2
"""
Adaped from	https://github.com/AastaNV/TRT_object_detection.git
A faster way to optimize models to run on the Jetson
This script has 2 parts. First is to convert the model to UFF format and then
optimize that using tensorRT.  This produces a .bin file.
The .bin file is then loaded and used to run inference on a video.
"""

import os
import sys
import cv2
import time
import ctypes
import numpy as np
from threading import Semaphore
import jetson.utils

import pycuda.driver as cuda
import rospy
from sensor_msgs.msg import Image
from field_obj.msg import TFDetection, TFObject
from cv_bridge import CvBridge, CvBridgeError
import rospkg

import uuid

import tensorrt as trt
# from config import model_ssd_inception_v2_coco_2017_11_17 as model
# from config import model_ssd_mobilenet_v1_coco_2018_01_28 as model
# from config import model_ssd_mobilenet_v2_coco_2018_03_29 as model
# from config import retinanet_mobilenet_v2_400x400 as model
from config import model_ssd_mobilenet_v2_512x512 as model
# from config import model_ssd_mobilenet_v3 as model
from visualization import BBoxVisualization
import timing
from object_detection.utils import label_map_util

ctypes.CDLL("/home/ubuntu/TensorRT/build/libnvinfer_plugin.so")
# CLASS_LABELS = class_labels.CLASSES_LIST
bridge = CvBridge()
category_index, detection_graph, sess, pub, pub_debug, vis = None, None, None, None, None, None
min_confidence = 0.1

# viz = BBoxVisualization(category_dict)



global init
init = False


def run_inference_for_single_image(msg):
    global init, host_inputs, cuda_inputs, host_outputs, cuda_outputs, stream, context, bindings, host_mem, cuda_mem, cv2gpu, imgResized, imgNorm, gpuimg, finalgpu

    # print("Starting inference")
    if init == False:

        init = True
        ori = bridge.imgmsg_to_cv2(msg, "bgr8")
        imgInput = jetson.utils.cudaFromNumpy(ori, isBGR=True)
        # Could be better nmessage
        print("Performing Init for CUDA")

        import pycuda.autoinit
        # initialize
        TRT_LOGGER = trt.Logger(trt.Logger.INFO)
        trt.init_libnvinfer_plugins(TRT_LOGGER, '')
        runtime = trt.Runtime(TRT_LOGGER)
        # compile model into TensorRT
        # This is only done if the output bin file doesn't already exist
        # TODO - replace this with the MD5 sum check we have for the other TRT detection
        if not os.path.isfile(model.TRTbin):
            import uff
            import graphsurgeon as gs
            dynamic_graph = model.add_plugin(gs.DynamicGraph(model.path))
            uff_model = uff.from_tensorflow(dynamic_graph.as_graph_def(), model.output_name, output_filename='tmp.uff')
            with trt.Builder(TRT_LOGGER) as builder, builder.create_network() as network, trt.UffParser() as parser:
                builder.max_workspace_size = 1 << 29
                builder.max_batch_size = 1
                # builder.fp16_mode = True
                parser.register_input('Input', model.dims)
                parser.register_output('MarkOutput_0')
                parser.parse('tmp.uff', network)
                engine = builder.build_cuda_engine(network)
                buf = engine.serialize()
                with open(model.TRTbin, 'wb') as f:
                    f.write(buf)
        # Start of inference code
        # create engine

        with open(model.TRTbin, 'rb') as f:
            buf = f.read()
            engine = runtime.deserialize_cuda_engine(buf)
        # create buffers
        host_inputs = []
        cuda_inputs = []
        host_outputs = []
        cuda_outputs = []
        bindings = []
    
        stream = cuda.Stream()
        for binding in engine:
            size = trt.volume(engine.get_binding_shape(binding)) * engine.max_batch_size
            host_mem = cuda.pagelocked_empty(size, np.float32)
            cuda_mem = cuda.mem_alloc(host_mem.nbytes)
            bindings.append(int(cuda_mem))
            print("Host mem=", host_mem, "CUDA mem", cuda_mem ,"Bindings=", bindings) 
            
            if engine.binding_is_input(binding):
                host_inputs.append(host_mem)
                cuda_inputs.append(cuda_mem)
            else:
                host_outputs.append(host_mem)
                cuda_outputs.append(cuda_mem)
        context = engine.create_execution_context()
        print("Host inputs=", host_inputs, "Cuda Inputs=", cuda_inputs)
        
       # List of the strings that is used to add correct label for each box.
        # print("Bindings are =" + str(bindings))
        rospack = rospkg.RosPack()
        THIS_DIR = os.path.join(rospack.get_path('tf_object_detection'), 'src/')
        PATH_TO_LABELS = os.path.join(THIS_DIR, '2020Game_label_map.pbtxt')
        rospy.logwarn("Loading labels from " + str(PATH_TO_LABELS))

        category_index = label_map_util.create_category_index_from_labelmap(PATH_TO_LABELS, use_display_name=True)
        category_dict = {0: 'background'}
        for k in category_index.keys():
            category_dict[k] = category_index[k]['name']

        finalgpu = jetson.utils.cudaAllocMapped(width=model.dims[1], height=model.dims[2], format='rgb32f')

    infrencetime_s = time.time()
    ori = bridge.imgmsg_to_cv2(msg, "bgr8")
    # Trying with gpu
    starttime = time.time()
    imgInput = jetson.utils.cudaFromNumpy(ori, isBGR=True)
    #Normalizes Image between the two values
    imagerange = (-1., 1.)
    #Custom function, no documentation
    #Finalgpu is on the gpu
    jetson.utils.cudaTensorConvert(imgInput, finalgpu, imagerange)
    print("Image preprocess time", time.time() - infrencetime_s)
    # This is should be replaced with somehow pointing the cuda_inputs to finalgpu (hopefully)
    copytime = time.time()
    # Gets the array on CPU
    #finalarray = jetson.utils.cudaToNumpy(finalgpu)
    host_inputs[0] = jetson.utils.cudaToNumpy(finalgpu).ravel()
    #finalarray = finalarray.ravel()
    # Now copies to another location on the cpu
    
    # This is very slow
    #np.copyto(host_inputs[0], finalarray.ravel())
    
    
    cuda.memcpy_htod_async(cuda_inputs[0], host_inputs[0], stream) 
    print("CopyTime took", time.time() - copytime)
    inferencetime = time.time()
    context.execute_async(bindings=bindings, stream_handle=stream.handle)

    # cuda.memcpy_dtoh_async(host_outputs[1], cuda_outputs[1], stream)
    cuda.memcpy_dtoh_async(host_outputs[0], cuda_outputs[0], stream)
    #print("Cuda outputs[0]", cuda_outputs[0])
    stream.synchronize()
    print("Inference took", time.time() - inferencetime)
    publishtime = time.time()
    output = host_outputs[0]

    height, width, channels = ori.shape
    boxes = []
    confs = []
    clss = []
    detection = TFDetection()
    detection.header = msg.header
    detection.header.frame_id = detection.header.frame_id.replace("_optical_frame", "_frame")
    for i in range(int(len(output) / model.layout)):
        prefix = i * model.layout
        
        index = int(output[prefix + 0])
        label = str(output[prefix + 1])
        conf = float(output[prefix + 2])
        xmin = float(output[prefix + 3] * width)
        ymin = float(output[prefix + 4] * height)
        xmax = float(output[prefix + 5] * width)
        ymax = float(output[prefix + 6] * height)
        
        
        # ROSifying the code
        obj = TFObject()
        obj.confidence = conf
        obj.tl.x = xmax
        obj.tl.y = ymax
        obj.br.x = xmin
        obj.br.y = ymin
        obj.id = index
        obj.label = label
        detection.objects.append(obj)

        boxes.append([output[prefix + 4], output[prefix + 3], output[prefix + 6], output[prefix + 5]])
        clss.append(int(output[prefix + 1]))
        confs.append(output[prefix + 2])
    # print(detection)
    pub.publish(detection)
    end = time.time()
    print("Publish time", time.time() - publishtime)
    inferencetime = end - infrencetime_s
    print("FINAL TIME", inferencetime)
    # writes published msg to file for easier viewing, only stores most recent msg
    os.chdir("/home/ubuntu/2020Offseason/zebROS_ws/src/tf_object_detection/src")

    f = open("finalgpuoptim.txt", "w+")
    f.write(str(detection))
    f.close()
    # viz.draw_bboxes(ori, boxes, confs, clss, 0.42)
    # cv2.imwrite("result.jpg", ori)
    # cv2.imshow("result", ori)

    # context.pop()
    print("Context popped!")


def main():
    os.chdir("/home/ubuntu/2020Offseason/zebROS_ws/src/tf_object_detection/src")

    global detection_graph, sess, pub, category_index, pub_debug, min_confidence, vis

    sub_topic = "/c920/rect_image"
    pub_topic = "obj_detection_msg"
    rospy.init_node('tf_object_detection', anonymous=True)
    min_confidence = 0.1
    if rospy.has_param('min_confidence'):
        min_confidence = rospy.get_param('min_confidence')
    else:
        print("Unable to get min confidence, defaulting to 0.1")
    if rospy.has_param('image_topic'):
        sub_topic = rospy.get_param('image_topic')
    else:
        print("Unable to get image topic, defaulting to /c920/rect_image")

    sub = rospy.Subscriber(sub_topic, Image, run_inference_for_single_image)
    pub = rospy.Publisher(pub_topic, TFDetection, queue_size=2)
    pub_debug = rospy.Publisher("debug_image", Image, queue_size=1)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    # maxthreads = Semaphore(1)
    # with maxthreads:
    main()

