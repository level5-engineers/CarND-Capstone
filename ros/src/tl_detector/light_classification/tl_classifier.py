import tensorflow as tf
import cv2
import numpy as np
import time

# Reference: https://github.com/udacity/CarND-Object-Detection-Lab
class TLClassifier(object):
    def __init__(self, simulator):
        self.seq = 0
        self.simulator = simulator
        start = time.time()
        if simulator:
            graph_filename = 'light_classification/model/sim_graph.pb'
        else:
            graph_filename = 'light_classification/model/site_graph.pb'
        labels_filename = 'light_classification/model/sim_labels.txt'
        
        self.labels = self.load_labels(labels_filename)
        self.labels_dic = {'yellow':1,
                            'green':2,
                            'red':0,
                            'none':4}
        
        print("Initializing TensorFlow...")
        self.detection_graph = tf.Graph()
        # configure for a GPU
        config = tf.ConfigProto()
        config.gpu_options.allow_growth = True
        # load trained tensorflow graph
        with self.detection_graph.as_default():
            graph_def = tf.GraphDef()
            with tf.gfile.GFile(graph_filename, 'rb') as f:
                graph_def.ParseFromString(f.read())
                tf.import_graph_def(graph_def, name='')
            
            self.sess = tf.Session(graph=self.detection_graph, config=config)
            
            # configure input and output
            if simulator:
                self.image_tensor   = self.detection_graph.get_tensor_by_name('input:0')
                self.softmax_tensor = self.detection_graph.get_tensor_by_name('final_result:0')
                image               = np.asarray(np.random.rand(224,224,3)*2.-1.)
            else:
                self.image_tensor   = self.detection_graph.get_tensor_by_name('image_tensor:0')
                self.num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')
                self.dboxes         = self.detection_graph.get_tensor_by_name('detection_boxes:0')
                self.dscores        = self.detection_graph.get_tensor_by_name('detection_scores:0')
                self.dclasses       = self.detection_graph.get_tensor_by_name('detection_classes:0')
                image               = np.asarray(np.random.rand(300,300,3)*255, dtype="uint8")

            # initialize the network by running a randomized image
            image_expanded = np.expand_dims(image, axis=0)
            startA = time.time()
            if simulator:
                _ = self.sess.run(self.softmax_tensor, {self.image_tensor: image_expanded, 'Placeholder:0':1.0})
            else:
                _ = self.sess.run([self.dboxes, self.dscores, self.dclasses, self.num_detections],
              feed_dict={self.image_tensor: image_expanded})
        endA = time.time()
        print('Priming duration: ', endA-startA)
        end = time.time()
        print('Total initialization time: ', end-start)

    def load_labels(self, filename):
        """Read in labels, one label per line.
        From Tensorflow Example image retrain"""
        return [line.rstrip() for line in tf.gfile.GFile(filename)]

    # downsample the incoming image from the ROS message
    def scale_image(self, img):
        image_data = cv2.resize(img, (224,224))
        image_data = (image_data - 128.)/128.
        image_data = np.reshape(image_data, (1, 224,224,3))
        return image_data

    # Convert normalized box coordinates to pixels
    def to_image_coords(self, boxes, dim):
        """
        The original box coordinate output is normalized, i.e [0, 1].
        This converts it back to the original coordinates based on the
        image size. Optimized.
        """
        h, w = dim[0], dim[1]
        box_coords = [int(boxes[0]*h), int(boxes[1]*w), int(boxes[2]*h), int(boxes[3]*w)]
        return np.array(box_coords)

    def locateTL(self, image):
        box = [0, 0, 0, 0]
        with self.detection_graph.as_default():
            image_expanded = np.expand_dims(image, axis=0)
            (boxes, scores, classes, num_detections) = self.sess.run(
              [self.dboxes, self.dscores, self.dclasses, self.num_detections],
              feed_dict={self.image_tensor: image_expanded})
            
            # Remove unnecessary dimensions
            boxes   = np.squeeze(boxes)
            class_  = np.int32(np.squeeze(classes).tolist())
            scores  = np.squeeze(scores)
            
            # First occurrence where clsid<4 (traffic lights)
            index = next((i for i, clsid in enumerate(class_) if clsid < 4), None)
            if index == None:
                print 'No traffic light detected'
            elif scores[index] < 0.5:
                print 'Low confidence: ', scores[index]
            else:
                b = self.to_image_coords(boxes[index], image.shape[0:2])
                b_w = b[3] - b[1]
                ratio = (b[2] - b[0]) / (b_w + 0.00001)
                if (b_w >= 20) and (ratio > 2.0) and (ratio < 3.8):
                    #print 'Confidence: ', scores[index]
                    box = b
        return box

    # Classify a traffic light based on simple geometric properties
    # Expects a gray-scale image
    def classifyTL(self, image_data):
        # get the image center geometry
        midX = int(image_data.shape[1]/2)
        midY = int(image_data.shape[0]/2)
        thirdY = int(image_data.shape[0]/3)
        p = int(thirdY/3) #patch size
        # get the center point of each ROI
        rROI = ( int(thirdY/2) , midX )
        yROI = ( midY, midX )
        gROI = ( midY+thirdY , midX )
        # find the average from each center patch
        rROI = int(np.mean(image_data[rROI[0]-p:rROI[0]+p, rROI[1]-p:rROI[1]+p]))
        yROI = int(np.mean(image_data[yROI[0]-p:yROI[0]+p, yROI[1]-p:yROI[1]+p]))
        gROI = int(np.mean(image_data[gROI[0]-p:gROI[0]+p, gROI[1]-p:gROI[1]+p]))
        # perform simple brightness comparisons and print for humans
        if (gROI > yROI) and (gROI > rROI):
            print(">>> GREEN <<<")
        elif (yROI > gROI) and (yROI > rROI):
            print(">>> YELLOW <<<")
        elif (rROI > yROI) and (rROI > gROI):
            print(">>> RED <<<")
        if (gROI > yROI) and (gROI > rROI):
            return 1 # GO
        else:
            return 0 # STOP
            
    def saveImage(self,img):
        fname = '/home/student/xdrive/debug/out' + str(self.seq).zfill(5)+'.png'
        print "Saving tl image", fname
        cv2.imwrite(fname, img)
        self.seq += 1
  
    def get_classification(self, image):
        """Locates and determines the color of the traffic light in the image
        Args:
            image (cv::Mat): image containing the traffic light
        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        print('____________________________________________________________________')
        #print image.shape[0], image.shape[1]

        if self.simulator:
            image_data = self.scale_image(image)
            predictions, = self.sess.run(self.softmax_tensor, {self.image_tensor: image_data, 'Placeholder:0':1.0})

            # Sort to show labels in order of confidence
            top_k = predictions.argsort()[-1:][::-1]
            for node_id in top_k:
                predict_label = self.labels[node_id]
                score = predictions[node_id]
                print '%s (score = %.5f)' % (predict_label, score)
            return self.labels_dic[predict_label]

        else:
            start = time.time()
            b = self.locateTL(image)
            end = time.time()
            print 'Detection time: ', end-start
            
            # If there is no detection or low-confidence detection
            if np.array_equal(b, np.zeros(4)):
                #print ('unknown')
                signal_status = True # Go
                tlFound = False
            else:
                img_tl = cv2.cvtColor(image[b[0]:b[2], b[1]:b[3]], cv2.COLOR_RGB2HSV)[:,:,2]
                #self.saveImage(img_tl)
                signal_status = self.classifyTL(img_tl)
                #print("GO" if signal_status else "STOP")
                tlFound = True
            end = time.time()
            return 4 if signal_status else 0, tlFound, (b[0],b[1]) # return position
