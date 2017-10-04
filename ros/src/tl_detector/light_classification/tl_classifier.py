#from styx_msgs.msg import TrafficLight
import tensorflow as tf

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        labels_filename = 'tl_detector/light_classification/model/sim_labels.txt'
        graph_filename = 'tl_detector/light_classification/model/sim_graph.pb'

        self.labels = self.load_labels(labels_filename)
        self.load_graph(graph_filename)
        self.labels_dic = {'yellow':1,
                            'green':2,
                            'red':0,
                            'none':4}


    def load_labels(self, filename):
        """Read in labels, one label per line.
        From Tensorflow Example image retrain"""
        return [line.rstrip() for line in tf.gfile.GFile(filename)]

    def load_image(self, filename):
        """Read in the image_data to be classified."""
        return tf.gfile.FastGFile(filename, 'rb').read()

    def load_graph(self, filename):
        """Unpersists graph from file as default graph."""
        with tf.gfile.FastGFile(filename, 'rb') as f:
            graph_def = tf.GraphDef()
            graph_def.ParseFromString(f.read())
            tf.import_graph_def(graph_def, name='')

    def run_graph(self, image_data, labels, input_layer_name, output_layer_name,
              num_top_predictions=1):
        with tf.Session() as sess:
        # Feed the image_data as input to the graph.
        #   predictions will contain a two-dimensional array, where one
        #   dimension represents the input image count, and the other has
        #   predictions per class
            softmax_tensor = sess.graph.get_tensor_by_name(output_layer_name)
            predictions, = sess.run(softmax_tensor, {input_layer_name: image_data})

            # Sort to show labels in order of confidence
            top_k = predictions.argsort()[-num_top_predictions:][::-1]
            for node_id in top_k:
                human_string = labels[node_id]
                score = predictions[node_id]
                print('%s (score = %.5f)' % (human_string, score))

            return human_string



    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        image_data = self.load_image(image)
        predict_label = self.run_graph(image_data, self.labels, 'input:0', 'final_result:0',
            1)


        return self.labels_dic[predict_label]
