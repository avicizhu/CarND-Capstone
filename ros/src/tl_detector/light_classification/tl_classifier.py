import rospy
import tensorflow as tf
import numpy as np
from styx_msgs.msg import TrafficLight

MIN_CLASSIFICATION_CONFIDENCE = 0.85
INFERENCE_MODEL_PATH = 'models/frozen_inference_graph_sim.pb'

class TLClassifier(object):
    def __init__(self):
        # Load classifier
        self.graph = tf.Graph()
        with self.graph.as_default():
            graph_def = tf.GraphDef()
            with tf.gfile.GFile(INFERENCE_MODEL_PATH, 'rb') as fid:
                serialized_graph = fid.read()
                graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(graph_def, name='')

            # Get input tensor from the graph
            self.image_tensor = self.get_tensor('image_tensor:0')

            # Get confidence level tensor from the graph
            self.detection_scores = self.get_tensor('detection_scores:0')

            # Get classification tensor from the graph
            self.detection_classes = self.get_tensor('detection_classes:0')

            self.sess = tf.Session(graph=self.graph)


    def get_tensor(self, name):
        return self.graph.get_tensor_by_name(name)

    def get_classification(self, image):
        """Determines the color of the traffic light in the image
        Args:
            image (cv::Mat): image containing the traffic light
        Returns:
            int: ID of traffic light color
                 (specified in styx_msgs/TrafficLight)
        """
        class_votes = np.zeros(3, dtype=int)

        with self.graph.as_default():
            # The model expects images to have shape: [1, None, None, 3]
            image_np_expanded = np.expand_dims(image, axis=0)

            # Run inference
            feed_dict = {self.image_tensor: image_np_expanded}
            (scores, classes) = self.sess.run([self.detection_scores,
                                               self.detection_classes],
                                              feed_dict=feed_dict)

            # Loop over detections and keep the ones with high confidence
            for i, score in enumerate(scores[0]):
                if score > MIN_CLASSIFICATION_CONFIDENCE:
                    class_votes[int(classes[0][i]) - 1] += 1

            if np.sum(class_votes) == 0:
                # No confident votes for any class
                output = TrafficLight.UNKNOWN
            else:
                # Pick best class
                best_class = np.argmax(class_votes) + 1

                # Return
                output = self.graph_class_to_traffic_light(best_class)

            rospy.logwarn('Traffic Light: {}'
                          .format(self.traffic_light_to_str(output)))

            return output

    def graph_class_to_traffic_light(self, graph_class):
        """ Converts from a class number as defined in the TensorFlow
            model, to a class number as defined in styx_msgs/TrafficLight
        """
        if graph_class == 1:
            return TrafficLight.GREEN
        elif graph_class == 2:
            return TrafficLight.YELLOW
        elif graph_class == 3:
            return TrafficLight.RED

        return TrafficLight.UNKNOWN

    def traffic_light_to_str(self, traffic_light):
        if traffic_light == TrafficLight.GREEN:
            return 'GREEN'
        elif traffic_light == TrafficLight.YELLOW:
            return 'YELLOW'
        elif traffic_light == TrafficLight.RED:
            return 'RED'
        return 'UNKNOWN'
