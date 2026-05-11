#!/usr/bin/env python3

import rospy
from gmm.gmm_logic import Gmm
from planner_msgs.msg import Graph, Vertex, Edge, Merge
from sklearn.mixture import GaussianMixture
import numpy as np
from scipy.special import kl_div
from scipy.spatial.distance import jensenshannon

class GmmNode:
    def __init__(self):
        rospy.init_node("Gmm_node")

        #Subscribers
        self.gmm_sub = rospy.Subscriber("gmm_node/evaluate_gmm",
                                    Merge,
                                    self.evaluateGmmCallback,
                                    queue_size=10)

        self.kl_div_sub = rospy.Subscriber("gmm_node/pdf_similarity",
                                    Graph,
                                    self.similarityCallback,
                                    queue_size=10)

        # Publishers
        self.gmm_pub = rospy.Publisher("gmm_node/vertices_to_keep",
                                    Merge,
                                    queue_size=10)

        self.msg_storing = []
        self.counter = 0

    def similarityCallback(self, graph_msg):
        if self.counter < 1:
            self.counter += 1
            self.msg_storing.append(graph_msg)
        else:
            data1 = np.array([[v.pose.position.x, v.pose.position.y] for v in self.msg_storing[0].vertices])
            data2 = np.array([[v.pose.position.x, v.pose.position.y] for v in graph_msg.vertices])

            data1 = np.vstack(data1)
            data2 = np.vstack(data2)

            gmm1 = GaussianMixture(n_components=6).fit(data1)
            gmm2 = GaussianMixture(n_components=6).fit(data2)

            #Generate a grid of points to evaluate the PDFs
            x_min = min(data1[:,0].min(), data2[:,0].min())
            x_max = max(data1[:,0].max(), data2[:,0].max())
            y_min = min(data1[:,1].min(), data2[:,1].min())
            y_max = max(data1[:,1].max(), data2[:,1].max())

            xx, yy = np.meshgrid(
                np.linspace(x_min, x_max, 100),
                np.linspace(y_min, y_max, 100)
            )

            X = np.vstack([xx.ravel(), yy.ravel()]).T

            density1 = np.exp(gmm1.score_samples(X))
            density2 = np.exp(gmm2.score_samples(X))

            # Normalize the densities
            density1 /= density1.sum()
            density2 /= density2.sum()

            # The final value of the KL divergence is 0 <= KL <= 1
            # 0 means that the two distributions are identical, while 1 means that they are completely different
            kl_value = kl_div(density1, density2).sum()
            print("KL Divergence:", kl_value)

    def evaluateGmmCallback(self, msg):
        rospy.loginfo("Starting the Gassian Mixture Model evaluation...")

        gmm = Gmm(6, msg.input_graph, msg.neighbour_graph)
        keep_query_vertices = gmm.fitGaussianMixtureModel(50)

        # Build the graph message to send back
        graphs_msg_pub = Merge()

        graphs_msg_pub.input_graph = msg.input_graph
        graphs_msg_pub.neighbour_graph = msg.neighbour_graph

        # Re-build the vertices graph msg only with vertices to keep from the GMM
        for v in keep_query_vertices:
            vertex = Vertex()
            vertex.id = v.id
            vertex.pose.position.x = v.x
            vertex.pose.position.y = v.y
            vertex.pose.position.z = v.z
            graphs_msg_pub.vertices_to_keep.append(vertex)

        # Keep track of how many vertices have been filtered by the GMM
        init_num_vertices = len(msg.neighbour_graph.vertices)
        final_num_vertices = len(graphs_msg_pub.vertices_to_keep)
        rospy.loginfo(f"Filtered vertices: {init_num_vertices - final_num_vertices}")

        # Publish to the merging node the own graph, the received one and the vertices to be kept from the received graph
        self.gmm_pub.publish(graphs_msg_pub)

        rospy.loginfo("Finished Gaussian Mixture Model evaluation!")

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    gmm_node = GmmNode()
    gmm_node.run()