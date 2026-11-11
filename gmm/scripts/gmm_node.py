#!/usr/bin/env python3

import rospy
from gmm.gmm_logic import Gmm
from planner_msgs.msg import Graph, Vertex, Edge, Merge
from sklearn.mixture import GaussianMixture
import numpy as np
from scipy.special import kl_div
from scipy.spatial.distance import jensenshannon

import os
import time
import re

class GmmNode:
    def __init__(self):
        rospy.init_node("Gmm_node")

        # Added for ablation study
        self.n_components = rospy.get_param("~n_components", 6)

        # Robot id: taken from the namespace, which is always rmf_obelix_<id>
        ns = rospy.get_namespace().strip("/")
        match = re.search(r"rmf_obelix_(\d+)", ns)
        self.robot_id = match.group(1) if match else "unknown"

        if self.robot_id == "unknown":
            rospy.logwarn(f"Could not parse robot id from namespace '{ns}'")

        # Folder where the GMM computation times are tracked
        default_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "timing_logs")
        self.log_dir = os.path.abspath(rospy.get_param("~timing_log_dir", default_dir))
        os.makedirs(self.log_dir, exist_ok=True)

        # One file per robot and per number of components
        self.time_file = os.path.join(
            self.log_dir,
            f"connectivitypitt_gmm_time_robot_{self.robot_id}_nc_{self.n_components}_run_1.csv"
        )

        # Write the header only once, when the file does not exist yet
        if not os.path.exists(self.time_file):
            with open(self.time_file, "w") as f:
                f.write("timestamp,n_components,n_input_vertices,n_query_vertices,n_kept,time_s\n")

        rospy.loginfo(f"GMM timing log: {self.time_file}")

        rospy.loginfo(f"GMM node using n_components={self.n_components}")

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

    def logComputationTime(self, elapsed, n_input, n_query, n_kept):
        with open(self.time_file, "a") as f:
            f.write(f"{time.time():.3f},{self.n_components},{n_input},{n_query},{n_kept},{elapsed:.6f}\n")

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

    # def evaluateGmmCallback(self, msg):
    #     rospy.loginfo("Starting the Gassian Mixture Model evaluation...")

    #     gmm = Gmm(self.n_components, msg.input_graph, msg.neighbour_graph)
    #     keep_query_vertices = gmm.fitGaussianMixtureModel(50)

    #     # Build the graph message to send back
    #     graphs_msg_pub = Merge()

    #     graphs_msg_pub.input_graph = msg.input_graph
    #     graphs_msg_pub.neighbour_graph = msg.neighbour_graph

    #     # Re-build the vertices graph msg only with vertices to keep from the GMM
    #     for v in keep_query_vertices:
    #         vertex = Vertex()
    #         vertex.id = v.id
    #         vertex.pose.position.x = v.x
    #         vertex.pose.position.y = v.y
    #         vertex.pose.position.z = v.z
    #         graphs_msg_pub.vertices_to_keep.append(vertex)

    #     # Keep track of how many vertices have been filtered by the GMM
    #     init_num_vertices = len(msg.neighbour_graph.vertices)
    #     final_num_vertices = len(graphs_msg_pub.vertices_to_keep)
    #     rospy.loginfo(f"Filtered vertices: {init_num_vertices - final_num_vertices}")

    #     # Publish to the merging node the own graph, the received one and the vertices to be kept from the received graph
    #     self.gmm_pub.publish(graphs_msg_pub)

    #     rospy.loginfo("Finished Gaussian Mixture Model evaluation!")
    def evaluateGmmCallback(self, msg):
        rospy.loginfo("Starting the Gassian Mixture Model evaluation...")

        # Graph loading: NOT timed
        gmm = Gmm(self.n_components, msg.input_graph, msg.neighbour_graph)

        # Time only the GMM fit + density filtering
        start = time.perf_counter()
        keep_query_vertices = gmm.fitGaussianMixtureModel(30)
        elapsed = time.perf_counter() - start

        # Build the graph message to send back
        graphs_msg_pub = Merge()
        graphs_msg_pub.input_graph = msg.input_graph
        graphs_msg_pub.neighbour_graph = msg.neighbour_graph

        for v in keep_query_vertices:
            vertex = Vertex()
            vertex.id = v.id
            vertex.pose.position.x = v.x
            vertex.pose.position.y = v.y
            vertex.pose.position.z = v.z
            graphs_msg_pub.vertices_to_keep.append(vertex)

        init_num_vertices = len(msg.neighbour_graph.vertices)
        final_num_vertices = len(graphs_msg_pub.vertices_to_keep)
        rospy.loginfo(f"Filtered vertices: {init_num_vertices - final_num_vertices}")
        rospy.loginfo(f"GMM fit + filter time: {elapsed:.4f} s")

        # Append the time to the tracking file
        self.logComputationTime(elapsed,
                                len(msg.input_graph.vertices),
                                init_num_vertices,
                                final_num_vertices)

        self.gmm_pub.publish(graphs_msg_pub)
        rospy.loginfo("Finished Gaussian Mixture Model evaluation!")

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    gmm_node = GmmNode()
    gmm_node.run()
