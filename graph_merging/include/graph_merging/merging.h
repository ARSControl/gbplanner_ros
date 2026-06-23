#include <ros/ros.h>

#include <planner_msgs/Edge.h>
#include <planner_msgs/Graph.h>
#include <planner_msgs/Vertex.h>
#include <planner_msgs/Merge.h>
#include <planner_msgs/pci_stop.h>
#include <kdtree/kdtree.h>
#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <std_msgs/Bool.h>

#include "planner_common/graph.h"
#include "planner_common/graph_base.h"
#include "planner_common/graph_manager.h"
#include "planner_common/params.h"

namespace merge{

  class GraphMerger{

  public:

    GraphMerger(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

    void initializeAttributes();

    struct Centroid {
      Centroid(int id, double x, double y, double z){
        centroid_id = id;
        centroid_x = x;
        centroid_y = y;
        centroid_z = z;
      }
      // Unique id for each centroid.
      int centroid_id;

      //Centroid coordinates
      double centroid_x;
      double centroid_y;
      double centroid_z;
    };

    // Reset all the parameters and data structures needed for the merging process
    void reset();
    // Generate the ID for the centroid
    int generateCentroidID();
    // Generate the ID for the group of vertices
    int generateGroupID();

    // Merging process
    void mergingProcess();

    // Merging process removing vertices (not to use)
    void graphMergingVertexDeletion(std::unordered_map<int, std::vector<Vertex*>>& group_vertices_);

    // Merging process creating a new graph choosing the closest centroid node as the supernode
    void mergeWithRepresentativeNodes(std::unordered_map<int, std::vector<Vertex*>>& group_vertices_);

    // Compute distance between a vertex and a group centroid
    double distance(const Vertex* v, const Centroid* centroid);

    // Find the closest group given a vertex
    void findClosestGroup(std::unordered_map<int, std::vector<Vertex*>>& group_vertices_, 
                          std::unordered_map<int, Centroid*>& group_centroids_list_);

    // Compute the centroid position
    void computeCentroids(std::unordered_map<int, std::vector<Vertex*>>& group_vertices_, 
                          std::unordered_map<int, Centroid*>& group_centroids_list_);

    // Returns the number of centroids
    int getNumCentroids();

    // Delete centroids pointers
    void cleanCentroids();

    // Returns the nearest centroid if found
    bool getNearestCentroid(const StateVec* state, Centroid** c_res);

    // Save the graph into a .txt file
    void saveGraphInFile(const std::string& filename, const std::shared_ptr<GraphManager>& graph);

    // Re-build the graph after Gaussian Mixture Model filtering
    void filteredSubgraphCallback(const planner_msgs::Merge& msg);

    // Load all the graphs from .bin file (for test purposes on Gaussian Mixture Model)
    void loadGraphsCallback(const std_msgs::Bool::ConstPtr& msg);

    // Perform the merging process via reading .bin files
    void graphMergeFromFile();

    // Melo
    bool loadParams();

    std::string world_frame_ = "world";

  private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    // Graph stuctures
    std::shared_ptr<GraphManager> global_graph_manager_;
    std::shared_ptr<GraphManager> new_graph_;

    // Subscribers
    ros::Subscriber global_graph_subscriber_;
    ros::Subscriber test_;
    ros::Subscriber gmm_sub_;

    // Publishers
    ros::Publisher merged_graph_pub_;
    ros::Publisher gmm_pub_;
    ros::Publisher kl_div_pub_;

    ros::ServiceClient stop_planner_client_;

    // For debugging purposes
    std::unordered_map<int, std::vector<int>> vertex_to_candidate_groups_;

    // Data structures
    // <Group_id, Centroid>
    std::unordered_map<int, Centroid*> group_centroids_list_;
    // <Group_id, Vertices of the group> non overlapping sets
    std::unordered_map<int, std::vector<Vertex*>> final_groups_;
    // <vertex_id, group_id vertex_id is assigned to>
    std::unordered_map<int, int> vertex_to_group_;

    // Other useful parameters
    kdtree* kd_tree_centroids_; // kdtree for centroid search
    int centroid_id_count_ = -1;
    int group_id = -1;

    int merge_threshold = 1;

    // Melo
    RobotParams robot_params_;
    int robot_id_;
  };
}
