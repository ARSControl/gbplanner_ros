#include "graph_merging/merging.h"

#include <iostream>
#include <fstream>
#include <limits>

merge::GraphMerger::GraphMerger(const ros::NodeHandle& nh,
                    const ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private) {

    initializeAttributes();
}

void merge::GraphMerger::initializeAttributes(){
    global_graph_manager_.reset(new GraphManager());
    new_graph_.reset(new GraphManager());
    kd_tree_centroids_ = NULL;

    // Publishers
    merged_graph_pub_ = nh_.advertise<planner_msgs::Graph>("merging_node/merged_graph", 10);
    // gmm_pub_ = nh_.advertise<planner_msgs::Merge>("gmm_node/evaluate_gmm", 10);
    // kl_div_pub_ = nh_.advertise<planner_msgs::Graph>("gmm_node/pdf_similarity", 10);

    // Subscribers
    // global_graph_subscriber_ = nh_.subscribe("merging_node/fusion", 10, &GraphMerger::GraphCallback, this);
    gmm_sub_ = nh_.subscribe("gmm_node/vertices_to_keep", 10, &GraphMerger::filteredSubgraphCallback, this);


    stop_planner_client_ = nh_.serviceClient<planner_msgs::pci_stop>("pci_stop", 10);
    // test_ = nh_.subscribe("/ok", 10, &GraphMerger::loadGraphsCallback, this);

    // Melo
    loadParams();

    ROS_INFO("Robot nominal height: %f ", robot_params_.nominal_flight_height);
    ROS_INFO("Robot delta factor: %f ", robot_params_.delta_factor);

    int num_robots;
    nh_.getParam("/num_robots", num_robots);

    // Take the robot id from the namespace
    std::string ns = ros::this_node::getNamespace();
    if (!ns.empty() && ns[0] == '/'){
        ns = ns.substr(1);
    }
    size_t slash = ns.find('/');
    std::string robot_ns = ns.substr(0, slash);
    size_t underscore = robot_ns.find_last_of('_');
    robot_id_ = std::stoi(robot_ns.substr(underscore + 1));
}

void merge::GraphMerger::reset(){
    global_graph_manager_->reset();
    new_graph_->reset();
    global_graph_manager_->group_vertices_.clear();
    final_groups_.clear();
    vertex_to_group_.clear();
    this->cleanCentroids();

    centroid_id_count_ = -1;
    group_id = -1;

    //Create the kdtree for centroid search
    if (kd_tree_centroids_) kd_free(kd_tree_centroids_);
    kd_tree_centroids_ = kd_create(3);
}

// Melo
bool merge::GraphMerger::loadParams() {
    std::string ns = ros::this_node::getName();
    if (!robot_params_.loadParams(ns + "/RobotParams")) return false;
    return true;
}
// Melo

void merge::GraphMerger::cleanCentroids(){
    for(const auto& [id, centroid] : group_centroids_list_){
        delete centroid;
    }

    group_centroids_list_.clear();

    //Create the kdtree for centroid search
    if (kd_tree_centroids_) kd_free(kd_tree_centroids_);
    kd_tree_centroids_ = kd_create(3);
}

int merge::GraphMerger::generateCentroidID() { return ++centroid_id_count_; }
int merge::GraphMerger::generateGroupID() { return ++group_id; }

int merge::GraphMerger::getNumCentroids(){
    return group_centroids_list_.size();
}

bool merge::GraphMerger::getNearestCentroid(const StateVec* state, Centroid** c_res) {
    if (this->getNumCentroids() <= 0) return false;
    kdres* nearest = kd_nearest3(kd_tree_centroids_, state->x(), state->y(), state->z());
    if (kd_res_size(nearest) <= 0) {
        kd_res_free(nearest);
        return false;
    }
    *c_res = (Centroid*)kd_res_item_data(nearest);
    kd_res_free(nearest);
    return true;
}

double merge::GraphMerger::distance(const Vertex* v, const Centroid* centroid){
    double dx = v->state.x() - centroid->centroid_x;
    double dy = v->state.y() - centroid->centroid_y;
    double dz = v->state.z() - centroid->centroid_z;

    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

void merge::GraphMerger::saveGraphInFile(const std::string& filename, const std::shared_ptr<GraphManager>& graph){
    // Create and open the file in append mode
    std::ofstream file(filename, std::ios::app);

    if (!file.is_open()) {
        std::cerr << "Failed to open file!" << std::endl;
        return;
    }

    int num_vertices = graph->getNumVertices();
    for(int id=0; id < num_vertices; ++id){
        Vertex* vertex = graph->getVertex(id);
        if(vertex->type == VertexType::kFrontier){
            file << "ID: " << id << " State: " << vertex->state.x() << " " << vertex->state.y() << " " << vertex->state.z() << " True" << "\n";
        }
        else{
        file << "ID: " << id << " State: " << vertex->state.x() << " " << vertex->state.y() << " " << vertex->state.z() << " False" << "\n";
        }
        for(const auto& pair : graph->edge_map_[id]){
            file << "ID connected: " << pair.first << " Cost: " << pair.second << " \n";
        }
    }
    file.close();
}

void merge::GraphMerger::loadGraphsCallback(const std_msgs::Bool::ConstPtr& msg){
    this->reset();

    planner_msgs::Graph graph_msg1;
    planner_msgs::Graph graph_msg2;

    std::string path_1 = "./graph_1.txt";
    std::string path_2 = "./graph_2.txt";

    // Load and publish the first graph
    global_graph_manager_->loadGraph("./graph_1.bin");
    global_graph_manager_->convertGraphToMsg(graph_msg1);
    // this->saveGraphInFile(path_1, global_graph_manager_);
    // gmm_pub_.publish(graph_msg1);
    // kl_div_pub_.publish(graph_msg1);

    // Load and publish the second graph
    new_graph_->loadGraph("./graph_2.bin");
    new_graph_->convertGraphToMsg(graph_msg2);
    // this->saveGraphInFile(path_2, new_graph_);
    // gmm_pub_.publish(graph_msg2);
    // kl_div_pub_.publish(graph_msg2);

    std::unordered_set<int> vertex_ids;
    std::unordered_map<int,int> mapping_old_new_id;

    // Add all the vertices filtered to the owned graph
    for (const auto& v : graph_msg2.vertices){
        vertex_ids.insert(v.id);

        Vertex* vertex_ptr = new_graph_->getVertex(v.id);
        StateVec state = vertex_ptr->state;

        int new_id = global_graph_manager_->generateVertexID();
        Vertex* new_vertex = new Vertex(new_id, state);

        // Copy all properties
        new_vertex->vol_gain.num_unknown_voxels = vertex_ptr->vol_gain.num_unknown_voxels;
        new_vertex->vol_gain.num_occupied_voxels = vertex_ptr->vol_gain.num_occupied_voxels;
        new_vertex->vol_gain.num_free_voxels = vertex_ptr->vol_gain.num_free_voxels;
        new_vertex->vol_gain.is_frontier = vertex_ptr->vol_gain.is_frontier;
        new_vertex->type = vertex_ptr->type;

        global_graph_manager_->addVertex(new_vertex);
        mapping_old_new_id[v.id] = new_id;
    }

    // Re-build all edges using the new IDs
    for (const auto& e : graph_msg2.edges) {
        if (vertex_ids.count(e.source_id) && vertex_ids.count(e.target_id)) {
            Vertex* source_vertex = global_graph_manager_->getVertex(mapping_old_new_id[e.source_id]);
            Vertex* target_vertex = global_graph_manager_->getVertex(mapping_old_new_id[e.target_id]);
            global_graph_manager_->addEdgeMerging(source_vertex, target_vertex, e.weight);
        }
    }

    // this->saveGraphInFile("./sum_g1g2.txt", global_graph_manager_);
}

void merge::GraphMerger::graphMergeFromFile(){
    ROS_INFO(("Starting graph merging process. Initial number of vertices: %d"), global_graph_manager_->getNumVertices());
    // Clear the necessary data and data structure

    ROS_INFO("Start finding overlapping groups...");
    // Iteration over all the graph vertices for building initial groups with possible overlap
    for(auto& [vertex_id, vertex_properties] : global_graph_manager_->vertices_map_){
        // Find all the nearest vertices within the specified range
        StateVec state;
        state.x() = vertex_properties->state.x();
        state.y() = vertex_properties->state.y();
        state.z() = vertex_properties->state.z();
        std::vector<Vertex*> vertices;
        // This function has been modified to avoid duplicate vertices in the same group
        global_graph_manager_->getNearestVertices_modified(&state, 0.5, &vertices);

        int group_id = generateGroupID();

        if(vertices.size() > 1){
            // Assign the vertices to a group, possible overlaps may exist
            global_graph_manager_->group_vertices_[group_id] = vertices;
        }
        else {
            // If the group contains only the vertex itself, add it to the final groups list as a single group
            Vertex* vertex = global_graph_manager_->getVertex(vertex_id);
            final_groups_[group_id] = {vertex};

            // Keep track to which groups vertices are assigned definitely
            vertex_to_group_[vertex_id] = group_id;
        }

        for (auto& v : vertices){
            vertex_to_candidate_groups_[v->id].push_back(group_id);
        }
    }

    // Compute centroids and find the closes centroid for each vertex of the graph
    this->computeCentroids(global_graph_manager_->group_vertices_, group_centroids_list_);
    this->findClosestGroup(global_graph_manager_->group_vertices_, group_centroids_list_);

    // Kdtree is not cleaned because it is not needed at this step, but may be done inside the function
    this->computeCentroids(final_groups_, group_centroids_list_);

    // // Core merging process
    this->mergeWithRepresentativeNodes(final_groups_);
}

void merge::GraphMerger::computeCentroids(std::unordered_map<int, std::vector<Vertex*>>& group_vertices_, 
                                            std::unordered_map<int, Centroid*>& group_centroids_list_){
    this->cleanCentroids();

    // Iterate over groups with more than one vertex
    for (const auto& [group_id, vertices] : group_vertices_){
        double cx = 0, cy = 0, cz= 0;
        for (auto* v : vertices){
            cx += v->state.x();
            cy += v->state.y();
            cz += v->state.z();
        }

        if (vertices.size() > 1) {
            // Define centroid properties
            double centroid_x = cx/vertices.size();
            double centroid_y = cy/vertices.size();
            double centroid_z = cz/vertices.size();
            int centroid_id = group_id;

            // Create the centroid and add it to the list of centroids
            Centroid* centroid = new Centroid(centroid_id, centroid_x, centroid_y, centroid_z);
            group_centroids_list_[centroid_id] = centroid;

            // Add the centroid to the kdtree for centroid search
            kd_insert3(kd_tree_centroids_, centroid->centroid_x, centroid->centroid_y, centroid->centroid_z, centroid);
        }
    }
}

void merge::GraphMerger::findClosestGroup(std::unordered_map<int, std::vector<Vertex*>>& group_vertices_,
                                            std::unordered_map<int, Centroid*>& group_centroids_list_){
    // Iterate over the graph vertices
    for (const auto& [vertex_id, vertex_properties] : global_graph_manager_->vertices_map_){
        // Skip the vertex if it is already assigned to a group (assigned to itself as a single group)
        if (vertex_to_group_.find(vertex_id) != vertex_to_group_.end()) continue;

        Vertex* v = global_graph_manager_->getVertex(vertex_id);
        StateVec state;
        state.x() = v->state.x();
        state.y() = v->state.y();
        state.z() = v->state.z();
        Centroid* closest_centroid = nullptr;

        int closest_group_id = -1;

        // Search the closest centroid, if found take the centroid id
        if (this->getNearestCentroid(&state, &closest_centroid)){
            closest_group_id = closest_centroid->centroid_id;     
        } else{
            ROS_WARN("Nearest centroid not found, node not assigned! Left in the old group!");
        }
        // Assign each vertex to a group in order to have disjoint sets of vertices
        if (closest_group_id != -1){
            final_groups_[closest_group_id].push_back(v);
            vertex_to_group_[v->id] = closest_group_id;
        }

        // Check if the assigned group is one of the original candidates (Debug)
        if(std::find(vertex_to_candidate_groups_[vertex_id].begin(),
                    vertex_to_candidate_groups_[vertex_id].end(),
                    closest_group_id) == vertex_to_candidate_groups_[vertex_id].end()){
            ROS_ERROR("Group assigned is not an original canididate!");
        }
    }
}

void merge::GraphMerger::filteredSubgraphCallback(const planner_msgs::Merge& msg){
    this->reset();

    // Own graph
    global_graph_manager_->convertMsgToGraph(msg.input_graph);

    // Received graph
    new_graph_->convertMsgToGraph(msg.neighbour_graph);

    std::unordered_set<int> vertex_ids;
    std::unordered_map<int,int> mapping_old_new_id;

    // Add all the vertices filtered to the owned graph
    for (const auto& v : msg.vertices_to_keep){
        vertex_ids.insert(v.id);

        Vertex* vertex_ptr = new_graph_->getVertex(v.id);
        StateVec state = vertex_ptr->state;

        // Melo
        state[2] = robot_params_.nominal_flight_height + robot_params_.delta_factor*robot_id_;
        // Melo

        int new_id = global_graph_manager_->generateVertexID();
        Vertex* new_vertex = new Vertex(new_id, state);

        // Copy all properties
        new_vertex->vol_gain.gain = vertex_ptr->vol_gain.gain;
        new_vertex->vol_gain.num_unknown_voxels = vertex_ptr->vol_gain.num_unknown_voxels;
        new_vertex->vol_gain.num_occupied_voxels = vertex_ptr->vol_gain.num_occupied_voxels;
        new_vertex->vol_gain.num_free_voxels = vertex_ptr->vol_gain.num_free_voxels;
        new_vertex->vol_gain.is_frontier = vertex_ptr->vol_gain.is_frontier;
        new_vertex->type = vertex_ptr->type;

        global_graph_manager_->addVertex(new_vertex);
        mapping_old_new_id[v.id] = new_id;
    }
    // Preserve the tree structure built by the planner
    // for(const auto& [vertex_id, vertex] : global_graph_manager_->vertices_map_){
    //     std::unordered_set<int> seen;
    //     std::vector<Vertex*> new_children_list;

    //     if(vertex->parent){
    //         int old_parent_id = vertex->parent->id;
    //         vertex->parent = global_graph_manager_->getVertex(mapping_old_new_id[old_parent_id]);

    //         for(const auto& children : vertex->children){
    //             int old_children_id = children->id;
    //             if(seen.insert(old_children_id).second){
    //                 new_children_list.push_back(global_graph_manager_->getVertex(mapping_old_new_id[old_children_id]));
    //             }
    //         }
    //         vertex->children = std::move(new_children_list);
    //     }
        
    // }

    // Re-build all edges using the new IDs
    for (const auto& e : msg.neighbour_graph.edges) {
        if (vertex_ids.count(e.source_id) && vertex_ids.count(e.target_id)) {
            Vertex* source_vertex = global_graph_manager_->getVertex(mapping_old_new_id[e.source_id]);
            Vertex* target_vertex = global_graph_manager_->getVertex(mapping_old_new_id[e.target_id]);
            global_graph_manager_->addEdgeMerging(source_vertex, target_vertex, e.weight);
        }
    }

    // Perform the merging process
    this->mergingProcess();
}

void merge::GraphMerger::mergingProcess(){
    ROS_INFO(("Starting graph merging process. Initial number of vertices: %d"), global_graph_manager_->getNumVertices());

    // Iteration over all the graph vertices for building initial groups with possible overlap
    for(auto& [vertex_id, vertex_properties] : global_graph_manager_->vertices_map_){
        // Find all the nearest vertices within the specified range
        StateVec state;
        state.x() = vertex_properties->state.x();
        state.y() = vertex_properties->state.y();
        state.z() = vertex_properties->state.z();
        std::vector<Vertex*> vertices;
        // This function has been modified to avoid duplicate vertices in the same group
        global_graph_manager_->getNearestVertices_modified(&state, merge_threshold, &vertices);

        int group_id = generateGroupID();
        if(vertices.size() > 1){
            // Assign the vertices to a group, possible overlaps may exist
            global_graph_manager_->group_vertices_[group_id] = vertices;
        }
        else {
            // If the group contains only the vertex itself, add it to the final groups list as a single group
            Vertex* vertex = global_graph_manager_->getVertex(vertex_id);
            final_groups_[group_id] = {vertex};

            // Keep track to which groups vertices are assigned definitely
            vertex_to_group_[vertex_id] = group_id;
        }

        // For debug purposes
        for (auto& v : vertices){
            vertex_to_candidate_groups_[v->id].push_back(group_id);
        }
    }

    // Compute centroids and find the closes centroid for each vertex of the graph
    this->computeCentroids(global_graph_manager_->group_vertices_, group_centroids_list_);
    this->findClosestGroup(global_graph_manager_->group_vertices_, group_centroids_list_);

    // Re-compute centroids
    this->computeCentroids(final_groups_, group_centroids_list_);

    // Core merging process
    this->mergeWithRepresentativeNodes(final_groups_);
}

void merge::GraphMerger::mergeWithRepresentativeNodes(std::unordered_map<int, std::vector<Vertex*>>& final_groups_){

    new_graph_->reset();
    new_graph_->generateVertexID(); // To start the count from 0 avoiding root to be assigned to another group

    int init_num = global_graph_manager_->getNumVertices();

    std::unordered_map<int, Vertex*> old_to_new_vertex;

    //Iterating over disjoint final groups
    for (const auto& [group_id, vertices] : final_groups_){

        Vertex* representative_vertex = nullptr;
        Vertex* merged_vertex = nullptr;
        bool root_involved = false;
        bool all_frontiers = true;

        // Set the intial distance between vertices and centroid to infinite
        double min_distance = std::numeric_limits<double>::infinity();
        if (vertices.size() > 1){
            // Find the closest vertex to the centroid for the current group
            for (auto v: vertices){
                if (v->id == 0){
                    root_involved = true;
                }
                // Further investigate to understand if may be okay
                if(!(v->type == VertexType::kFrontier)){
                    all_frontiers = false;
                }
                double current_distance = this->distance(v, group_centroids_list_[group_id]);
                if (current_distance < min_distance){
                    min_distance = current_distance;
                    representative_vertex = v;
                }
            }
            // Manage the group containing the root vertex, to preserve the existence of the root
            if(root_involved){
                merged_vertex = new Vertex(0, representative_vertex->state);
            } else{
                merged_vertex = new Vertex(new_graph_->generateVertexID(), representative_vertex->state);
            }
            // Copy all properties
            merged_vertex->vol_gain.gain = representative_vertex->vol_gain.gain;
            merged_vertex->vol_gain.num_unknown_voxels = representative_vertex->vol_gain.num_unknown_voxels;
            merged_vertex->vol_gain.num_occupied_voxels = representative_vertex->vol_gain.num_occupied_voxels;
            merged_vertex->vol_gain.num_free_voxels = representative_vertex->vol_gain.num_free_voxels;
            // merged_vertex->vol_gain.is_frontier = representative_vertex->vol_gain.is_frontier;
            merged_vertex->vol_gain.is_frontier = all_frontiers;
            merged_vertex->type = representative_vertex->type;
            new_graph_->addVertex(merged_vertex);

            // Keep track of which nodes are representd by vertex v
            for (auto v: vertices){
                old_to_new_vertex[v->id] = merged_vertex;
            }
        } else {
            // Check if the vertex is the root vertex, to preserve the existence of the root
            if(vertices[0]->id == 0){
                merged_vertex = new Vertex(0, vertices[0]->state);
            } else{
                merged_vertex = new Vertex(new_graph_->generateVertexID(), vertices[0]->state);
            }
            // Copy all properties
            merged_vertex->vol_gain.gain = vertices[0]->vol_gain.gain;
            merged_vertex->vol_gain.num_unknown_voxels = vertices[0]->vol_gain.num_unknown_voxels;
            merged_vertex->vol_gain.num_occupied_voxels = vertices[0]->vol_gain.num_occupied_voxels;
            merged_vertex->vol_gain.num_free_voxels = vertices[0]->vol_gain.num_free_voxels;
            merged_vertex->vol_gain.is_frontier = vertices[0]->vol_gain.is_frontier;
            merged_vertex->type = vertices[0]->type;

            old_to_new_vertex[vertices[0]->id] = merged_vertex;
            new_graph_->addVertex(merged_vertex);
        }
    }

    // Preserve the tree structure built by the planner
    // for(const auto& [vertex_id, vertex] : new_graph_->vertices_map_){
    //     std::unordered_set<int> seen;
    //     std::vector<Vertex*> new_children_list;

    //     if(vertex->parent){
    //         int old_parent_id = vertex->parent->id;
    //         vertex->parent = old_to_new_vertex[old_parent_id];

    //         for(const auto& children : vertex->children){
    //             int old_children_id = children->id;
    //             if(seen.insert(old_children_id).second){
    //                 new_children_list.push_back(old_to_new_vertex[old_children_id]);
    //             }
    //         }

    //         vertex->children = std::move(new_children_list);
    //     }
    // }

    // Iterate over all the old vertices and get the old neighbours they are connected with 
    // to maintain connectivity in the new graph
    for (const auto& [old_id, old_vertex] : global_graph_manager_->vertices_map_) {
        std::vector<int> neighbours;
        global_graph_manager_->getAdjacentVertices(old_vertex, neighbours);
        for (int neigh_id : neighbours) {
            Vertex* new_from = old_to_new_vertex[old_id];
            Vertex* new_to   = old_to_new_vertex[neigh_id];

            // Avoid self-loops (same merged group)
            if (new_from->id == new_to->id){
                continue;
            }

            StateVec new_to_state = new_to->state;
            Eigen::Vector3d origin(new_from->state[0], new_from->state[1],
                        new_from->state[2]);
            Eigen::Vector3d direction(new_to_state[0] - origin[0], new_to_state[1] - origin[1],
                                        new_to_state[2] - origin[2]);
            double direction_norm = direction.norm();

            // Duplicate edges are avoided within the function
            new_graph_->addEdgeMerging(new_from, new_to, direction_norm);
        }
    }

    int final_num_vertices = new_graph_->getNumVertices();
    ROS_INFO("Graph merging process completed. Final number of vertices: %d", final_num_vertices);
    ROS_INFO("Removed vertices: %d", init_num - final_num_vertices);

    // Publish the merged graph
    planner_msgs::Graph merged_graph_msg;
    new_graph_->convertGraphToMsg(merged_graph_msg);
    ROS_INFO("Publishing merged graph msg with %zu nodes and %zu edges", merged_graph_msg.vertices.size(), merged_graph_msg.edges.size());
    merged_graph_pub_.publish(merged_graph_msg);
}