#include <ros/ros.h>

#include "graph_merging/merging.h"

int main(int argc, char** argv) {

  ros::init(argc, argv, "graph_merging_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  merge::GraphMerger merger(nh, nh_private);

  ros::spin();

  return 0;
}
