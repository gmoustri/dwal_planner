/*
 * ff_cluster_processing.h
 *
 *  Created on: Nov 20, 2015
 *      Author: root
 */
#include <ros/ros.h>
#include <vector>
#include <math.h>
#include <algorithm>
#include <dwal_planner/Cluster_Group.h>

#ifndef DWAL_CLUSTER_PROCESSING_H_
#define DWAL_CLUSTER_PROCESSING_H_

class ProcessedCluster{
public:
	double bound_low, bound_hi;
	double mean_phi;	//mean phi of largest sub-cluster
	double cluster_mean;	//mean phi of cluster
	int id;
	double R;
};
class ProcessedGroup{
public:
	ros::Subscriber group_subscriber;
	std::vector<int> subCls_start, subCls_size;
	ProcessedCluster tmp_Cl;
	std::vector<ProcessedCluster> cluster_objects;
	ros::NodeHandle nh;
	double R_group;
	int cluster_num, cindex, cl_size, min_cost_Index, max_subCl_index, min_cost;
	bool group_empty,is_subCl_open;
	int subCls_start_Index,subCls_end_index;

	ProcessedGroup();
	void processGroup(const dwal_planner::Cluster_Group::ConstPtr& msg);
	ProcessedCluster* getClusterById(const int& id);
	ProcessedCluster* getClusterById(const int& id) const;
	double getGroupMaxBound();
	double getGroupMinBound();
};



#endif /* DWAL_CLUSTER_PROCESSING_H_ */
