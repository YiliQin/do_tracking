#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/impl/io.hpp>

#include <cpd/version.hpp>
#include <cpd/nonrigid.hpp>

#include <iostream>
#include <chrono>
#include <string>

ros::Publisher pub;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
{
static int cb_count = 0;
cb_count++;
if (cb_count == 1)
{
cb_count = 0;
	static int downShow = 0;

	sensor_msgs::PointCloud2 output;

	// Convert pcl::PCLPointCloud2 to pcl::PointCloud<T>
	pcl::PointCloud<pcl::PointXYZRGB>* cloud = new pcl::PointCloud<pcl::PointXYZRGB>;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptrCloud(cloud);
	pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cptrCloud(ptrCloud);
	pcl::fromROSMsg(*input, *cloud);

	// Voxel filtering
	auto tic_voxel = std::chrono::high_resolution_clock::now();
	pcl::PointCloud<pcl::PointXYZRGB>* voxelFiltered = new pcl::PointCloud<pcl::PointXYZRGB>;
	pcl::VoxelGrid<pcl::PointXYZRGB> ds;
	ds.setInputCloud(cptrCloud);
	// For line
	//ds.setLeafSize(0.01, 0.01, 0.01);
	// For plane
	ds.setLeafSize(0.02, 0.02, 0.02);
	ds.filter(*voxelFiltered);

	auto toc_voxel = std::chrono::high_resolution_clock::now();
	std::chrono::microseconds dur_ms;
	std::chrono::duration<double, std::milli> dur_voxel_ms = toc_voxel - tic_voxel; 
	std::cout << "Voxel filtering duration(ms) >>> " << dur_voxel_ms.count() << std::endl;

	// Color segmentation
	auto tic_seg = std::chrono::high_resolution_clock::now();
	pcl::PointCloud<pcl::PointXYZRGB>* colorFiltered = new pcl::PointCloud<pcl::PointXYZRGB>;
	
	int count = 0;
	for (size_t i = 0; i < voxelFiltered->size(); i++)
	{
		// For line
		//if (voxelFiltered->points[i].r < 80 && voxelFiltered->points[i].g < 80 && voxelFiltered->points[i].b > 120)
		// For plane
		if (voxelFiltered->points[i].r < 40 && voxelFiltered->points[i].g < 40 && voxelFiltered->points[i].b > 60)
		{
			colorFiltered->push_back(voxelFiltered->points[i]);
			count ++;
		}
	}
	std::cout << "Numbers of point cloud of the calbe >>> " << count << std::endl;

	auto toc_seg = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double, std::milli> dur_seg_ms = toc_seg - tic_seg; 
	std::cout << "Color segmentation duration(ms) >>> " << dur_seg_ms.count() << std::endl;

	// pcl::PointXYZRGB -> Eigen::MatrixXd
	size_t nrows = colorFiltered->size();
	size_t ncols = 3;
	std::cout << "Fixed point cloud size (dataFixed) >>> " << nrows << std::endl;
	Eigen::MatrixXd dataFixed(nrows, ncols);
	for (size_t i = 0; i < nrows; i++)
	{
		dataFixed(i, 0) = (double)colorFiltered->points[i].x;
		dataFixed(i, 1) = (double)colorFiltered->points[i].y;
		dataFixed(i, 2) = (double)colorFiltered->points[i].z;
	}

	auto tic_cpd = std::chrono::high_resolution_clock::now();
	// Generate the point set
	static Eigen::MatrixXd gen_pointset(15, 3);
	static int loop_cpd = 0;
	loop_cpd++;
	std::cout << "Current loop of CPD >>> " << loop_cpd << std::endl;
	if (loop_cpd == 1)
	{
		std::cout << "Initial the loop_cpd ... " << std::endl;
		//// Genreate a line
		//for (size_t i = 0; i < gen_pointset.rows(); i++)
		//{
			//double u = (double)(i);
			//gen_pointset(i, 0) = (u / 10);
			//gen_pointset(i, 1) = (u / 10);
			//gen_pointset(i, 2) = (u / 10);
		//}

		// Genreate a plane
		int index = 0;
		for (size_t i = 0; i < gen_pointset.rows()/3; i++)
		{
			gen_pointset(index + i, 0) = ((double)(i))/2;
			//gen_pointset(index + i, 1) = ((double)(i))/3;
			//gen_pointset(index + i, 2) = ((double)(i))/3;
			gen_pointset(index + i, 1) = 0.5;
			gen_pointset(index + i, 2) = 0.0;
		}
		index = gen_pointset.rows()/3;
		for (size_t i = 0; i < gen_pointset.rows()/3; i++)
		{
			//gen_pointset(index + i, 0) = ((double)(i))/3 + 0.5;
			gen_pointset(index + i, 0) = ((double)(i))/2;
			//gen_pointset(index + i, 1) = ((double)(i))/3;
			//gen_pointset(index + i, 2) = ((double)(i))/3;
			gen_pointset(index + i, 1) = 0.0;
			gen_pointset(index + i, 2) = 0.0;
		}
		index = 2*gen_pointset.rows()/3;
		for (size_t i = 0; i < gen_pointset.rows()/3; i++)
		{
			//gen_pointset(index + i, 0) = ((double)(i))/3 - 0.5;
			gen_pointset(index + i, 0) = ((double)(i))/2;
			//gen_pointset(index + i, 1) = ((double)(i))/3;
			//gen_pointset(index + i, 2) = ((double)(i))/3;
			gen_pointset(index + i, 1) = -0.5;
			gen_pointset(index + i, 2) = 0.0;
		}
	}
	else 
	{
		cpd::Nonrigid nonrigid;
		nonrigid.correspondence("true");
		nonrigid.outliers(0.1);
		nonrigid.tolerance(1e-5);
		cpd::NonrigidResult result = nonrigid.run(dataFixed, gen_pointset);
		gen_pointset = result.points;
	}
	auto toc_cpd = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double, std::milli> dur_cpd_ms = toc_cpd - tic_cpd;
	std::cout << "CPD duration(ms) >>> " << dur_cpd_ms.count() << std::endl;
	// Translate Eigen::MatrixXf to PointXYZRGB
	pcl::PointCloud<pcl::PointXYZRGB>* pointsDisplay = new (pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointXYZRGB p;

	for (size_t i = 0; i < colorFiltered->size(); i++)
	{
		p.x = colorFiltered->points[i].x;
		p.y = colorFiltered->points[i].y;
		p.z = colorFiltered->points[i].z;
		p.r = 0;
		p.g = 0;
		p.b = 255;

		pointsDisplay->push_back(p);
	}

	for (size_t i = 0; i < gen_pointset.rows(); i++)
	{
		// Generate the point cloud
		p.x = gen_pointset(i, 0);
		p.y = gen_pointset(i, 1);
		p.z = gen_pointset(i, 2);

		// Generate the point cloud
		p.r = 255;
		p.g = 0;
		p.b = 0;

		// Push the generated point
		pointsDisplay->push_back(p);
	}

	//static bool getPCL = false;
	//static int getPCLCount = 0;
	//getPCLCount++;
	//if (getPCL == false && getPCLCount == 100)
	//{
		//getPCL = true;
		//pcl::PointCloud<pcl::PointXYZ> cloud1;
		//std::cout << "Copy data finish ..." << std::endl;
		//pcl::copyPointCloud(*colorFiltered, cloud1);
		//std::cout << "Start to writing .PCD >>> " << std::endl;
		//pcl::io::savePCDFileASCII("/home/yili/Documents/workspace/cpd/pcl_matching/data/biwi_face_database/cable.pcd", cloud1);
	//}

	// Convert pcl::PCLPointCloud2 to pcl::PointCloud<T>
	// Output the color-based filtered
	//pcl::toROSMsg(*colorFiltered, output);
	// Output the down-sampling result
	//pcl::toROSMsg(*voxelFiltered, output);
	// Output the registration result
	pcl::toROSMsg(*pointsDisplay, output);
	output.header.stamp = ros::Time::now();
	output.header.frame_id = "camera_rgb_optical_frame";
	pub.publish(output);
}
}

int main(int argc, char** argv)
{
	// Successfully run
	std::cout << "Successfully run!" << std::endl;

	//pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");

	// Initial ROS
	ros::init(argc, argv, "do_tracking");
	ros::NodeHandle nh;

	// Create a ROS subscriber for the input depth data
	ros::Subscriber sub = nh.subscribe("input", 1, cloud_cb);
	
	// Create a ROS publisher
	pub = nh.advertise<sensor_msgs::PointCloud2>("output", 1);

	std::cout << "Version of Library CPD: " << cpd::version() << std::endl; 
	std::cout << "This is before the ros::spin() function" << std::endl;

	// Spin
	ros::spin();

	std::cout << "This is after the ros::spin() function" << std::endl;
	
	return 0;
}

