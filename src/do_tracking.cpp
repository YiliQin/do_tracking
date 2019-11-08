#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/pcl_config.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_types_conversion.h>
#include <pcl_conversions/pcl_conversions.h>
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

#define FRAME_BASE 30
#define DESTINATION_RATE 15
#define OUTPUT_TIME_INFO true
#define OUTPUT_DEBUG_INFO false

/* 1 - Matching result
 * 2 - Voxel filtering result
 * 3 - Color filtering result
 *
 */
#define DISPLAY_SELECT 1

/* 1 - HSV filter
 * 2 - RGB filter
 *
 */
#define COLOR_FILTER_SEL 1

ros::Publisher pub;
int cb_count = 0;

/*  Voxel filter.
 *
 */
pcl::PointCloud<pcl::PointXYZRGB> * voxel_filter(pcl::PointCloud<pcl::PointXYZRGB> & cloud)
{
	pcl::VoxelGrid<pcl::PointXYZRGB> vg;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptrTmp(&cloud);
	pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cptrCloud(ptrTmp);
	pcl::PointCloud<pcl::PointXYZRGB> * ptrCloudVoxel = new (pcl::PointCloud<pcl::PointXYZRGB>);

	auto tic_voxel = std::chrono::high_resolution_clock::now();

	vg.setInputCloud(cptrCloud);
	// For cable
	//ds.setLeafSize(0.01, 0.01, 0.01);
	// For paper
	vg.setLeafSize(0.02, 0.02, 0.02);
	// For nappe
	vg.setLeafSize(0.02, 0.02, 0.02);
	vg.filter(*ptrCloudVoxel);

	auto toc_voxel = std::chrono::high_resolution_clock::now();
	std::chrono::microseconds dur_ms;
	std::chrono::duration<double, std::milli> dur_voxel_ms = toc_voxel - tic_voxel; 
	if (OUTPUT_TIME_INFO == true)
			std::cout << "Voxel filtering duration(ms) >>> " << dur_voxel_ms.count() << std::endl;

	return ptrCloudVoxel;
}

/*  HSV - Color based filter.
 *
 */
pcl::PointCloud<pcl::PointXYZRGB> * colorHSV_filter(pcl::PointCloud<pcl::PointXYZRGB> & cloud)
{
	pcl::PointCloud<pcl::PointXYZHSV> * ptrCloudHSV = new (pcl::PointCloud<pcl::PointXYZHSV>); 

	auto tic_hsv = std::chrono::high_resolution_clock::now();

	for (size_t i = 0; i < cloud.size(); i++)
	{
		pcl::PointXYZHSV p;
		pcl::PointXYZRGBtoXYZHSV(cloud.points[i], p);	
		p.x = cloud.points[i].x;
		p.y = cloud.points[i].y;
		p.z = cloud.points[i].z;
		if (OUTPUT_DEBUG_INFO == true)
			std::cout << "HSV: h=" << p.h << "; s=" << p.s << "; v=" << p.v << std::endl;
		if ((0 <= p.h && p.h <= 10 || (250 <= p.h && p.h <= 340)) &&
					(0.3 <= p.s && p.s <= 0.8) &&
						(0.35 <= p.v && p.v <= 0.85))
		{
			ptrCloudHSV->push_back(p);
		}
	}

	pcl::PointCloud<pcl::PointXYZRGB> * ptrCloudRGB = new (pcl::PointCloud<pcl::PointXYZRGB>);
		for (size_t i = 0; i < ptrCloudHSV->size(); i++)
		{
			pcl::PointXYZRGB p;
			pcl::PointXYZHSVtoXYZRGB(ptrCloudHSV->points[i], p);	
			p.x = ptrCloudHSV->points[i].x;
			p.y = ptrCloudHSV->points[i].y;
			p.z = ptrCloudHSV->points[i].z;
			ptrCloudRGB->push_back(p);
		}

	auto toc_hsv = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double, std::milli> dur_hsv_ms = toc_hsv - tic_hsv; 
	if (OUTPUT_TIME_INFO == true)
		std::cout << "HSV segmentation duration(ms) >>> " << dur_hsv_ms.count() << std::endl;
	
	return ptrCloudRGB;
}	

/*  RGB - Color based filter.
 *
 */
pcl::PointCloud<pcl::PointXYZRGB> * colorRGB_filter(pcl::PointCloud<pcl::PointXYZRGB> & cloud)
{
	pcl::PointCloud<pcl::PointXYZRGB> * ptrColorFilter = new pcl::PointCloud<pcl::PointXYZRGB>;
	int count = 0;

	auto tic_seg = std::chrono::high_resolution_clock::now();

	for (size_t i = 0; i < cloud.size(); i++)
	{
		// For cable
		//if cloud.points[i].r < 80 && cloud.points[i].g < 80 && cloud.points[i].b > 120)
		// For paper
		//if (cloud.points[i].r < 40 && cloud.points[i].g < 40 && cloud.points[i].b > 60)
		// For nappe
		if ((100 <= cloud.points[i].r && cloud.points[i].r <= 255) &&
					(0 <= cloud.points[i].g && cloud.points[i].g <= 100) && 
						(0 <= cloud.points[i].b && cloud.points[i].b <= 100))
		{
			ptrColorFilter->push_back(cloud.points[i]);
			count++;
		}
	}
	if (OUTPUT_DEBUG_INFO == true)
		std::cout << "RGB color filter: Numbers of points >>> " << count << std::endl;

	auto toc_seg = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double, std::milli> dur_seg_ms = toc_seg - tic_seg; 
	if (OUTPUT_TIME_INFO == true)
		std::cout << "RGB segmentation duration(ms) >>> " << dur_seg_ms.count() << std::endl;

	return ptrColorFilter;
}

/* 	CPD matching.
 *
 */
pcl::PointCloud<pcl::PointXYZRGB> * cpd_matching(pcl::PointCloud<pcl::PointXYZRGB> & cloud)
{
	// pcl::PointXYZRGB -> Eigen::MatrixXd
	size_t nrows = cloud.size();
	size_t ncols = 3;
	if (OUTPUT_DEBUG_INFO == true)
		std::cout << "Fixed point cloud size (dataFixed) >>> " << nrows << std::endl;
	Eigen::MatrixXd dataFixed(nrows, ncols);
	for (size_t i = 0; i < nrows; i++)
	{
		dataFixed(i, 0) = (double)cloud.points[i].x;
		dataFixed(i, 1) = (double)cloud.points[i].y;
		dataFixed(i, 2) = (double)cloud.points[i].z;
	}

	// CPD registration
	auto tic_cpd = std::chrono::high_resolution_clock::now();
	// Generate the point set
	static Eigen::MatrixXd gen_pointset(15, 3);
	static int loop_cpd = 0;
	loop_cpd++;
	if (OUTPUT_DEBUG_INFO == true)
		std::cout << "CPD looping ... " << loop_cpd << std::endl;
	if (loop_cpd == 1)
	{
		std::cout << "Initialze CPD ... " << std::endl;
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
		index = 2 * gen_pointset.rows() / 3;
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
	if (OUTPUT_TIME_INFO == true)
		std::cout << "CPD duration(ms) >>> " << dur_cpd_ms.count() << std::endl;

	// Convert Eigen::MatrixXf to PointXYZRGB
	pcl::PointCloud<pcl::PointXYZRGB> * pointsDisplay = new (pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointXYZRGB p;

	for (size_t i = 0; i < cloud.size(); i++)
	{
		p.x = cloud.points[i].x;
		p.y = cloud.points[i].y;
		p.z = cloud.points[i].z;
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

	return pointsDisplay;
}

/* Tracking the surface of the deformable object
 *
 */
void do_tracking(const sensor_msgs::PointCloud2ConstPtr & input)

{
	cb_count++;
	if (OUTPUT_DEBUG_INFO == true)
		std::cout << "cb_count = " << cb_count << std::endl;

	if (cb_count == int(FRAME_BASE)/int(DESTINATION_RATE))
	{
		cb_count = 0;

		// Convert pcl::PCLPointCloud2 to pcl::PointCloud<T>
		pcl::PointCloud<pcl::PointXYZRGB> * ptrCloudRGB = new pcl::PointCloud<pcl::PointXYZRGB>;
		pcl::fromROSMsg(*input, *ptrCloudRGB);

		// Voxel filter
		pcl::PointCloud<pcl::PointXYZRGB> * ptrVoxelFilter = new pcl::PointCloud<pcl::PointXYZRGB>;
		ptrVoxelFilter = voxel_filter(*ptrCloudRGB);

		// Color filter
		pcl::PointCloud<pcl::PointXYZRGB> * ptrColorFilter = new (pcl::PointCloud<pcl::PointXYZRGB>);
		switch (COLOR_FILTER_SEL)
		{
			case 1:	ptrColorFilter = colorHSV_filter(*ptrVoxelFilter); break;
			case 2: ptrColorFilter = colorRGB_filter(*ptrVoxelFilter); break;
			default: break;
		}
		
		// CPD matching
		pcl::PointCloud<pcl::PointXYZRGB> * ptrColorCPD= new (pcl::PointCloud<pcl::PointXYZRGB>);
		ptrColorCPD = cpd_matching(*ptrColorFilter);
		
		// Convert pcl::PCLPointCloud2 to pcl::PointCloud<T>
		pcl::PointCloud<pcl::PointXYZRGB> * displayCloud = new (pcl::PointCloud<pcl::PointXYZRGB>);
		sensor_msgs::PointCloud2 output;
		switch (DISPLAY_SELECT)
		{
			case 1: displayCloud = ptrColorCPD; break;
			case 2: displayCloud = ptrVoxelFilter; break;
			case 3: displayCloud = ptrColorFilter; break;
			default: break;
		}

		// pcl::toROSMsg(*pointsDisplay, output);
		pcl::toROSMsg(*displayCloud, output);
		output.header.stamp = ros::Time::now();
		output.header.frame_id = "camera_rgb_optical_frame";
		pub.publish(output);
	}
}

int main(int argc, char * argv[])
{
	// Print out system info
	std::cout << "PCL Version: " << PCL_VERSION << std::endl;
	std::cout << "CPD Version: " << cpd::version() << std::endl; 
	std::cout << "Start tracking ...  " << std::endl;

	// Initialize ROS
	ros::init(argc, argv, "do_tracking");
	ros::NodeHandle nh;

	// Create ROS subscriber & publisher 
	ros::Subscriber sub = nh.subscribe("input", 1, do_tracking);
	pub = nh.advertise<sensor_msgs::PointCloud2>("output", 1);

	// Spin
	ros::spin();
	
	return 0;
}
