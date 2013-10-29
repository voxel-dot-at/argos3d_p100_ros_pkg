/******************************************************************************
 * Copyright (c) 2013
 * VoXel Interaction Design
 *
 * Copyright (c) 2011
 * GPS GmbH
 *
 * @author Simon Vogl
 * @author Angel Merino Sastre
 * 
 * Pinaki Sunil Banerjee
 *
 *
 * This software is published under a dual-license: GNU Lesser General Public
 * License LGPL 2.1 and BSD license. The dual-license implies that users of this
 * code may choose which terms they prefer.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * * Neither the name of Locomotec nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 2.1 of the
 * License, or (at your option) any later version or the BSD license.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL and the BSD license for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL and BSD license along with this program.
 *
 ******************************************************************************/
#define SOURCE_PARAM ""
#define PROC_PARAM ""
#include <pmdsdk2.h>

#include <ros/console.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <ros/publisher.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <stdio.h>
#include <time.h>
#include <sstream>

#include <argos3d_p100/argos3d_p100Config.h>
#include <dynamic_reconfigure/server.h>

typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;

/**
 * Global Parameter Declarations
 */

/**
 * Camera Configuration Parameters
 */
int integrationTime;
int modulationFrequency;
bool bilateralFilter;

bool AtLeastFrequency;
bool AtMostFrequency;

bool StatisticalNoiseFilterOn;
int NoiseFilteringNoOfNeighbours;
float StdDevMulThreshold;

bool AmplitudeFilterOn;
float AmplitudeThreshold;

int noOfRows;
int noOfColumns;

bool first;
/**
 * Camera Driver Parameters
 */
PMDHandle hnd;
int res;
char err[128];

/**
 * ROS Parameters
 */

bool dataPublished;
ros::Publisher pub_non_filtered;
ros::Publisher pub_filtered;
ros::Publisher pub_outliers;

int help() {
	std::cout << "\n Using help for argos3d_p100_ros_pkg\n"
		" You can set defauld configuration values for the camera with the following options: \n" << std::endl;
	std::cout << " Usage:\n rosrun argos3d_p100 argos3d_p100_node "<< std::endl
		<< "\t-it *Integration_Time* \n\tIntegration time(in msec) for the sensor \n\t(min: 100 | max: 2700 | default: 1500) "<< std::endl
		<< "\t-mf  *Modulation_Frequency* \n\tSet the modulation frequency(Hz) of the sensor \n\t(min: 5000000 | max: 30000000 | default: 30000000) "<< std::endl
		<< "\t-bf *Bilateral_Filter* \n\tTurns bilateral filtering on or off \n\t(ON: if set | OFF: otherwise | default: OFF) "<< std::endl
		/*<< "\t-al *At_Least* \n\tModulation Frequency no less than the entered frequency \n\t(ON: 1 | OFF: 0 | default: OFF) "<< std::endl
		<< "\t-am *At_Most* \n\tModulation Frequency no more than the entered frequency \n\t(ON: 1 | OFF: 0 | default: OFF) "<< std::endl
		<< "\t-snf *Statistical_Noise_Filter_On* \n\tWhether to apply statistical noise filter from pcl or not \n\t(ON: 1 | OFF: 0 | default: OFF) "<< std::endl
		<< "\t-nfn *Noise_Filtering_NoOfNeighbours* \n\tNo. of neighbours to be considered for applying statistical noise reduction \n\t(min: 1 | max: 200 | default: 30) "<< std::endl
		<< "\t-sdt *Std_Dev_Mul_Threshold* \n\tStandard Deviation Multiplier Threshold for applying statistical noise reduction \n\t(min: 0.0 | max: 10.0 | default: 0.4) "<< std::endl*/
		<< "\t-af *Amplitude_Filter_On* \n\tWhether to apply amplitude filter or not. Image pixels with amplitude values less than the threshold will be filtered out \n\t(ON: if set | OFF: otherwisecatk | default: OFF) " << std::endl
		<< "\t-at *Amplitude_Threshold* \n\tWhat should be the amplitude filter threshold. Image pixels with lesser aplitude values will be filtered out. Amplitude Filter Status should be true to use this filter \n\t(min: 0 | max: 2500 | default: 0) "<< std::endl
		<< "\n Example:" << std::endl
		<< "rosrun argos3d_p100 argos3d_p100_node -it 1500 -mf 30000000 \n" << std::endl;
	return 0;
} //print_help

void callback(argos3d_p100::argos3d_p100Config &config, uint32_t level)
{
	// Check the configuretion parameters with those given in the initialization
	if(first) {
		config.Integration_Time = integrationTime;
		config.Modulation_Frequency = modulationFrequency;
		config.Bilateral_Filter = bilateralFilter;
		config.Amplitude_Filter_On = AmplitudeFilterOn;
		config.Amplitude_Threshold = AmplitudeThreshold;
		integrationTime = modulationFrequency = AmplitudeThreshold = -1;
		bilateralFilter = !bilateralFilter;
		AmplitudeFilterOn = !AmplitudeFilterOn;
	}

	if(integrationTime != config.Integration_Time) {
		integrationTime = config.Integration_Time;
		res = pmdSetIntegrationTime (hnd, 0, integrationTime);
		if (res != PMD_OK)
		{
			pmdGetLastError (hnd, err, 128);
			ROS_WARN_STREAM("Could not set integration time: " << err);
		}
	}

	if(modulationFrequency != config.Modulation_Frequency) {
		modulationFrequency = config.Modulation_Frequency;
		res = pmdSetModulationFrequency(hnd, 0, modulationFrequency);
		if (res != PMD_OK) {
			pmdGetLastError (hnd, err, 128);
			ROS_WARN_STREAM("Could not set modulation frequency: " << err);
		}
	}

	if(bilateralFilter != config.Bilateral_Filter) {
		bilateralFilter = config.Bilateral_Filter;
		err[0] = 0;
		if(bilateralFilter)
			res = pmdProcessingCommand(hnd, err, sizeof(err), "SetBilateralFilter on");
		else
			res = pmdProcessingCommand(hnd, err, sizeof(err), "SetBilateralFilter off");
		if (res != PMD_OK) {
			pmdGetLastError (hnd, err, 128);
			ROS_WARN_STREAM("Could not set bilateral filter: " << err);
		}
	}

	/*AtLeastFrequency = config.At_Least;
	AtMostFrequency = config.At_Most;

	StatisticalNoiseFilterOn = config.Statistical_Noise_Filter_On;
	NoiseFilteringNoOfNeighbours = config.Noise_Filtering_NoOfNeighbours;
	StdDevMulThreshold = (float)config.Std_Dev_Mul_Threshold;*/

	AmplitudeFilterOn = config.Amplitude_Filter_On;
	AmplitudeThreshold = config.Amplitude_Threshold;
}

/**
 * Initialize the camera and initial parameter values. Returns 1 if properly initialized.
 */
int initialize(int argc, char *argv[],ros::NodeHandle nh){
	/*
	 * Inital Setup for parameters
	 */
	integrationTime = 1500;
	modulationFrequency = 30000000;
	bilateralFilter = false;
	
	AtLeastFrequency = false;
	AtMostFrequency = false;

	StatisticalNoiseFilterOn = false;
	NoiseFilteringNoOfNeighbours = 30;
	StdDevMulThreshold = 0.4;

	AmplitudeFilterOn = false;
	AmplitudeThreshold = 0;
	
	for( int i = 1; i < argc; i++) {
		
		// reading width
		if( std::string(argv[i]) == "-it" ) {
			if( sscanf(argv[++i], "%d", &integrationTime) != 1 
				|| integrationTime < 100 || integrationTime > 2700 ) {
				ROS_WARN("*invalid integration time");
				return help();
			}
		}
		// reading heigth
		else if( std::string(argv[i]) == "-mf" ) {
			if( sscanf(argv[++i], "%d", &modulationFrequency) != 1 
				|| modulationFrequency < 5000000 || integrationTime > 30000000 ) {
				ROS_WARN("*invalid modulation frequency");
				return help();
			}
		}
		else if( std::string(argv[i]) == "-bf" ) {
			bilateralFilter = true;
		}
		// reading images file name
		/*else if( std::string(argv[i]) == "-al" ) {
			AtLeastFrequency = true;
		}
		// reading calibration paremeter for left camera
		else if( std::string(argv[i]) == "-am" ) {
			AtMostFrequency = true;
		}
		// reading calibration paremeter for right camera
		else if( std::string(argv[i]) == "-snf" ) {
			StatisticalNoiseFilterOn = true;
		}
		// corner distance
		else if( std::string(argv[i]) == "-nfn" ) {
			if( sscanf(argv[++i], "%d", &NoiseFilteringNoOfNeighbours) != 1 
				|| NoiseFilteringNoOfNeighbours < 1 || NoiseFilteringNoOfNeighbours > 200 ) {
				std::cout << "*invalid noise filtering No. of neighbours" << std::endl;
				return help();
			}
		}
		else if( std::string(argv[i]) == "-sdt" ) {
		   if( sscanf(argv[++i], "%f", &StdDevMulThreshold) != 1 
				|| StdDevMulThreshold < 1.0f || StdDevMulThreshold > 10.0f ) {
				std::cout << "*invalid standard deviation multiplier threshold" << std::endl;
				return help();
			}
		}*/
		// additional parameters
		else if( std::string(argv[i]) == "-af" ) {
			AmplitudeFilterOn = true;
		}
		else if( std::string(argv[i]) == "-at" ) {
			if( sscanf(argv[++i], "%f", &AmplitudeThreshold) != 1 
				|| AmplitudeThreshold < 0 || AmplitudeThreshold > 2500 ) {
				ROS_WARN("*invalid amplitude threshold");
				return help();
			}	
		}
		// print help
		else if( std::string(argv[i]) == "--help" ) {
			ROS_WARN_STREAM("arguments: " << argc << " which: " << argv[i]);		
			return help();
		}
		else if( argv[i][0] == '-' ) {
			ROS_WARN_STREAM("invalid option " << argv[i]);
			return help();
		}
	} 	

	/*
	 * Camera Initialization
	 */
	std::stringstream sourcePluginLocation, procPluginLocation;
	sourcePluginLocation.clear();
	sourcePluginLocation.clear();
	sourcePluginLocation << PMD_PLUGIN_DIR << "digicam";
	procPluginLocation << PMD_PLUGIN_DIR << "digicamproc";

	// If the camera is not connected at all, we will get an segmentation fault.
	res = pmdOpen (&hnd, sourcePluginLocation.str().c_str(), SOURCE_PARAM, procPluginLocation.str().c_str(), PROC_PARAM);
	if (res != PMD_OK)
	{
		pmdGetLastError (0, err, 128);
		ROS_ERROR_STREAM("Could not connect: " << err);
		return 0;
	}

	res = pmdUpdate (hnd);
	if (res != PMD_OK)
	{
		pmdGetLastError (hnd, err, 128);
		ROS_ERROR_STREAM("Could not transfer data: " << err);
		pmdClose (hnd);
		return 0;
	}


	PMDDataDescription dd;

	res = pmdGetSourceDataDescription (hnd, &dd);
	if (res != PMD_OK)
	{
		pmdGetLastError (hnd, err, 128);
		ROS_ERROR_STREAM("Could not get data description: " << err);
		pmdClose (hnd);
		return 0;
	}

	if (dd.subHeaderType != PMD_IMAGE_DATA)
	{
		ROS_ERROR_STREAM("Source data is not an image!\n");
		pmdClose (hnd);
		return 0;
	}
	noOfRows = dd.img.numRows;
	noOfColumns = dd.img.numColumns;

	/*
	 * ROS Node Initialization
	 */
	pub_non_filtered = nh.advertise<PointCloud> ("depth_non_filtered", 1);
	pub_filtered = nh.advertise<PointCloud> ("depth_filtered", 1);
	pub_outliers = nh.advertise<PointCloud> ("depth_outliers", 1);
	dataPublished=true;
	return 1;
}

/**
 * Publishes the point clod passed as a parameter
 */
void publishCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr , ros::Publisher pub){
	PointCloud::Ptr msg (new PointCloud);
	msg->header.frame_id = "tf_argos";
	msg->height = 1;
	msg->width = cloud_ptr->points.size();
	for  (unsigned int i =0; i< cloud_ptr->points.size() ; i++)
		msg->points.push_back ( pcl::PointXYZI(cloud_ptr->points[i]) );
#if ROS_VERSION > ROS_VERSION_COMBINED(1,9,49)
	msg->header.stamp = ros::Time::now().toNSec();
#else
	msg->header.stamp = ros::Time::now();
#endif
	pub.publish (msg);
}


/**
 * Publish the data based on set up parameters.
 */
int publishData() {

	/*
	 * Update Camera settings
	 */
	res = pmdUpdate (hnd);
	if (res != PMD_OK)
	{
		pmdGetLastError (hnd, err, 128);
		ROS_ERROR_STREAM("Could transfer data: " << err);
		pmdClose (hnd);
		return 0;
	}

	/*
	 * Obtain PointClouds
	 */
	float * cartesianDist = new float [noOfRows * noOfColumns * 3];
	res = pmdGet3DCoordinates (hnd, cartesianDist, noOfColumns * noOfRows * 3 * sizeof (float));
	if (res != PMD_OK)
	{
		pmdGetLastError (hnd, err, 128);
		ROS_ERROR_STREAM("Could get cartesian coordinates: " << err);
		pmdClose (hnd);
		return 0;
	}


	/*
	 * Obtain Amplitude Values
	 */
	float * amplitudes = new float [noOfRows * noOfColumns];

		res = pmdGetAmplitudes (hnd, amplitudes, noOfRows * noOfColumns * sizeof (float));
		if (res != PMD_OK)
		{
			pmdGetLastError (hnd, err, 128);
			ROS_ERROR_STREAM("Could get amplitude values: " << err);
			pmdClose (hnd);
			return 1;
		}

	/*
	 * Creating the pointcloud
	 */
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr_filtered (new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr_outliers (new pcl::PointCloud<pcl::PointXYZI>);

	// Fill in the cloud data
	cloud_ptr->width    = noOfColumns*noOfRows;
	cloud_ptr->height   = 1;
	cloud_ptr->is_dense = false;
	cloud_ptr->points.resize (cloud_ptr->width * cloud_ptr->height);

	int countWidth=0;

	if(AmplitudeFilterOn){


		for (size_t i = 0; i < cloud_ptr->points.size (); ++i)
		{
			if(amplitudes[i]>AmplitudeThreshold){
				cloud_ptr->points[i].x = cartesianDist[(i*3) + 0];
				cloud_ptr->points[i].y = cartesianDist[(i*3) + 1];
				cloud_ptr->points[i].z = cartesianDist[(i*3) + 2];
				cloud_ptr->points[i].intensity = amplitudes[i];
				countWidth++;
			}
		}

	} else {

		for (size_t i = 0; i < cloud_ptr->points.size (); ++i)
		{
			cloud_ptr->points[i].x = cartesianDist[(i*3) + 0];
			cloud_ptr->points[i].y = cartesianDist[(i*3) + 1];
			cloud_ptr->points[i].z = cartesianDist[(i*3) + 2];
			cloud_ptr->points[i].intensity = amplitudes[i];
			countWidth++;
		}
	}

	cloud_ptr->width    = countWidth;
	cloud_ptr->points.resize (cloud_ptr->width * cloud_ptr->height);

	/*
	 * Filtering the Data
	 */

	 if(StatisticalNoiseFilterOn){
		 pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
		 sor.setInputCloud (cloud_ptr);
		 sor.setMeanK (30);
		 sor.setStddevMulThresh (0.4);
		 sor.filter (*cloud_ptr_filtered);
		 sor.setNegative (true);
		 sor.filter (*cloud_ptr_outliers);
	 }

	 /*
	  * Publishing the messages
	  */

	 if(AmplitudeFilterOn){

		 if(StatisticalNoiseFilterOn){
			 publishCloud(cloud_ptr_filtered, pub_filtered);
			 publishCloud(cloud_ptr_outliers, pub_outliers);
		 } else {
			 publishCloud(cloud_ptr, pub_filtered);
		 }


	 } else {

		 if(StatisticalNoiseFilterOn){
			 publishCloud(cloud_ptr_filtered, pub_filtered);
			 publishCloud(cloud_ptr_outliers, pub_outliers);
		 }

	 }

	 PointCloud::Ptr msg_non_filtered (new PointCloud);
	 msg_non_filtered->header.frame_id = "tf_argos";
	 msg_non_filtered->height = 1;
	 msg_non_filtered->width = noOfRows*noOfColumns;
	 for  (int i =0; i< noOfRows*noOfColumns ; i++){
		 pcl::PointXYZI temp_point;
		 temp_point.x = cartesianDist[(i*3) + 0];
		 temp_point.y = cartesianDist[(i*3) + 1];
		 temp_point.z = cartesianDist[(i*3) + 2];
		 temp_point.intensity = amplitudes[i];
		 msg_non_filtered->points.push_back(temp_point);
		 //msg_non_filtered->points.push_back ( pcl::PointXYZI(cartesianDist[(i*3) + 0],cartesianDist[(i*3) + 1],cartesianDist[(i*3) + 2], amplitudes[i]) );
	 }
#if ROS_VERSION > ROS_VERSION_COMBINED(1,9,49)
	msg_non_filtered->header.stamp = ros::Time::now().toNSec();
#else
	msg_non_filtered->header.stamp = ros::Time::now();
#endif
	pub_non_filtered.publish (msg_non_filtered);

	return 1;
}

int main(int argc, char *argv[]) {
	ROS_INFO("Starting argos3d_p100 ros...");
	ros::init (argc, argv, "argos3d_p100");
	ros::NodeHandle nh;

	dynamic_reconfigure::Server<argos3d_p100::argos3d_p100Config> srv;
	dynamic_reconfigure::Server<argos3d_p100::argos3d_p100Config>::CallbackType f;

	f = boost::bind(&callback, _1, _2);

	if(initialize(argc, argv,nh)){
		first = true;
		srv.setCallback(f);
		first = false;
		ROS_INFO("Initalized Camera... Reading Data");
		ros::Rate loop_rate(10);
		while (nh.ok() && dataPublished)
		{
			if(publishData()) dataPublished==true;
			ros::spinOnce ();
			loop_rate.sleep ();
		}
	} else {
		ROS_WARN("Cannot Initialize Camera. Check the parameters and try again!!");
		return 0;
	}

	pmdClose (hnd);
	return 0;
}
