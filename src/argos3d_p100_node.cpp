/******************************************************************************
 * Copyright (c) 2013
 * VoXel Interaction Design GmbH
 *
 * @author Angel Merino Sastre
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 ******************************************************************************/

/** @mainpage Argos3D P100 ROS package
 *
 * @section intro_sec Introduction
 *
 * This software defines a interface for working through the network interface with
 * the ToF camera Sentis-m100 form Bluetechnix GmbH. It allows to create tcp control 
 * connections in order to write and read register of the camera which define its 
 * configuration. Also provides a continuous reader for the image frames through the 
 * udp data connection.
 *
 * @section install_sec Installation
 *
 * We encorage you to follow the instruction we prepared in:
 *
 * ROS wiki: http://wiki.ros.org/argos3d_p100
 * Github repository: https://github.com/voxel-dot-at/argos3d_p100_ros_pkg
 *
 */

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
int frameRate;
bool bilateralFilter;

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

/**
 *
 * @brief This method prints help in command line if given --help option
 * or if there is any error in the options 
 *
 */
int help() {
	std::cout << "\n Using help for argos3d_p100_ros_pkg\n"
		" You can set default configuration values for the camera with the following options: \n" << std::endl;
	std::cout << " Usage:\n rosrun argos3d_p100 argos3d_p100_node "<< std::endl
		<< "\t-it *Integration_Time* \n\tIntegration time(in msec) for the sensor \n\t(min: 100 | max: 2700 | default: 1500) "<< std::endl
		<< "\t-mf  *Modulation_Frequency* \n\tSet the modulation frequency(Hz) of the sensor \n\t(min: 5000000 | max: 30000000 | default: 30000000) "<< std::endl
		<< "\t-bf *Bilateral_Filter* \n\tTurns bilateral filtering on or off \n\t(ON: if set | OFF: default) "<< std::endl
		<< "\t-fr *Frame_Rate* \n\tSet the frame rate of the camera by setting the Phase Time \n\t(min: 1 | max: 40 | default: 40)" << std::endl
		<< "\t-af *Amplitude_Filter_On* \n\tWhether to apply amplitude filter or not. Image pixels with amplitude values less than the threshold will be filtered out \n\t(ON: if set | OFF: default) " << std::endl
		<< "\t-at *Amplitude_Threshold* \n\tWhat should be the amplitude filter threshold. Image pixels with lesser aplitude values will be filtered out. Amplitude Filter Status should be true to use this filter \n\t(min: 0 | max: 2500 | default: 0) "<< std::endl
		<< "\n Example:" << std::endl
		<< "rosrun argos3d_p100 argos3d_p100_node -it 1500 -mf 30000000 \n" << std::endl;
	exit(0);
} //print_help

/**
 *
 * @brief Callback for rqt_reconfigure. It is called any time we change a 
 * parameter in the visual interface 
 *
 * @param [in] argos3d_p100::argos3d_p100Config
 * @param [in] uint32_t
 *
 */
void callback(argos3d_p100::argos3d_p100Config &config, uint32_t level)
{
	// Check the configuretion parameters with those given in the initialization
	if(first) {
		config.Integration_Time = integrationTime;
		config.Modulation_Frequency = modulationFrequency;
		config.Frame_rate = frameRate;
		config.Bilateral_Filter = !bilateralFilter;
		
		config.Amplitude_Filter_On = AmplitudeFilterOn;
		config.Amplitude_Threshold = AmplitudeThreshold;
		
		integrationTime = modulationFrequency = frameRate = -1;
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
	
	if(frameRate != config.Frame_rate) {
		frameRate = config.Frame_rate;
		err[0] = 0;
		std::stringstream spt;
		spt << "SetPhaseTime " << (1000000/(4*frameRate));
		res = pmdSourceCommand (hnd, err, sizeof(err), spt.str().c_str());
		if (res != PMD_OK) {
			pmdGetLastError (hnd, err, 128);
			ROS_WARN_STREAM("Could not set frame rate: " << err);
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

	AmplitudeFilterOn = config.Amplitude_Filter_On;
	AmplitudeThreshold = config.Amplitude_Threshold;
}

/**
 *
 * @brief Initialize the camera and initial parameter values. Returns 1 if properly initialized.
 *
 * @param [in] int
 * @param [in] argv
 * @param [in] ros::NodeHandle
 *
 */
int initialize(int argc, char *argv[],ros::NodeHandle nh){
	/*
	 * Inital Setup for parameters
	 */
	integrationTime = 1500;
	modulationFrequency = 30000000;
	frameRate = 40;
	bilateralFilter = false;
	
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
				|| modulationFrequency < 5000000 || modulationFrequency > 30000000 ) {
				ROS_WARN("*invalid modulation frequency");
				return help();
			}
		}
		if( std::string(argv[i]) == "-fr" ) {
			if( sscanf(argv[++i], "%d", &frameRate) != 1 
				|| frameRate < 1 || frameRate > 40 ) {
				ROS_WARN("*invalid frame rate");
				return help();
			}
		}
		else if( std::string(argv[i]) == "-bf" ) {
			bilateralFilter = true;
		}
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

	char result[128];
	result[0] = 0;
	res = pmdSourceCommand(hnd, result, sizeof(result), "IsCalibrationDataLoaded");
	if (res != PMD_OK)
	{
		pmdGetLastError (0, err, 128);
		ROS_ERROR_STREAM("Could not execute source command: " << err);
		pmdClose (hnd);
		return 0;
	}
	if (std::string(result) == "YES")
		ROS_INFO("Calibration file loaded.");
	else
		ROS_INFO("No calibration file found");

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
	dataPublished=true;
	return 1;
}

static float * cartesianDist = 0;
static float * amplitudes = 0;


/**
 *
 * @brief Publish the data based on set up parameters.
 *
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
	if (!cartesianDist)
		cartesianDist = new float [noOfRows * noOfColumns * 3];
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
	if (!amplitudes)
		amplitudes = new float [noOfRows * noOfColumns];

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
	 
	// Fill in the cloud data
	PointCloud::Ptr msg_non_filtered (new PointCloud);
	msg_non_filtered->header.frame_id = "tf_argos3d";
	msg_non_filtered->height = 1;
	msg_non_filtered->width = noOfRows*noOfColumns;
	
	PointCloud::Ptr msg_filtered (new PointCloud);
	msg_filtered->header.frame_id = "tf_argos3d";
	msg_filtered->width    = 1;
	msg_filtered->height   = noOfColumns*noOfRows;
	msg_filtered->is_dense = false;
	//msg_filtered->points.resize (noOfRows*noOfColumns);

	int countWidth=0;

	for (size_t i = 0; i < noOfRows*noOfColumns; ++i)	{
		pcl::PointXYZI temp_point;
		temp_point.x = cartesianDist[(i*3) + 0];
	 	temp_point.y = cartesianDist[(i*3) + 1];
	 	temp_point.z = cartesianDist[(i*3) + 2];
	 	temp_point.intensity = amplitudes[i];

		if(AmplitudeFilterOn==true && amplitudes[i]>AmplitudeThreshold) {
			msg_filtered->points.push_back(temp_point);
			countWidth++;
		}
		msg_non_filtered->points.push_back(temp_point);
	}
	msg_filtered->height   = countWidth;

	 /*
	  * Publishing the messages
	  */
	 if(AmplitudeFilterOn){
		#if ROS_VERSION > ROS_VERSION_COMBINED(1,9,49)
			msg_filtered->header.stamp = ros::Time::now().toNSec();
		#else
			msg_filtered->header.stamp = ros::Time::now();
		#endif
			pub_filtered.publish (msg_filtered);
	 }

	#if ROS_VERSION > ROS_VERSION_COMBINED(1,9,49)
		msg_non_filtered->header.stamp = ros::Time::now().toNSec();
	#else
		msg_non_filtered->header.stamp = ros::Time::now();
	#endif
		pub_non_filtered.publish (msg_non_filtered);

	return 1;
}

/**
 *
 * @brief Main function
 *
 * @param [in] int
 * @param [in] char *
 *
 */
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

