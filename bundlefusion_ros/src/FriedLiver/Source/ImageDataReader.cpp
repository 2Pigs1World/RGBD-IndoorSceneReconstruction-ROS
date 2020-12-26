#include "ImageDataReader.h"
#include "GlobalAppState.h"
#include "GlobalBundlingState.h"
#include "SiftGPU/MatrixConversion.h"
#include "PoseHelper.h"

#include <algorithm>
#include <iostream>
#include <fstream>
#include <list>
#include <vector>
#include <string>
#include <iomanip>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include <ros/ros.h>
#include <ros/topic.h>

ImageDataReader::ImageDataReader()
{
	m_currFrame = 0;
}

ImageDataReader::~ImageDataReader(){}

std::string ImageDataReader::getSensorName() const{
	return "Read from image file";
}

void ImageDataReader::createFirstConnected(){
    
	m_colorImagePath = GlobalAppState::get().s_colorImagePath;
	m_depthImagePath = GlobalAppState::get().s_depthImagePath;

	m_numFrames = 5000;
	m_depthShift = 1000;

	initializeDepthIntrinsics(596, 596, 333, 246);

	initializeColorIntrinsics(622, 623, 335, 233);

	// RGBDSensor::init(depthWidth, depthHeight, colorWidth, colorHeight, depthRingBufferSize)
	RGBDSensor::init(640, 480, 640, 480, 1);
}


bool ImageDataReader::processDepth(){

	cv::Mat depth;
	cv::Mat color;

	if(m_currFrame < m_numFrames){

		std::stringstream str;
		std::string index;
		str << std::setw(6) << std::setfill('0') << std::to_string(m_currFrame);
		str >> index;

		std::string depthFile = m_depthImagePath + "frame-" + index + ".depth.png";
		std::string colorFile = m_colorImagePath + "frame-" + index + ".color.jpg";

		depth = cv::imread(depthFile, cv::IMREAD_ANYDEPTH);
		color = cv::imread(colorFile, cv::IMREAD_COLOR);

	}

	assert(!depth.empty());
	assert(!color.empty());

	m_currFrame++;
	
	// Data type for imread cv::IMREAD_ANYDEPTH is cv::CV_16UC1
	// Convert to float
	float* depthFloat = getDepthFloat();

	for(int y=0; y<m_depthHeight; y++){
		for(int x=0; x<m_depthWidth; x++){
			int index = x + y*m_depthWidth;
			if(depth.at<uint16_t>(y, x) == 0) depthFloat[index] = -std::numeric_limits<float>::infinity();
			else depthFloat[index] = (float)depth.at<uint16_t>(y, x) / m_depthShift; // depth shift, convert millimeter to meter?
		}
	}
	incrementRingbufIdx();

	// Data type for imread cv::IMREAD_COLOR is cv::CV_8UC3
	// Convert to vec4uc
	for(int y=0; y<m_colorHeight; y++){
		for(int x=0; x<m_colorWidth; x++){
			int index = x + y*m_colorWidth;
			// 3-dim vector of uchar in BGR
			cv::Vec3b pixel = color.at<cv::Vec3b>(y,x); 
			vec4uc pixel_v4uc(pixel[2], pixel[1], pixel[0], 0x01);			
			m_colorRGBX[index] = pixel_v4uc;
		}
	}
	
	return true;

}

bool ImageDataReader::myProcessDepth(cv_bridge::CvImageConstPtr *pColorImage, cv_bridge::CvImageConstPtr *pDepthImage){

		std::cout<<"############myProcessDepth"<<std::endl;

	cv::Mat depth;
	cv::Mat color;

	if(m_currFrame < m_numFrames){

		std::stringstream str;
		std::string index;
		str << std::setw(6) << std::setfill('0') << std::to_string(m_currFrame);
		str >> index;

		std::string depthFile = m_depthImagePath + "frame-" + index + ".depth.png";
		std::string colorFile = m_colorImagePath + "frame-" + index + ".color.jpg";

		// depth = cv::imread(depthFile, cv::IMREAD_ANYDEPTH);
		// color = cv::imread(colorFile, cv::IMREAD_COLOR);

		(*pColorImage)->image.copyTo(color);
		(*pDepthImage)->image.copyTo(depth);

	}

	assert(!depth.empty());
	assert(!color.empty());

	m_currFrame++;
	
	// Data type for imread cv::IMREAD_ANYDEPTH is cv::CV_16UC1
	// Convert to float
	float* depthFloat = getDepthFloat();

	for(int y=0; y<m_depthHeight; y++){
		for(int x=0; x<m_depthWidth; x++){
			int index = x + y*m_depthWidth;
			if(depth.at<uint16_t>(y, x) == 0) depthFloat[index] = -std::numeric_limits<float>::infinity();
			else depthFloat[index] = (float)depth.at<uint16_t>(y, x) / m_depthShift; // depth shift, convert millimeter to meter?
			// std::cout << (float)depth.at<uint16_t>(y, x) / m_depthShift << std::endl;
		}
	}
	incrementRingbufIdx();

	// Data type for imread cv::IMREAD_COLOR is cv::CV_8UC3
	// Convert to vec4uc
	for(int y=0; y<m_colorHeight; y++){
		for(int x=0; x<m_colorWidth; x++){
			int index = x + y*m_colorWidth;
			// 3-dim vector of uchar in BGR
			cv::Vec3b pixel = color.at<cv::Vec3b>(y,x); 
			vec4uc pixel_v4uc(pixel[2], pixel[1], pixel[0], 0x01);			
			m_colorRGBX[index] = pixel_v4uc;
		}
	}
	
	return true;

}

