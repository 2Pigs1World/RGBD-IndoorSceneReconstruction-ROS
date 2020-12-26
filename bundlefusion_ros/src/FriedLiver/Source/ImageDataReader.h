#pragma once

/************************************************************************/
// Reads sensor data files from color & depth image sequence                            
// Inherits from RGBDSensor
/************************************************************************/

#include "GlobalAppState.h"
#include "RGBDSensor.h"

#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

class ImageDataReader : public RGBDSensor{

public:

    ImageDataReader();

    ~ImageDataReader();

    void createFirstConnected();
    
	std::string getSensorName() const;

    bool processDepth();

    bool myProcessDepth(cv_bridge::CvImageConstPtr *pColorImage, cv_bridge::CvImageConstPtr *pDepthImage);


    bool processColor(){
        return true;
    }


private:

	unsigned int		m_numFrames;
	unsigned int		m_currFrame;

	std::string     	m_colorImagePath;
	std::string 		m_depthImagePath;

    float               m_depthShift;
};