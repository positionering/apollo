// Written by the Twizy Perception team at Chalmers 2018.
// Copyright 2018

#include "lp_heading.h"
#include <math.h>
#include <ros/ros.h>


	double normalize(double angle){
		return atan2(sin(angle), cos(angle));
	}

	LP_heading::LP_heading(int posBufferSize, double minDelta, double maxDelta,double initHeading){
		this->posBufferSize = posBufferSize;
		this->minDelta = minDelta;
		this->maxDelta = maxDelta;
		this->gyrSum = 0;
		this->gyrNumberOfSamples = 0;
		this->posBufferHeading = 0;
		this->heading = initHeading;
		this->lastTimeAdded = 0;

		for (int i = 0; i<posBufferSize; i++){
			this->posBufferX[i] = 0;
			this->posBufferY[i] = 0;
		}
		return;
	}

	double LP_heading::getHeading(){
		return heading;
	}
	void LP_heading::addGyroSample(double gyrZ){
		gyrSum += gyrZ;
		gyrNumberOfSamples++;
	}
	void LP_heading::addPosPoint(double x, double y, double std_dev, double time){
		double dx = x - posBufferX[0];
		double dy = y - posBufferY[0];
		double limit  = 3*std_dev;
		if (limit > maxDelta) limit = maxDelta;
		else if (limit < minDelta) limit = minDelta;
		if (dx*dx + dy*dy > limit*limit){
			// add to buffers
			for (int i = posBufferSize-1; i>0; i--){
				posBufferX[i] = posBufferX[i-1];
				posBufferY[i] = posBufferY[i-1];
			}


			posBufferX[0] = x;
			posBufferY[0] = y;
			for (int i = 0; i<posBufferSize; i++){
				//ROS_WARN_STREAM("x[" << i  << "]" << posBufferX[i] << " y[" << i  << "]" << posBufferY[i]);
			}
				double dxq = posBufferX[0] - posBufferX[1];
				double dyq = posBufferY[0] - posBufferY[1];
				double angleq = atan2(dyq,dxq);
				//ROS_WARN_STREAM("dx " << dxq << " dy " << dyq);
				//ROS_WARN_STREAM("New angle: " << angleq*180/3.1415);
			// calculate new buffer heading
			double angleSum = 0;
			for(int i=0; i<posBufferSize-1; i++){
				double dx = posBufferX[i] - posBufferX[i+1];
				double dy = posBufferY[i] - posBufferY[i+1];
				double angle = atan2(dy,dx);
				angleSum += angle;
			}

			posBufferHeading = angleSum/(posBufferSize-1);
			posBufferHeading = normalize(posBufferHeading);
			//ROS_WARN_STREAM("UPDATE ### posBufferHeading: " << posBufferHeading);


		}
		if (gyrNumberOfSamples > 0){
			double deltaT = time - lastTimeAdded;
			if (lastTimeAdded == 0){
				deltaT = 0;
			}
			double gyrH = normalize(heading + gyrSum/gyrNumberOfSamples*deltaT);
			heading = normalize(0.04*posBufferHeading + 0.96*gyrH);
		//ROS_WARN_STREAM("deltaT: " << deltaT);
		//ROS_WARN_STREAM("gyro: " << gyrSum/gyrNumberOfSamples*deltaT);
		//ROS_WARN_STREAM("posBufferHeading: " << posBufferHeading*180/3.1415);

		//ROS_WARN_STREAM("Heading: " << heading*180/3.1415);

		//  Reset counters
		lastTimeAdded = time;
		gyrSum = 0;
		gyrNumberOfSamples = 0;
	}

		return;

	}
