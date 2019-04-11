#ifndef LP_HEADING
#define LP_HEADING
class LP_heading
{
public:
	LP_heading(int posBufferSize, double minDelta, double maxDelta, double initHeading);

	double getHeading();
	void addGyroSample(double gyrZ);
	void addPosPoint(double x, double y, double std_dev, double time); 

private: 
	int posBufferSize;
	double minDelta;
	double maxDelta;
	double gyrSum;
	int gyrNumberOfSamples;
	double posBufferHeading;
	double heading;
	double lastTimeAdded;

	double posBufferX[100];
	double posBufferY[100];
};

#endif