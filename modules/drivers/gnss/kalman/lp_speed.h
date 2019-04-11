#ifndef LP_SPEED
#define LP_SPEED
class LP_speed
{
public:
	LP_speed(int posBufferSize);

	double getSpeed();
	void addPosPoint(double x, double y, double time); 

private: 
	int posBufferSize;
	double bufferSpeed;
	double nrOfSamples;
	double posBufferSpeed;
	double speed;
	double posBufferX[100];
	double posBufferY[100];
	double posBufferTime[100];
};

#endif