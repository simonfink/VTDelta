#ifndef ORG_EEROS_CONTROL_TRAJECTORYGENERATOR_HPP_
#define ORG_EEROS_CONTROL_TRAJECTORYGENERATOR_HPP_

#include <eeros/types.hpp>
#include <eeros/control/Block.hpp>
#include <eeros/control/RealSignalInput.hpp>
#include <eeros/control/RealSignalOutput.hpp>
#include <vector>

class TrajectoryGenerator2: public Block {
  
public:
	TrajectoryGenerator2(const double velMax[], const double accMax[], const double decMax[], double dt,  sigdim_t dim, int trajType);
	TrajectoryGenerator2(const double velMax[], const double accMax[],  double dt,  sigdim_t dim, int trajType);

	virtual ~TrajectoryGenerator2();
	virtual void run();
	virtual void reset();
	virtual void setVelMax(const double velMax[]);
	virtual void setAccMax(const double accMax[]);
	virtual void setDecMax(const double decMax[]);
	virtual void setPosition(const double posFinal[], bool isHelp);
	virtual RealSignalOutput& getOut(int outputIndex);
	
private:
	sigdim_t dim;
	static const int dimBuffer = 8; 
	int indexAddPos, indexReadPos, trajType;
	
	std::vector<bool> isNewValue, isHelpValue;
	std::vector<double> velMax, accMax, decMax; 
	std::vector<double> distance, distance1, distance2;
	std::vector<double> posFinalPrev, posPrev; 
	std::vector<std::vector<double> > posFinal;
	std::vector<double> posHelp, posA, posB, cCircle;
	
	static const double dCircle = 0.1;
	double rCircle, thetaCircleInit, thetaCircleEnd, thetaPrev;
	double lambda, velNorm, velNorm1, velNorm2, velNormC; 
	double dt, dT1, dT2, dT3, dT4, dTC; 
	double tOffset, time, timePrev, timeScaled;
	
	virtual void setLambda(int trajType);	
	virtual void setSmoothCurvesParameters();
	virtual void setTrajectoryParameters();
	virtual double setPosGain(double k, double dK);
	virtual double setVelGain(double k, double dK);
	virtual double setAccGain(double k, double dK);

protected:
	bool first;
	bool trajParamSet;
	bool smoothTrajectory;
	bool helpParamError = false;
	
	RealSignalOutput acc;
	RealSignalOutput vel;
	RealSignalOutput pos; 	
};

#endif /* ORG_EEROS_CONTROL_TRAJECTORYGENERATOR_HPP_ */