#include "ParallelScaraControlSystem.hpp"
#include <eeros/core/Executor.hpp>

using namespace parallelscara;
using namespace eeros::safety;
using namespace eeros::control;
using namespace eeros::math;

	ParallelScaraControlSystem::ParallelScaraControlSystem(SafetySystem& safetySys, ParallelScaraSafetyProperties& ssProperties) :
		hall0("hall0"), hall1("hall1"), hall2("hall2"), hall3("hall3"),
		hallXout("hallXout"), hallYout("hallYout"),
		q0EncPos("enc0"), q1EncPos("enc1"),
		q0EncOffset(0.0), q1EncOffset(0.0),
		q0InvGearRatio(1.0/i), q1InvGearRatio(1.0/i),
		directKinematic(l1, l2),
		pathPlanner(velMax, accMax, decMax, dt),
// 		pathPlanner(dt),
		inverseKinematic(l1, l2, q0_limit, q1_limit, deltaQ_limMin, deltaQ_limMax),
		xyRefPosSwitch(0),	// set to path planner input
		dac0("dacOut0"), dac1("dacOut1"),
		checkPos(safPosLimitLower, safPosLimitUpper),
		checkVel(velLimitLower, velLimitUpper),
		robotController(safetySys, ssProperties),
		setWatchdog("Wdt"),
		trace1(traceLen),
		trace2(traceLen),
		trace3(traceLen),
		trace4(traceLen),
		
		timedomain("Main time domain", dt, true)
	{
		hallMux.getIn(0).connect(hall0.getOut());
		hallMux.getIn(1).connect(hall1.getOut());
		hallMux.getIn(2).connect(hall2.getOut());
		hallMux.getIn(3).connect(hall3.getOut());
		hallDataRead.getIn().connect(hallMux.getOut());	
		hallXout.getIn().connect(hallDataRead.getOutX());
		hallYout.getIn().connect(hallDataRead.getOutY());
	
		q0EncSumOffset.getIn(0).connect(q0EncPos.getOut());
		q0EncSumOffset.getIn(1).connect(q0EncOffset.getOut());
		q1EncSumOffset.getIn(0).connect(q1EncPos.getOut());
		q1EncSumOffset.getIn(1).connect(q1EncOffset.getOut());
		q0InvGearRatio.getIn().connect(q0EncSumOffset.getOut());
		q1InvGearRatio.getIn().connect(q1EncSumOffset.getOut());
		encPosMux.getIn(0).connect(q0InvGearRatio.getOut());
		encPosMux.getIn(1).connect(q1InvGearRatio.getOut());
		encPosMux.getOut().getSignal().setName("enc pos act");

		rotAngleTransformation.getInPhi().connect(hallDataRead.getOut());
		rotAngleTransformation.getInEncPos().connect(encPosMux.getOut());
		directKinematic.getOut().getSignal().setName("xy pos");
		directKinematic.getIn().connect(encPosMux.getOut());  
		pathPlanner.getPosOut().getSignal().setName("xy pos ref from pp");
// 		pathPlannerMux.getIn(0).connect(pathPlannerX.getPosOut());
// 		pathPlannerMux.getIn(1).connect(pathPlannerY.getPosOut());
// 		pathPlannerMux.getOut().getSignal().setName("xy pos ref from pp");
		
		pendulumController.getIn_refPos_xy().connect(pathPlanner.getPosOut());
		pendulumController.getIn_actPos_xy().connect(directKinematic.getOut());           
		pendulumController.getIn_phiHall().connect(rotAngleTransformation.getOutPhi()); 
// 		pendulumController.getEnableIntegral().connect(hallDataRead.getBarOn()); 
		pendulumController.getOutRefPos().getSignal().setName("xy pos ref from tip controller");
			
		xyRefPosSwitch.getIn(0).connect(pathPlanner.getPosOut());
		xyRefPosSwitch.getIn(1).connect(pendulumController.getOutRefPos());
		xyRefPosSwitch.getOut().getSignal().setName("xy ref pos");
		inverseKinematic.getInActPosXY().connect(xyRefPosSwitch.getOut());                       
		inverseKinematic.getInActPosEnc().connect(encPosMux.getOut());  	
		velPathPl.getIn().connect(inverseKinematic.getOut());                    
		
		robotController.getInEncRefPos().connect(inverseKinematic.getOut());       
		robotController.getInEncRefVel().connect(velPathPl.getOut());       
		robotController.getInEncPosAct().connect(encPosMux.getOut());
		demuxDAC.getIn().connect(robotController.getOutSetpointMotors());
		dac0.getIn().connect(demuxDAC.getOut(0));
		dac1.getIn().connect(demuxDAC.getOut(1));
		
		encPosDiff.getIn().connect(encPosMux.getOut());
		checkVel.getIn().connect(encPosDiff.getOut());
		checkVel.registerSafetyEvent(safetySys, ssProperties.doEmergency);
		checkPos.getIn().connect(encPosMux.getOut());
		checkPos.registerSafetyEvent(safetySys, ssProperties.doEmergency);
		trace1.getIn().connect(encPosMux.getOut());
		trace2.getIn().connect(directKinematic.getOut());
		trace3.getIn().connect(robotController.velSum.getOut());
		trace4.getIn().connect(robotController.getOutSetpointMotors());
		
		// Run blocks
		timedomain.addBlock(hall0);
		timedomain.addBlock(hall1);
		timedomain.addBlock(hall2);
		timedomain.addBlock(hall3);
		timedomain.addBlock(hallMux);
		timedomain.addBlock(hallDataRead);
		timedomain.addBlock(hallXout);
		timedomain.addBlock(hallYout);
		
		timedomain.addBlock(q0EncPos);
		timedomain.addBlock(q1EncPos);
		timedomain.addBlock(q0EncOffset);
		timedomain.addBlock(q1EncOffset);
		timedomain.addBlock(q0EncSumOffset);
		timedomain.addBlock(q1EncSumOffset);
		timedomain.addBlock(q0InvGearRatio);
		timedomain.addBlock(q1InvGearRatio);
		timedomain.addBlock(encPosMux);
		
		timedomain.addBlock(rotAngleTransformation);
		timedomain.addBlock(directKinematic);
		timedomain.addBlock(pathPlanner);
		timedomain.addBlock(pendulumController);
		timedomain.addBlock(xyRefPosSwitch);	
		timedomain.addBlock(inverseKinematic);	
		timedomain.addBlock(velPathPl);  
		
		timedomain.addBlock(robotController);
		timedomain.addBlock(demuxDAC);
		timedomain.addBlock(dac0);
		timedomain.addBlock(dac1);
		
		timedomain.addBlock(encPosDiff);
		timedomain.addBlock(checkVel);
		timedomain.addBlock(checkPos);
		timedomain.addBlock(setWatchdog);
		timedomain.addBlock(trace1);
		timedomain.addBlock(trace2);
		timedomain.addBlock(trace3);
		timedomain.addBlock(trace4);
		
		eeros::Executor::instance().add(timedomain);
	}
	 
	void ParallelScaraControlSystem::start() {
		timedomain.start();
	}

	void ParallelScaraControlSystem::stop() {
		timedomain.stop();
	}
