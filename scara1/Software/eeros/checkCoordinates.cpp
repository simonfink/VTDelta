 void checkCoordinates(const double posFinal[]){
    
  double lArm1 = 0.25;
  double lArm2 = 0.25;
  double zMax = 0.15;
  double zMin = 0;

  double a = 0.1; 
  double b = 0.1;
  double r2 = lArm1+lArm2;
  
  bool coordInWorkspace = false; 
  bool checkCoord = false; 

  if  (((posFinal[0]*posFinal[0]+posFinal[1]*posFinal[1])<(lArm1*lArm1)) || ((posFinal[0]*posFinal[0]+posFinal[1]*posFinal[1])>(r2*r2))) {
    
    if(posFinal[1]<=0){
	  if((lArm2*lArm2<((posFinal[0]+a)*(posFinal[0]+a)+(posFinal[1]-b)*(posFinal[1]-b))&&(posFinal[0]<=0)))
	      coordInWorkspace = 0;
	  if((lArm2*lArm2<((posFinal[0]-a)*(posFinal[0]-a)+(posFinal[1]+b)*(posFinal[1]+b))&&(posFinal[0]<=0)))
	      coordInWorkspace = 0;
      }
      else
	  coordInWorkspace = 0;
  }
  else
      coordInWorkspace = 1;

  if((coordInWorkspace == 0)||(posFinal[2] <= zMin) ||(posFinal[2] >= zMax))
      coordInWorkspace = 0;
  else
    coordInWorkspace = 1;

  checkCoord = true; 
  }
