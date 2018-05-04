#include<math.h>
 
 void inverseKinematic(const double cartesianCoords[]){
    
  double lArm1 = 0.25;
  double lArm2 = 0.25;
  
  double phi1, phi2, phi3, phi4; 
  
  phi2 = acos((-lArm1*lArm1-lArm2*lArm2+cartesianCoords[0]*cartesianCoords[0]+cartesianCoords[1]*cartesianCoords[1])/(2*lArm1*lArm2));

  // avoid singularity, phi2 doesn't have to be equals to zero
  if(phi2>-0.2 && phi2 <0.2){
    if(phi2<0)
      phi2 = -0.2;
    else
      phi2 = 0.2;
  }
  
  // calculate phi1
  if (phi2 >= 0)
    phi1 = atan(cartesianCoords[1]/cartesianCoords[0]) - acos((cartesianCoords[0]*cartesianCoords[0]+cartesianCoords[1]*cartesianCoords[1]+lArm1*lArm1-lArm2*lArm2)/(2*lArm1*sqrt(cartesianCoords[0]*cartesianCoords[0]+cartesianCoords[1]*cartesianCoords[1])));
  else
    phi1 = atan(cartesianCoords[1]/cartesianCoords[0]) + acos((cartesianCoords[0]*cartesianCoords[0]+cartesianCoords[1]*cartesianCoords[1]+lArm1*lArm1-lArm2*lArm2)/(2*lArm1*sqrt(cartesianCoords[0]*cartesianCoords[0]+cartesianCoords[1]*cartesianCoords[1])));
    
  phi3 = -cartesianCoords[2];
  phi4 = phi1+phi2-cartesianCoords[3];
  }
