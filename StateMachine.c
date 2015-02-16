/*  The State machine is for demonstrating target acquisition from an array prior 
*   to getting GCode parsder functional.
*   
*   Ok... *THIS* version is static... whatever..
*
*/

void updateState() {            // update LCD readout
    if (doState){
      doState=false;

      switch(currentState) {
        case 0:
          XaxisSetpoint = 1500;       
          YaxisSetpoint = 0;       
          displaySetpoints();
          currentState=1;
          break;
          
        case 1:
          XaxisSetpoint = 1500;       
          YaxisSetpoint = 1500;       
          displaySetpoints();
          currentState=2;
          break;
          
        case 2:
          XaxisSetpoint = 0;       
          YaxisSetpoint = 1500;       
          displaySetpoints();
          currentState=3;
          break;
          
        case 3:
          XaxisSetpoint = 0;       
          YaxisSetpoint = 0;       
          displaySetpoints();
          currentState=0;
          break;
      }  
          
    }
}
