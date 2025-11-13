// Author: Morgan White (SN: 101184799) & COAUTHOR Cameron Rolfe SN: 101073278

import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Robot;
import com.cyberbotics.webots.controller.PositionSensor;

public class Lab5Controller {

  static final double  MAX_SPEED      = 0.25;  //6.28 is maximum but value of 1 is required for simulator to work properly for kinematics
  static final double  WHEEL_RADIUS   = 2.05; // cm 
  static final double  WHEEL_BASE     = 5.80; // cm
  
  static Robot           Epuck;
  static Motor           LeftMotor;
  static Motor           RightMotor;
  static PositionSensor  rightEncoder;
  static double          PreviousReading = 0;
  static int             TimeStep;


  // Set each motor to a specific speed and wait until the left sensor reaches 
  // the specified number of radians.
  private static void move(double leftSpeed, double rightSpeed, double thisManyRadians) {
    LeftMotor.setVelocity(leftSpeed);
    RightMotor.setVelocity(rightSpeed);
    while(true) {
      double reading = rightEncoder.getValue() - PreviousReading;
      if ((thisManyRadians > 0) && (reading >= thisManyRadians)) 
        break;
      if ((thisManyRadians < 0) && (reading <= thisManyRadians)) 
        break;
      Epuck.step(TimeStep);
    }
    LeftMotor.setVelocity(0);
    RightMotor.setVelocity(0);
    Epuck.step(TimeStep);
    PreviousReading = rightEncoder.getValue();
    Epuck.step(TimeStep);
  }
  
	
  public static void main(String[] args) {
    Epuck = new Robot();
    TimeStep = (int) Math.round(Epuck.getBasicTimeStep());

    // Get the motors
    LeftMotor = Epuck.getMotor("left wheel motor");
    RightMotor = Epuck.getMotor("right wheel motor");
    LeftMotor.setPosition(Double.POSITIVE_INFINITY);// indicates continuous rotation servo
    RightMotor.setPosition(Double.POSITIVE_INFINITY);// indicates continuous rotation serv
    LeftMotor.setVelocity(0);
    RightMotor.setVelocity(0);
    
    // Get the encoders
    rightEncoder = Epuck.getPositionSensor("right wheel sensor");
    rightEncoder.enable(TimeStep);
       
    // Travel through the points in the array one at a time in sequence
    // WRITE YOUR CODE HERE
    //
    

    double[] arrayThingX = {0,30,30,-10,10,-10,-60,-50,-40,-30,-20,0};
    double[] arrayThingY = {0,30,60,70,50,50,40,30,30,20,20,0};
    
    double curTheta = 90;
    double xChange;
    double yChange;
    double newTheta;
    double thetaChange;
    double forwardTime;
    
    for (int i = 1; i < arrayThingX.length; i++) {
    
      xChange = arrayThingX[i] - arrayThingX[i - 1];
      yChange = arrayThingY[i] - arrayThingY[i - 1];
    
      newTheta = Math.atan2(yChange,xChange)*180/Math.PI;
      thetaChange = (newTheta - curTheta) % 360;
      
      
      if (thetaChange > 180) {
      
        thetaChange -= 360;
      
      } else if (thetaChange < -180) {
      
        thetaChange += 360;
      
      }
      //System.out.println(thetaChange);
      
      if (thetaChange > 0) {
      
        move(-MAX_SPEED, MAX_SPEED, Math.toRadians(thetaChange*WHEEL_BASE/WHEEL_RADIUS/2));
      
      } else {
      
        move(MAX_SPEED, -MAX_SPEED, Math.toRadians(thetaChange*WHEEL_BASE/WHEEL_RADIUS/2));
      
      }
      
      
      forwardTime = Math.sqrt(xChange*xChange+yChange*yChange)/WHEEL_RADIUS;
      
      move(MAX_SPEED, MAX_SPEED, forwardTime);
      
      
      
      curTheta = newTheta;
    
    
    }
    
    
  }
}
