// Author: Morgan White SN: 101184799, CoAuthor: Tom Lam SN: 101184799
 
import com.cyberbotics.webots.controller.DistanceSensor;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Robot;

public class Lab1Controller {

  static final double  MAX_SPEED = 6.28;  // maximum speed of the epuck robot

  public static void main(String[] args) {
 
    Robot robot = new Robot();
    int timeStep = (int) Math.round(robot.getBasicTimeStep());

    // Get the motors
    Motor leftMotor = robot.getMotor("left wheel motor");
    Motor rightMotor = robot.getMotor("right wheel motor");
    leftMotor.setPosition(Double.POSITIVE_INFINITY);// indicates continuous rotation servo
    rightMotor.setPosition(Double.POSITIVE_INFINITY);// indicates continuous rotation servo

    // Get and enable the sensors
    DistanceSensor leftAheadSensor = robot.getDistanceSensor("ps7"); 
    DistanceSensor leftAngledSensor = robot.getDistanceSensor("ps6"); 
    DistanceSensor rightAheadSensor = robot.getDistanceSensor("ps0"); 
    DistanceSensor rightAngledSensor = robot.getDistanceSensor("ps1"); 
    DistanceSensor leftSideSensor = robot.getDistanceSensor("ps5"); 
    DistanceSensor rightSideSensor = robot.getDistanceSensor("ps2"); 
    leftAheadSensor.enable(timeStep);
    leftAngledSensor.enable(timeStep);
    rightAheadSensor.enable(timeStep);
    rightAngledSensor.enable(timeStep);
    leftSideSensor.enable(timeStep);
    rightSideSensor.enable(timeStep);
    
    leftMotor.setVelocity(0);
    rightMotor.setVelocity(0);
    
    int duration = 1;
    int dist = 150; 
    
    while (robot.step(timeStep) != -1) {
      // SENSE: Read the distance sensors
      // THINK: Check for obstacle and decide how we need to turn
      // REACT: Move motors accordingly
      
      
      if (leftAngledSensor.getValue() > dist) {
      
        while (leftAngledSensor.getValue() > dist || leftAheadSensor.getValue() > dist || rightAngledSensor.getValue() > dist || rightAheadSensor.getValue() > dist) {
        
          spinRight(leftMotor, rightMotor);
          robot.step(timeStep);
        
        }
      
        
        //robot.step(timeStep*duration);
      
      } else if (rightAngledSensor.getValue() > dist) {
      
        while (leftAngledSensor.getValue() > dist || leftAheadSensor.getValue() > dist || rightAngledSensor.getValue() > dist || rightAheadSensor.getValue() > dist) { 
        
          spinLeft(leftMotor, rightMotor);
          robot.step(timeStep*duration);
        }
      
      } else if (leftAheadSensor.getValue() > dist || rightAheadSensor.getValue() > dist) {
      
        
          double rand = Math.random();
        
          if (rand < 0.5) {
        
            while (leftAngledSensor.getValue() > dist || leftAheadSensor.getValue() > dist || rightAngledSensor.getValue() > dist || rightAheadSensor.getValue() > dist) {
        
              spinLeft(leftMotor, rightMotor);
              robot.step(timeStep*duration);
            
            }
            //robot.step(timeStep*duration);
        
          } else {
          
            while (leftAngledSensor.getValue() > dist || leftAheadSensor.getValue() > dist || rightAngledSensor.getValue() > dist || rightAheadSensor.getValue() > dist) {
            
              spinRight(leftMotor, rightMotor);
              robot.step(timeStep*duration);
          
            }
          
        }
        
      
        } else if (leftSideSensor.getValue() > dist && rightSideSensor.getValue() > dist) {
      
          moveStraight(leftMotor, rightMotor);
          //robot.step(timeStep*duration);
      
      
        } else if (leftSideSensor.getValue() > dist) {
      
          //int count = 0;
          while (leftSideSensor.getValue() > dist && !(leftAngledSensor.getValue() > dist || leftAheadSensor.getValue() > dist || rightAngledSensor.getValue() > dist || rightAheadSensor.getValue() > dist)) {
            curveRight(leftMotor, rightMotor);
            robot.step(timeStep*duration);
            /*
              if (count > 20000) {
              
                while (count > 20000-10) {
                leftMotor.setVelocity(-MAX_SPEED);
                rightMotor.setVelocity(-MAX_SPEED);
    
                System.out.println("backwards");
                count--;
                robot.step(timeStep*duration);
              
                }
                count = 0;
              
              }*/
          
        }
      
      } else if (rightSideSensor.getValue() > dist) {
      
        //int count = 0;
        while (rightSideSensor.getValue() > dist && !(leftAngledSensor.getValue() > dist || leftAheadSensor.getValue() > dist || rightAngledSensor.getValue() > dist || rightAheadSensor.getValue() > dist)) {
          curveLeft(leftMotor, rightMotor);
          robot.step(timeStep*duration);
          /*
          if (count > 20000) {
            
              while (count > 20000-10) {
              leftMotor.setVelocity(-MAX_SPEED);
              rightMotor.setVelocity(-MAX_SPEED);
  
              System.out.println("backwards");
              count--;
              robot.step(timeStep*duration);
            
              }
              count = 0;
            
            }
          
        
          */
          
          
        }
        //robot.step(timeStep*duration);
        
      
      } else {
    
        moveStraight(leftMotor, rightMotor);
    
      }
    
    
    }
  }
  
  
  
  public static void moveStraight(Motor leftMotor, Motor rightMotor) {
  
    leftMotor.setVelocity(MAX_SPEED);
    rightMotor.setVelocity(MAX_SPEED);
  
    //System.out.println("straight state");
  
  
  
  }
  
  public static void spinRight(Motor leftMotor, Motor rightMotor) {
  
    leftMotor.setVelocity(0.5*MAX_SPEED);
    rightMotor.setVelocity(0.5*-MAX_SPEED);
  
  //System.out.println("spin right state");
  
  }
  
  public static void spinLeft(Motor leftMotor, Motor rightMotor) {
  
    leftMotor.setVelocity(0.5*-MAX_SPEED);
    rightMotor.setVelocity(0.5*MAX_SPEED);
  
  //System.out.println("spin left state");
  
  }
  
  public static void curveRight(Motor leftMotor, Motor rightMotor) {
  
    leftMotor.setVelocity(MAX_SPEED);
    rightMotor.setVelocity(0.75*MAX_SPEED);
  
  //System.out.println("curve right state");
  
  }
  
  public static void curveLeft(Motor leftMotor, Motor rightMotor) {
  
    leftMotor.setVelocity(0.75*MAX_SPEED);
    rightMotor.setVelocity(MAX_SPEED);
  
    //System.out.println("curveleft state");
  
  }
  
  
}
