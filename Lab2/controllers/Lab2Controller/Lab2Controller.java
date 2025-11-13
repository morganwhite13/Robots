// Author: Morgan White (SN: 101184799) & Tanay Shah (SN: 101183078)
 
import com.cyberbotics.webots.controller.DistanceSensor;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Robot;

public class Lab2Controller {

  // Various modes that the robot may be in
  static final byte    STRAIGHT = 0;
  static final byte    SPIN_LEFT = 1;
  static final byte    PIVOT_RIGHT = 2;
  static final byte    CURVE_LEFT = 3;
  static final byte    CURVE_RIGHT = 4;

  static final double  MAX_SPEED = 5;  // maximum speed of the epuck robot

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
    DistanceSensor rightAheadSensor = robot.getDistanceSensor("ps0"); 
    DistanceSensor rightAngledSensor = robot.getDistanceSensor("ps1"); 
    DistanceSensor rightSideSensor = robot.getDistanceSensor("ps2"); 
    leftAheadSensor.enable(timeStep);
    rightAheadSensor.enable(timeStep);
    rightAngledSensor.enable(timeStep);
    rightSideSensor.enable(timeStep);

    // Initialize the logic variable for turning
    byte    currentMode = STRAIGHT;
    double  leftSpeed, rightSpeed;

    
    int far = 90;
    int close = 100;
    int veryFar = 80;
    int superFar = 67;
    
    
    while (robot.step(timeStep) != -1) {
      // SENSE: Read the distance sensors
      // THINK: Check for obstacle and decide how we need to turn             
      // REACT: Move motors accordingly\
      
      
      
       leftAheadSensor = robot.getDistanceSensor("ps7"); 
       rightAheadSensor = robot.getDistanceSensor("ps0"); 
       rightAngledSensor = robot.getDistanceSensor("ps1"); 
       rightSideSensor = robot.getDistanceSensor("ps2"); 
        
       //System.out.println(rightSideSensor.getValue());
      
       //if (leftAheadSensor.getValue() > close || rightAheadSensor.getValue() > close || rightAngledSensor.getValue() > close) {
       if ((leftAheadSensor.getValue() > close || rightAheadSensor.getValue() > close || rightAngledSensor.getValue() > close) && (leftAheadSensor.getValue() > superFar && rightAheadSensor.getValue() > superFar && rightAngledSensor.getValue() > superFar)) {
       
         spinLeft(leftMotor, rightMotor);
       
       } else if (rightSideSensor.getValue() < veryFar) {
       
         pivotRight(leftMotor, rightMotor);
       
       } else if (rightSideSensor.getValue() < far) {
       
         curveRight(leftMotor, rightMotor);
       
       
       } else if (rightSideSensor.getValue() > close) {
       
         curveLeft(leftMotor, rightMotor);
       
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
  
    //leftMotor.setVelocity(0.5*-MAX_SPEED);
    //rightMotor.setVelocity(0.5*MAX_SPEED);
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
  
  
  
  public static void pivotRight(Motor leftMotor, Motor rightMotor) {
  
    leftMotor.setVelocity(MAX_SPEED);
    rightMotor.setVelocity(0.1*MAX_SPEED);
  
    //System.out.println("PIVOTING");
  
  //System.out.println("spin right state");
  
  }
  
  
  
  
  
  
  
  
  
  
  
  
}







