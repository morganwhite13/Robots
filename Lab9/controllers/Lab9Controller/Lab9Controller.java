// Author: Morgan White & Quentin Wolkensperg
 
import com.cyberbotics.webots.controller.DistanceSensor;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.PositionSensor;
import com.cyberbotics.webots.controller.Field;
import com.cyberbotics.webots.controller.Node;
import com.cyberbotics.webots.controller.Supervisor;
import com.cyberbotics.webots.controller.Compass;

public class Lab9Controller {

  // Various modes that the robot may be in
  static final byte    STRAIGHT = 0;
  static final byte    SPIN_LEFT = 1;
  static final byte    PIVOT_RIGHT = 2;
  static final byte    CURVE_LEFT = 3;
  static final byte    CURVE_RIGHT = 4;
  
  static final double  MAX_SPEED       = 1;    // maximum speed of the epuck robot
  static final double  WHEEL_RADIUS    = 2.05; // cm 
  static final double  WHEEL_BASE      = 5.80; // cm

  static PositionSensor  LeftEncoder;
  static PositionSensor  RightEncoder;
  static double          LeftReading, RightReading, PreviousLeft, PreviousRight;
  
  // Store the (x, y) location amd angle (degrees) estimate as well ad radius and theta-delta
  static double X, Y, A, R, TD;


  // Read the compass
  private static int getCompassReadingInDegrees(Compass compass) {
    double compassReadings[] = compass.getValues();
    double rad = Math.atan2(compassReadings[0], compassReadings[2]);
    double bearing = -((rad + Math.PI) / Math.PI * 180.0);
    if (bearing > 180)
      bearing = 360 - bearing;
    if (bearing < -180)
      bearing = 360 + bearing;
    return (int)(bearing);
  }
      
  
  // Update the estimated position
  static void updateEstimate(byte previousMode, Compass compass) {
    LeftReading = LeftEncoder.getValue() - PreviousLeft;
    RightReading = RightEncoder.getValue() - PreviousRight;
    PreviousLeft = LeftEncoder.getValue();
    PreviousRight = RightEncoder.getValue();
    
    //int x = (int) previousMode;
    //System.out.println(" previous node " + x);
    switch(previousMode) {
        case SPIN_LEFT:
          A = getCompassReadingInDegrees(compass);
          break;
          
        case PIVOT_RIGHT:
        case CURVE_LEFT:
        case CURVE_RIGHT:
          //System.out.println(" idkdkd " + Y);
          if (RightReading == LeftReading) {
            X = X + (LeftReading * WHEEL_RADIUS) * Math.cos(Math.toRadians(A));
            Y = Y + (LeftReading * WHEEL_RADIUS) * Math.sin(Math.toRadians(A));
          }
          else {
            R = WHEEL_BASE * (LeftReading  / (RightReading - LeftReading)) + WHEEL_BASE/2;
            TD = (RightReading - LeftReading) * WHEEL_RADIUS / WHEEL_BASE / Math.PI * 180;
            X = X + (R*Math.cos(Math.toRadians(TD))*Math.sin(Math.toRadians(A))) +
                    (R*Math.cos(Math.toRadians(A))*Math.sin(Math.toRadians(TD))) - (R*Math.sin(Math.toRadians(A)));
            Y = Y + (R*Math.sin(Math.toRadians(TD))*Math.sin(Math.toRadians(A))) -
                    (R*Math.cos(Math.toRadians(A))*Math.cos(Math.toRadians(TD))) + (R*Math.cos(Math.toRadians(A)));
            A = getCompassReadingInDegrees(compass);
          }
          break;
        default:
          X = X + (LeftReading * WHEEL_RADIUS) * Math.cos(Math.toRadians(A));
          Y = Y + (LeftReading * WHEEL_RADIUS) * Math.sin(Math.toRadians(A));
          break;
    }
  }
  
  public static void main(String[] args) {

    Supervisor robot = new Supervisor();
    int timeStep = (int) Math.round(robot.getBasicTimeStep());
    
    // Code required for being able to get the robot's location
    Node robotNode = robot.getSelf();
    Field   translationField = robotNode.getField("translation");

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

    // Get the encoders
    LeftEncoder = robot.getPositionSensor("left wheel sensor");
    RightEncoder = robot.getPositionSensor("right wheel sensor");
    LeftEncoder.enable(timeStep);
    RightEncoder.enable(timeStep);
    PreviousLeft = 0; PreviousRight = 0;
    
    // Get the Compass sensor
    Compass  compass = robot.getCompass("compass");
    compass.enable(timeStep);

    // Initialize the logic variables for turning
    byte    currentMode = STRAIGHT;
    byte    previousMode = STRAIGHT;
    double  leftSpeed=0, rightSpeed=0;
    
    // Set the first estimate to match the current location
    double values[] = translationField.getSFVec3f();
    X = (values[0]*100);
    Y = -(values[2]*100); // Need to negate the Y value
    A = 180;
    System.out.printf("Robot starts at (x, y) = (%2.1f, %2.1f, 180 degrees)\n", (values[0]*100), -(values[2]*100));
    
    double goalx = 170;
    double goaly = 0;
    double contactx;
    double contacty;
    double slope = (goaly-Y)/(goalx-X);
    double B = goaly-slope*goalx;
    int far = 90;
    int close = 100;
    int veryFar = 80;
    int superFar = 67;
    boolean clearedObstacle = false;
    double theta = Math.toDegrees(Math.tan((goaly-Y)/(goalx-X)));

    while (robot.step(timeStep) != -1 && !((X > goalx - 2 && X < goalx + 2) && (Y > goaly - 2 && Y < goaly + 2))) {
    //while (robot.step(timeStep) != -1) {
      // SENSE: Read the distance sensors
      boolean sideTooClose  = rightSideSensor.getValue() > 300;
      boolean sideTooFar  = rightSideSensor.getValue() < 81;
      boolean frontTooClose  = (rightAheadSensor.getValue() > 81) || (leftAheadSensor.getValue() > 81) || (rightAngledSensor.getValue() > 81);
      boolean lostContact  = rightSideSensor.getValue() < 81;
        
        // System.out.println("RS" + rightSideSensor.getValue());
        // System.out.println("LA" + leftAheadSensor.getValue());
        // System.out.println("RA" + rightAngledSensor.getValue());
        
        
        //System.out.println("x " + (int) X + " Y " + (int) Y);
        
        
      // THINK: Check for obstacle and decide how we need to turn      
      // switch (currentMode) {
        // case STRAIGHT: //System.out.printf("STRAIGHT\n");
          // if (lostContact) { currentMode = PIVOT_RIGHT; break; }
          // if (sideTooFar) { currentMode = CURVE_RIGHT; break; }
          // if (sideTooClose) { currentMode = CURVE_LEFT; break; }
          // if (frontTooClose) { currentMode = SPIN_LEFT; break; }
          // break;
        // case CURVE_RIGHT:   //System.out.printf("CURVE RIGHT\n");
          // if (!sideTooFar) { currentMode = STRAIGHT; break; }
          // if (sideTooClose) { currentMode = CURVE_LEFT; break; }
          // if (frontTooClose) currentMode = SPIN_LEFT; 
          // break;
        // case PIVOT_RIGHT:   //System.out.printf("PIVOT RIGHT\n");
          // if (sideTooClose) { currentMode = CURVE_LEFT; break; }
          // if (frontTooClose) currentMode = SPIN_LEFT; 
          // break;
        // case SPIN_LEFT:     //System.out.printf("SPIN LEFT\n");
          // if (!frontTooClose) currentMode = STRAIGHT; 
          // break;
        // case CURVE_LEFT:    //System.out.printf("CURVE LEFT\n");
          // if (!sideTooClose) currentMode = STRAIGHT; 
          // break;
       // }
       
       //if (!(X == goalx) && (Y == goaly) && !sideTooClose && !frontTooClose) {
       
       if (clearedObstacle) {
       //if (true) {
       
          while (robot.step(timeStep) != -1 && (theta > A + 2 || theta < A - 2)) {
          
            
            leftSpeed  = -1 * MAX_SPEED;
            rightSpeed = 1 * MAX_SPEED;
            
            
          
            // System.out.println("angle" + A);
            // System.out.println("theta" + theta);
            currentMode = SPIN_LEFT;
            updateEstimate(previousMode, compass);
            previousMode = currentMode;
            leftMotor.setVelocity(leftSpeed);
            rightMotor.setVelocity(rightSpeed);
            
          
          }
          
         clearedObstacle = false;
       
       
       }
       
       // while (robot.step(timeStep) != -1 && (theta > A + 20 || theta < A - 20)) {
          
            
            // leftSpeed  = -1 * MAX_SPEED;
            // rightSpeed = 1 * MAX_SPEED;
            
            
          
            // System.out.println("angle" + A);
            // System.out.println("theta" + theta);
            // currentMode = SPIN_LEFT;
            // updateEstimate(previousMode, compass);
            // previousMode = currentMode;
            // leftMotor.setVelocity(leftSpeed);
            // rightMotor.setVelocity(rightSpeed);
            
          
          // }
       
       
       if (!sideTooClose && !frontTooClose) {
       
          leftSpeed  = 1 * MAX_SPEED;
          rightSpeed = 1 * MAX_SPEED;
          // System.out.println("IN IF STATMENT");
          currentMode = STRAIGHT;
          updateEstimate(previousMode, compass);
          previousMode = currentMode;
          leftMotor.setVelocity(leftSpeed);
          rightMotor.setVelocity(rightSpeed);
       
       
       } else if (sideTooClose || frontTooClose) {
       
         // System.out.println("IN ELIF STATMENT");
       
         contactx = X;
         contacty = Y;
         double intersect;
         
         do {
         
         
           if ((leftAheadSensor.getValue() > close || rightAheadSensor.getValue() > close || rightAngledSensor.getValue() > close) && (leftAheadSensor.getValue() > superFar && rightAheadSensor.getValue() > superFar && rightAngledSensor.getValue() > superFar)) {
       
             leftSpeed  = -1 * MAX_SPEED;
             rightSpeed = 1 * MAX_SPEED;
             currentMode = SPIN_LEFT;
           
           } else if (rightSideSensor.getValue() < veryFar) {
           
             leftSpeed  = 1 * MAX_SPEED;
             rightSpeed = 0.25 * MAX_SPEED;
             currentMode = PIVOT_RIGHT;
           
           } else if (rightSideSensor.getValue() < far) {
           
             leftSpeed  = 1 * MAX_SPEED;
             rightSpeed = 0.9 * MAX_SPEED;
             currentMode = CURVE_RIGHT;
           
           
           } else if (rightSideSensor.getValue() > close) {
           
             leftSpeed  = 0.9 * MAX_SPEED;
             rightSpeed = 1 * MAX_SPEED;
             currentMode = CURVE_LEFT;
           
           } else {
           
             leftSpeed  = 1 * MAX_SPEED;
             rightSpeed = 1 * MAX_SPEED;
             currentMode = STRAIGHT;
           
           
           }
           
           updateEstimate(previousMode, compass);
           previousMode = currentMode;
           
           leftMotor.setVelocity(leftSpeed);
           rightMotor.setVelocity(rightSpeed);
           
           intersect = Y - slope*X - B;
           
         } while (robot.step(timeStep) != -1 && (intersect > 5 || Math.sqrt((X-goalx)*(X-goalx)+(Y-goaly)*(Y-goaly)) + 10 > Math.sqrt((contactx-goalx)*(contactx-goalx)+(contacty-goaly)*(contacty-goaly))));
         //} while (robot.step(timeStep) != -1 && (intersect > 1 || Math.sqrt((X-goalx)*(X-goalx)+(Y-goaly)*(Y-goaly)) > Math.sqrt((contactx-goalx)*(contactx-goalx)+(contacty-goaly)*(contacty-goaly))) && (X != contactx && Y != contacty));
         // if (X == contactx && Y == contacty) {
         
           // System.out.println("QUIT");
         
         // }
         //System.out.println("MADE IT OUT");
         clearedObstacle = true;
       
       }
       
       
       
       
       
       
       //updateEstimate(previousMode, compass);
       //previousMode = currentMode;

      // REACT: Move motors accordingly
      // switch(currentMode) {
        // case SPIN_LEFT:
          // leftSpeed  = -1 * MAX_SPEED;
          // rightSpeed = 1 * MAX_SPEED;
          // break;
        // case PIVOT_RIGHT:
          // leftSpeed  = 1 * MAX_SPEED;
          // rightSpeed = 0.25 * MAX_SPEED;
          // break;
        // case CURVE_LEFT:
          // leftSpeed  = 0.9 * MAX_SPEED;
          // rightSpeed = 1 * MAX_SPEED;
          // break;
        // case CURVE_RIGHT:
          // leftSpeed  = 1 * MAX_SPEED;
          // rightSpeed = 0.9 * MAX_SPEED;
          // break;
        // case STRAIGHT: 
          // leftSpeed  = 1 * MAX_SPEED;
          // rightSpeed = 1 * MAX_SPEED;
          // break;
      // }
      // leftMotor.setVelocity(leftSpeed);
      // rightMotor.setVelocity(rightSpeed);
    }
    //System.out.println("WHILE LOOP OVER");
    
  }
}
