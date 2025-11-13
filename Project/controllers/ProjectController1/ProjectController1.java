// Author: Devon Robitaille (SN: 101031827) CO-Authors: Morgan White SN: 101184799, Warren Currie SN: 100939912
import java.awt.Point; 
import java.util.ArrayList;
import com.cyberbotics.webots.controller.Camera;
import com.cyberbotics.webots.controller.Accelerometer;
import com.cyberbotics.webots.controller.DistanceSensor;
import com.cyberbotics.webots.controller.TouchSensor;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Compass;
import com.cyberbotics.webots.controller.Robot;

public class ProjectController1 {

  private static final byte    CAMERA_WIDTH = 64;
  private static final byte    CAMERA_HEIGHT = 64;
  private static final double  GRIPPER_MOTOR_MAX_SPEED = 0.1;
  
  static final int MAX_SPEED = 1; // maximum speed of the robot
  
  // Delay for the gripper between different states
  static final int GRIPPERDELAYMOVEMENT = 120;
  
  // When the robot resets itself to look for more jars when are the arcs of travel
  static final int LEFTARC = 95;
  static final int RIGHTARC = 90;
  
  // These values determine how much blue at the finish is acceptable
  static final float min_percent = 0.1f;
  static final float max_percent = 0.2f;
  
  // These values determine how much green and blue pixels are found in the camera
  static final int green_maximum = 50; // max values for red and blue
  static final int green_minimum = 30; // min values for green
  static final int blue_maximum = 20;  // max values for red and green
  static final int blue_minimum = 80;  // min values for blue
   
  // Various modes for the robot to be in
  static final byte    STRAIGHT       = 0;
  static final byte    SPIN_LEFT      = 1;
  static final byte    SPIN_RIGHT     = 2;
  static final byte    REVERSE        = 3;  
  static final byte    GRAB_OBJECT    = 4; // grab jar
  static final byte    RELEASE_OBJECT = 5; // release jar
  static final byte    FINISHED       = 6; // robot has completed its tasks
  static final byte    TRACKING       = 7; // looking for green jar
  static final byte    DELIVERING     = 8; // looking for blue finish  
  static final byte    RESET          = 9; // leave finish area
  static final byte    LIFT_GRIPPER   = 10; // lift gripper
  static final byte    DROP_GRIPPER   = 11; // drop gripper

  // Gripper Claw Modes
  static final float OPEN = 0.099f;
  static final float CLOSE = 0.01f;
  static final float UP = -0.0499f;
  static final float DOWN = 0.001f;
  
  static final int EPSILON = 5;
  
  private static Robot           robot;
  private static Motor           leftMotor;
  private static Motor           rightMotor;
  private static Motor           gripperLift;
  private static Motor           gripperLeftSide;
  private static Motor           gripperRightSide;
  private static DistanceSensor  leftSideSensor; 
  private static DistanceSensor  rightSideSensor;
  private static DistanceSensor  leftAheadSensor; 
  private static DistanceSensor  rightAheadSensor;
  private static DistanceSensor  leftAngledSensor; 
  private static DistanceSensor  rightAngledSensor;
  private static TouchSensor     jarDetectedSensor;
  private static Compass         compass;
  private static Accelerometer   accelerometer;
  private static Camera          camera;
    
    
  // Wait for a certain number of milliseconds
  private static void delay(int milliseconds, int timeStep) {
    int elapsedTime = 0;
    while (elapsedTime < milliseconds) {
      robot.step(timeStep);
      elapsedTime += timeStep;
    }
  }
  
  // Put the gripper up/down to the given position.
  // -0.0499 is "up all the way" and 0.001 is down as mush as it can
  public static void liftLowerGripper(float position) {
    gripperLift.setVelocity(GRIPPER_MOTOR_MAX_SPEED);
    gripperLift.setPosition(position);
  }

  // Put the gripper open/closed to the given position
  // 0.099 is "open all the way" and 0.01 is closed as mush as it can
  public static void openCloseGripper(float position) {
    gripperLeftSide.setVelocity(GRIPPER_MOTOR_MAX_SPEED);
    gripperRightSide.setVelocity(GRIPPER_MOTOR_MAX_SPEED);
    gripperLeftSide.setPosition(position);
    gripperRightSide.setPosition(position);
  }

  
  // This is where it all begins
  public static void main(String[] args) {
    robot = new Robot();
    int timeStep = (int) Math.round(robot.getBasicTimeStep());
    
    // Set up the motors
    leftMotor = robot.getMotor("left wheel");
    rightMotor = robot.getMotor("right wheel");
    gripperLift = robot.getMotor("lift motor");file:///C:/Users/Devon Robitaille/Documents/University/Fall2021/COMP2801-A/Lab 10/Lab10/controllers/Lab10Controller/Lab10Controller.java
    gripperLeftSide = robot.getMotor("left finger motor");
    gripperRightSide = robot.getMotor("right finger motor");
    leftMotor.setPosition(Double.POSITIVE_INFINITY);
    rightMotor.setPosition(Double.POSITIVE_INFINITY);
    leftMotor.setVelocity(0);
    rightMotor.setVelocity(0); 
    
    // Get and enable the distance sensors
    leftSideSensor = robot.getDistanceSensor("so0"); 
    leftAngledSensor = robot.getDistanceSensor("so1"); 
    leftAheadSensor = robot.getDistanceSensor("so3"); 
    rightAheadSensor = robot.getDistanceSensor("so4"); 
    rightAngledSensor = robot.getDistanceSensor("so6"); 
    rightSideSensor = robot.getDistanceSensor("so7"); 
    leftAheadSensor.enable(timeStep);
    rightAheadSensor.enable(timeStep);
    leftAngledSensor.enable(timeStep);
    rightAngledSensor.enable(timeStep);
    leftSideSensor.enable(timeStep);
    rightSideSensor.enable(timeStep);
    
    // Prepare the accelerometer
    accelerometer = new Accelerometer("accelerometer");
    accelerometer.enable(timeStep);
    
    // Prepare the camera
    camera = new Camera("camera");
    camera.enable(timeStep);
    
    // Prepare the Compass sensor
    compass = robot.getCompass("compass");
    compass.enable(timeStep);
    
    // Prepare the jar detecting sensor
    jarDetectedSensor= new TouchSensor("touch sensor");
    jarDetectedSensor.enable(timeStep);
    
    // Run the robot
    byte currentMode  = STRAIGHT;
    byte jarMode = TRACKING;
    double leftSpeed = 0, rightSpeed = 0;
    int deliveriesMade = 0; // should equal 5 at the end    
    int reverseTime = 0, travelTime = 0; // reverseCounter is how long to reverse after delivering jar
    int gripperMovementDelay = 0;
    
    openCloseGripper(OPEN); // open gripper
    liftLowerGripper(DOWN); // lower gripper
    
    while (robot.step(timeStep) != -1) {
      // Read Camera
      int[] image = camera.getImage();
      
      int[][] r = new int[CAMERA_WIDTH][CAMERA_HEIGHT];
      int[][] g = new int[CAMERA_WIDTH][CAMERA_HEIGHT];
      int[][] b = new int[CAMERA_WIDTH][CAMERA_HEIGHT];
      
      for (int i = 0; i < CAMERA_WIDTH; i++) {
        for (int j = -CAMERA_HEIGHT/2; j < CAMERA_HEIGHT/2; j++) {
          r[i][j+CAMERA_HEIGHT/2] = Camera.imageGetRed(image, CAMERA_WIDTH, i, CAMERA_HEIGHT/2+j);
          g[i][j+CAMERA_HEIGHT/2] = Camera.imageGetGreen(image, CAMERA_WIDTH, i, CAMERA_HEIGHT/2+j);
          b[i][j+CAMERA_HEIGHT/2] = Camera.imageGetBlue(image, CAMERA_WIDTH, i, CAMERA_HEIGHT/2+j);
        }
      }
      
      // [0][0] = green left side
      // [0][1] = green front side
      // [0][2] = green right side
      // [1][0] = blue left side
      // [1][1] = blue front side
      // [1][2] = blue right side
      int[][] colorCount = countColor(r, g, b);
      
      // Find object
    
      // SENSE: Read the sensors
      boolean objectLeft    = colorCount[0][0] > colorCount[0][1] + EPSILON && colorCount[0][0] > colorCount[0][2] + EPSILON;
      boolean objectCenter  = colorCount[0][1] > colorCount[0][0] && colorCount[0][1] > colorCount[0][2];
      boolean objectRight   = colorCount[0][2] > colorCount[0][0] + EPSILON && colorCount[0][2] > colorCount[0][1] + EPSILON;
      
      boolean dropoffLeft    = colorCount[1][0] > colorCount[1][1] + EPSILON && colorCount[1][0] > colorCount[1][2] + EPSILON;
      boolean dropoffCenter  = colorCount[1][1] > colorCount[1][0] && colorCount[1][1] > colorCount[1][2];
      boolean dropoffRight   = colorCount[1][2] > colorCount[1][0] + EPSILON && colorCount[1][2] > colorCount[1][1] + EPSILON;
      
      boolean inContact = jarDetectedSensor.getValue() == 1.0f;
      
      if (deliveriesMade == 5) currentMode = FINISHED; // the robot as made 5 jar deliveries
      
      int bearing = getCompassReadingInDegrees(compass);
      int totalBlue = colorCount[1][0] + colorCount[1][1] + colorCount[1][2];
      
      // if (jarMode == DELIVERING && !inContact) {
        // jarMode = TRACKING;
        // openCloseGripper(0.099f); //move to open state
      // }
      
      // THINK: Make a decision as to what MODE to be in
      switch(currentMode) {
        case STRAIGHT:
          switch(jarMode) {
            case TRACKING:
              if (objectLeft) currentMode = SPIN_RIGHT;
              else if (objectRight) currentMode = SPIN_LEFT;
              else if (!objectLeft && !objectRight && !objectCenter) currentMode = SPIN_LEFT;
              // when near jar, grab jar
              if (inContact) {
                currentMode = GRAB_OBJECT;
                gripperMovementDelay = GRIPPERDELAYMOVEMENT;
              }
              break;
              
            case DELIVERING:
              if (dropoffLeft) currentMode = SPIN_RIGHT;
              else if (dropoffRight) currentMode = SPIN_LEFT;
              else if (!dropoffLeft && !dropoffRight && !dropoffCenter) currentMode = SPIN_LEFT;
              break;
          }
          break;
          
        case SPIN_LEFT:   
        case SPIN_RIGHT:
          switch(jarMode) {          
            case TRACKING:
                if (objectCenter) currentMode = STRAIGHT;
                break;              
              case DELIVERING:
                if (dropoffCenter) currentMode = STRAIGHT;
                break;
          }
          break;
          
        //-------------------------//
        case GRAB_OBJECT:
          if (gripperMovementDelay == 0) {
            System.out.println("GRAB OBJECT COMPLETE");
            currentMode = LIFT_GRIPPER;
            gripperMovementDelay = GRIPPERDELAYMOVEMENT;
          }
          break;
          
        case LIFT_GRIPPER:
          if (gripperMovementDelay == 0) {
            System.out.println("LIFT GRIPPER COMPLETE");
            currentMode = STRAIGHT;
            jarMode = DELIVERING;
          }
          break;
        //-------------------------//      
        
        //-------------------------//    
        case DROP_GRIPPER:
          if (gripperMovementDelay == 0) {
            System.out.println("DROP GRIPPER COMPLETE");
            currentMode = RELEASE_OBJECT;
            gripperMovementDelay = GRIPPERDELAYMOVEMENT;
          }
          break;
          
        case RELEASE_OBJECT:
          if (gripperMovementDelay == 0) {
            System.out.println("RELEASE OBJECT COMPLETE");
            currentMode = RESET;
            jarMode = TRACKING;
            reverseTime = 500;
            travelTime = 500;
            deliveriesMade++;
          }
          break;
        //-------------------------//

        case FINISHED:
          break;
          
        case RESET:
          if (reverseTime == 0 && (getCompassReadingInDegrees(compass) <= LEFTARC && getCompassReadingInDegrees(compass) >= RIGHTARC) && travelTime == 0) currentMode = STRAIGHT;
          break;
      }
      
      if (checkDoorway(jarMode, bearing, blueFinish(r, g, b, max_percent, min_percent))) {
        System.out.println("---AT DOORWAY---");
        currentMode = DROP_GRIPPER; // At finish line
        gripperMovementDelay = GRIPPERDELAYMOVEMENT;
        jarMode = TRACKING;
      }
           
      // REACT: Move motors according to the MODE
      switch(currentMode) {
        case STRAIGHT:
          leftSpeed  = 1 * MAX_SPEED;
          rightSpeed = 1 * MAX_SPEED;
          break;
          
        case SPIN_LEFT:
          leftSpeed  = -1 * MAX_SPEED;
          rightSpeed = 1 * MAX_SPEED;
          break;
          
        case SPIN_RIGHT:
          leftSpeed  = 1 * MAX_SPEED;
          rightSpeed = -1 * MAX_SPEED;
          break;
          
        case REVERSE:
          leftSpeed  = -1 * MAX_SPEED;
          rightSpeed = -1 * MAX_SPEED;
          break;
          
        case GRAB_OBJECT:  
          openCloseGripper(CLOSE); // close gripper
          leftSpeed  = 0 * MAX_SPEED;
          rightSpeed = 0 * MAX_SPEED;
          gripperMovementDelay--;
          break;  
          
        case LIFT_GRIPPER:
          liftLowerGripper(UP);
          leftSpeed  = 0 * MAX_SPEED;
          rightSpeed = 0 * MAX_SPEED;
          gripperMovementDelay--;
          break; 
          
        case RELEASE_OBJECT:  
          openCloseGripper(OPEN); // open gripper
          leftSpeed  = 0 * MAX_SPEED;
          rightSpeed = 0 * MAX_SPEED;
          gripperMovementDelay--;
          break;   
          
        case DROP_GRIPPER:
          liftLowerGripper(DOWN);
          leftSpeed  = 0 * MAX_SPEED;
          rightSpeed = 0 * MAX_SPEED;
          gripperMovementDelay--;
          
        case FINISHED:
          leftSpeed  = 0 * MAX_SPEED;
          rightSpeed = 0 * MAX_SPEED;
          break;
          
        case RESET:
          if (reverseTime > 0) {
            leftSpeed  = -1 * MAX_SPEED;
            rightSpeed = -1 * MAX_SPEED;
            reverseTime--;
          }
          else if (getCompassReadingInDegrees(compass) > LEFTARC || getCompassReadingInDegrees(compass) < RIGHTARC ) {
            leftSpeed  = -1 * MAX_SPEED;
            rightSpeed = 1 * MAX_SPEED;
          } else {
            leftSpeed  = 1 * MAX_SPEED;
            rightSpeed = 1 * MAX_SPEED;
            travelTime--;
          }
          break;
      }
      
      leftMotor.setVelocity(leftSpeed);
      rightMotor.setVelocity(rightSpeed);
    }
  }
  
  /*  
    Count the total number of pixels on the screen
  */
  static private int[][] countColor(int[][] r, int[][] g, int[][] b) {
    int[][] results = new int[2][3];
    // 0 => <=CAMERA_WIDTH/3
    // 1 => CAMERA_WIDTH/3 => CAMERA_WIDTH - CAMERA_WIDTH/3
    // 2 => >= CAMERA_WIDTH - CAMERA_WIDTH/3
    
    // left side
    results[0][0] = 0; // green
    results[1][0] = 0; // blue
    // center
    results[0][1] = 0; // green
    results[1][1] = 0; // blue
    // right side
    results[0][2] = 0; // green
    results[1][2] = 0; // blue
    
    for (int j = 0; j < CAMERA_HEIGHT; j++) {
      for (int i = 0; i < (int)Math.floor(CAMERA_WIDTH/3); i++) {
        if (r[i][j]<green_maximum && g[i][j]>green_minimum && b[i][j]<green_maximum) results[0][0]++;
        if (r[i][j]<blue_maximum && g[i][j]<blue_maximum && b[i][j]>blue_minimum)    results[1][0]++;
      }
      
      for (int i = (int)Math.floor(CAMERA_WIDTH/3); i < CAMERA_WIDTH-(int)Math.floor(CAMERA_WIDTH/3); i++) {
        if (r[i][j]<green_maximum && g[i][j]>green_minimum && b[i][j]<green_maximum) results[0][1]++;
        if (r[i][j]<blue_maximum && g[i][j]<blue_maximum && b[i][j]>blue_minimum)    results[1][1]++;
      }
      
      for (int i = CAMERA_WIDTH-(int)Math.floor(CAMERA_WIDTH/3); i < CAMERA_WIDTH; i++) {
        if (r[i][j]<green_maximum && g[i][j]>green_minimum && b[i][j]<green_maximum) results[0][2]++;
        if (r[i][j]<blue_maximum && g[i][j]<blue_maximum && b[i][j]>blue_minimum)    results[1][2]++;
      }
    }

    
    return results;
  }
  
  /*  
    Convert the compass to be between -180 and 180
  */
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
  
  private static boolean checkDoorway(byte jarMode, int bearing, boolean totalBlue) {
    return (jarMode == DELIVERING && bearing < 5 && bearing > -5 && totalBlue);
  }
  
  /*  
    Is the robot seeing the appropriate number of blue pixels in the top quarter of the camera?
  */
  private static boolean blueFinish(int[][] r, int[][] g, int[][] b, float max_percent, float min_percent) {
    int totalCount = 0;
    
    int blue_maximum = 20;
    int blue_minimum = 80;
    
    for (int j = 0; j < CAMERA_HEIGHT/4; j++) {
      for (int i = 0; i < CAMERA_WIDTH; i++) {
        if (r[i][j]<blue_maximum && g[i][j]<blue_maximum && b[i][j]>blue_minimum) totalCount++;
      }
    }
    
    int totalPixels = CAMERA_WIDTH * (CAMERA_HEIGHT/4);
    int maxPercentTotalPixels = (int)Math.floor(totalPixels * max_percent);
    int minPercentTotalPixels = (int)Math.floor(totalPixels * min_percent);
    
    return (totalCount < maxPercentTotalPixels && totalCount < minPercentTotalPixels);
  }
}