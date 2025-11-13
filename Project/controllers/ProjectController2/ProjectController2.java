// Author: Warren Currie SN: 100939912 CO-Authors: Morgan White SN: 101184799, Devon Robitaille (SN: 101031827)

import java.awt.Point; 
import java.util.ArrayList;
import com.cyberbotics.webots.controller.Camera;
import com.cyberbotics.webots.controller.Accelerometer;
import com.cyberbotics.webots.controller.DistanceSensor;
import com.cyberbotics.webots.controller.TouchSensor;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Compass;
import com.cyberbotics.webots.controller.Robot;
import com.cyberbotics.webots.controller.Lidar;

// For Testing
//import com.cyberbotics.webots.controller.Display;
//import com.cyberbotics.webots.controller.Supervisor;

public class ProjectController2 {

  private static final byte    CAMERA_WIDTH = 64;
  private static final byte    CAMERA_HEIGHT = 64;
  private static final float   ADJACENCY_TOLERANCE = 0.15f; // 15cm to be considered on same wall
  private static final float   TOO_CLOSE_DISTANCE = 0.52f; // 50cm to be considered too close
  private static final float   TOO_CLOSE_SIDE_DISTANCE = 0.30f; // 25cm to be considered too close
  private static final double  GRIPPER_MOTOR_MAX_SPEED = 0.1;
  private static final int     MAX_SPEED = 12;
  
  private static final int     WORLD_WIDTH  = 2000;
  private static final int     WORLD_HEIGHT = 2000;
  
  // Various modes for the robot to be in
  static final byte WAITING_FOR_JAR  = 0;
  static final byte MOVING_TO_SQUARE  = 1;
  static final byte DROP_OFF = 2;
  static final byte PICK_UP = 3;
  static final byte MOVE_TO_DROP_OFF = 4;
  static final byte WAIT = 5;
  static final byte ORIENTATE = 6;
  static final byte REVERSE = 7;
  
  // Square to Square Travel Flags
  static final byte SENDING = 0;
  static final byte RETURNING = 1;
  
  // Gripper Claw modes
  static final float OPEN = 0.099f;
  static final float CLOSE = 0.01f;
  static final float UP = -0.0499f;
  static final float DOWN = 0.001f;
  
  // Robot Movement
  static final byte LEFT = 0;
  static final byte RIGHT = 1;
  static final byte NONE = 2;
  
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
  
  // The app for displaying the lidar data for testing
  //private static LidarDisplayApp  displayApp;
  
  // Wait for a certain number of milliseconds
  private static void delay(int milliseconds, int timeStep) {
    int elapsedTime = 0;
    while (elapsedTime < milliseconds) {
      robot.step(timeStep);
      elapsedTime += timeStep;
    }
  }
  
  // Put the gripper up/down to the given position
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
  
  // Read the compass
  private static int getCompassReadingInDegrees() {
    double compassReadings[] = compass.getValues();
    double rad = Math.atan2(compassReadings[0], compassReadings[2]);
    double bearing = -((rad + Math.PI) / Math.PI * 180.0);
    if (bearing > 180)
      bearing = 360 - bearing;
    if (bearing < -180)
      bearing = 360 + bearing;
    return (int)(bearing);
  }
  
  // This method turns away
  private static void turnAway(int leftSpeed, int rightSpeed) {
    leftMotor.setVelocity(leftSpeed);
    rightMotor.setVelocity(rightSpeed);
  }
  
  // This method moves from point (x1,y1) to point (x2, y2) with a slight curve
  private static void moveFrom(double x1, double y1, double x2, double y2) {
    double xDiff = x2 - x1;
    double yDiff = y2 - y1;
    double turn = Math.atan2(yDiff, xDiff) * 180 / Math.PI;
    turn = (turn - getCompassReadingInDegrees()) % 360;
    if (turn < -180)
      turn += 360;
    else if (turn > 180)
      turn -= 360;

    if (turn > 0) {
      leftMotor.setVelocity((int)(MAX_SPEED*((90-turn)/90.0)*0.85));
      rightMotor.setVelocity(MAX_SPEED);
    }
    else {//if (turn < -10) {
      leftMotor.setVelocity(MAX_SPEED);
      rightMotor.setVelocity((int)(MAX_SPEED*((90+turn)/90.0)*0.85));
    }
  }
  
  private static void moveToCenter(int x, int y, float lidarValues[]) {
      // Get the actual robot location
      //double values[] = TranslationField.getSFVec3f();
      //displayApp.applyLidarReading(lidarValues, x, y, getCompassReadingInDegrees());
      
      Point doorStart = null;
      Point doorEnd = null;
      
      int a0 =  getCompassReadingInDegrees() + 90;
      if (a0 < 0) a0 += 360;
      else if (a0 > 359) a0 -= 360;
      
      double centerX = 100*lidarValues[0] * Math.cos(Math.toRadians(a0));
      double centerY = 100*lidarValues[0] * Math.sin(Math.toRadians(a0));
      
      for (int i=1;i<180;i++) {
        // Angle for door start
        int a = getCompassReadingInDegrees() + 90 - i;
        if (a < 0) a += 360;
        else if (a > 359) a -= 360;
        
        // Angle for door end
        int a2 = getCompassReadingInDegrees() + 90 - (180 -i);
        if (a2 < 0) a2 += 360;
        else if (a2 > 359) a2 -= 360;
        
        // If door start has not been found, check
        if (doorStart == null && Math.abs(lidarValues[i] - lidarValues[i-1]) > ADJACENCY_TOLERANCE) {
          double px1 = 100*lidarValues[i-1] * Math.cos(Math.toRadians(a));
          double py1 = 100*lidarValues[i-1] * Math.sin(Math.toRadians(a));
          doorStart = new Point((int)px1, (int)py1);
        }
        // If door end has not been found, check
        if (doorEnd == null && Math.abs(lidarValues[180-i] - lidarValues[180-i-1]) > ADJACENCY_TOLERANCE) {
          double px2 = 100*lidarValues[180-i] * Math.cos(Math.toRadians(a2));
          double py2 = 100*lidarValues[180-i] * Math.sin(Math.toRadians(a2));
          doorEnd = new Point((int)px2, (int)py2);
        }
        
        // Add current reading value to center
        centerX += 100*lidarValues[i] * Math.cos(Math.toRadians(a));
        centerY += 100*lidarValues[i] * Math.sin(Math.toRadians(a));

        }
        /*
        if (doorStart != null && doorEnd != null) {
        //System.out.println(doorStart + " " + doorEnd);
        double doorWidth = java.awt.geom.Point2D.distance(doorStart.x,doorStart.y, doorEnd.x,doorEnd.y);
        if (doorWidth >= 80 && doorWidth <= 100) { 
	displayApp.setDoorwayPoints(doorStart.x,doorStart.y, doorEnd.x,doorEnd.y, x, y);
        } else displayApp.setDoorwayPoints(0,0, 0,0, x, y);
        } else displayApp.setDoorwayPoints(0,0, 0,0, x, y);
        */
        centerX = centerX/180 + x;
        centerY = centerY/180 + y;
		
        //displayApp.setWayPoint((int)centerX, (int)centerY, x, y);
		
        if(lidarValues[0] < TOO_CLOSE_SIDE_DISTANCE) {
          turnAway(MAX_SPEED, MAX_SPEED/2);
          System.out.println("Too Close Right");
        }else if (lidarValues[179] < TOO_CLOSE_SIDE_DISTANCE) {
          turnAway(MAX_SPEED/2, MAX_SPEED);
          System.out.println("Too Close Left");
        }else if (lidarValues[89] < TOO_CLOSE_DISTANCE) {
          turnAway(MAX_SPEED/2, -MAX_SPEED/2);
          System.out.println("Too Close Front");
        }else {
          moveFrom(x, y, centerX, centerY);
        }
  }
  
  // This is where it all begins
  public static void main(String[] args) {
    robot = new Robot();
    //robot = new Supervisor();
    int timeStep = (int) Math.round(robot.getBasicTimeStep());
    
    // Display app for testing
    //displayApp = new LidarDisplayApp(WORLD_WIDTH/2, WORLD_HEIGHT/2, robot.getDisplay("display"));
	
    // Set up the motors
    leftMotor = robot.getMotor("left wheel");
    rightMotor = robot.getMotor("right wheel");
    gripperLift = robot.getMotor("lift motor");
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
    
    //  Prepare the Lidar sensor
    Lidar lidar = new Lidar("Sick LMS 291");
    lidar.enable(timeStep);
	
    float lidarValues[] = null;
    
    int [] image;
    int blueCount, blueCountRight, blueCountCenter, greenCountLeft, greenCountRight, greenCountCenter, r,b,g;  
    
    // Run the robot
    byte currentMode = MOVING_TO_SQUARE;
    //byte currentMode = WAIT;
    byte currentDir = NONE;
    byte currentTravel = RETURNING;
    
    int count = 0;
    int jars = 5;
    
    openCloseGripper(OPEN);
    liftLowerGripper(DOWN);
    
    while (robot.step(timeStep) != -1) {
      // SENSE: Read the sensors
      
      // Get Lidar Values for distance checking
      lidarValues = lidar.getRangeImage();
      //System.out.println(lidarValues[29] + ", " + lidarValues[89] + ", " +  lidarValues[149]);
      
      // Check if in contact with a jar
      boolean inContact = jarDetectedSensor.getValue() == 1.0f;
      
      // Blue/Green Values in 1st/2nd/3d thirds of the camera
      image = camera.getImage();
      blueCount = blueCountRight = blueCountCenter = greenCountLeft = greenCountRight = greenCountCenter = 0;

      // THINK: Make a decision as to what MODE to be in
      switch(currentMode) {
        case WAITING_FOR_JAR:
            // Jar found, no need to calculate RGB values
            if(inContact) {
              currentMode = PICK_UP;
              count = 100;
              break;
            }
            
            // Check Camera to scan for blue/green values for squar/jar dectection
            for (int i = 0; i < 64; i++){
                for (int j = 0; j < 64; j++){
                  r = Camera.imageGetRed(image, 64, i, j);
                  g = Camera.imageGetGreen(image, 64, i, j);
                  b = Camera.imageGetBlue(image, 64, i, j);
                      
                  // Check the current Green Values
                  if (r < 60 && g > 80 && b < 60){
                    if (i < 24) greenCountLeft++;
                    else if (i < 40) greenCountCenter++;
                    else greenCountRight++;
                  }
              }
            }
            
            if (greenCountLeft > (greenCountRight + 5) && greenCountLeft > greenCountCenter) currentDir = LEFT;
            else if (greenCountRight > (greenCountLeft + 5) && greenCountRight > greenCountCenter) currentDir = RIGHT;
            else if (greenCountCenter > 5) currentDir = NONE;
            else currentDir = RIGHT; // Spin right if no jar in view
            //System.out.println(greenCountLeft + ", " + greenCountCenter + ", " + greenCountRight);
            
            break;
        
        case MOVING_TO_SQUARE:
            // Check Camera to scan for blue/green values for squar/jar dectection
            for (int i = 0; i < 64; i++){
                for (int j = 0; j < 64; j++){
                  r = Camera.imageGetRed(image, 64, i, j);
                  g = Camera.imageGetGreen(image, 64, i, j);
                  b = Camera.imageGetBlue(image, 64, i, j);
                      
                  // Check the current Green Values
                  if (r < 60 && g < 60 && b > 120){
                    blueCount++;
                  }
              }
            }
            // System.out.println(blueCount);
            if (blueCount > 800) {
              if (currentTravel == RETURNING) currentMode = WAITING_FOR_JAR;
              if (currentTravel == SENDING) currentMode = MOVE_TO_DROP_OFF;
            }
            break;
        
        case MOVE_TO_DROP_OFF:
            if(lidarValues[29] > 3 && lidarValues[89] > 7 && lidarValues[149] > 1) {
              currentMode = DROP_OFF;
              count = 100;
            }
            break;
        case DROP_OFF:
            count--;
            if(count <= 0) {
              count = 60;
              jars--;
              currentMode = REVERSE;
              currentTravel = RETURNING;
              currentDir = LEFT;
            }
            break;
        case PICK_UP:
            count--;
            if(count <= 0) {
              currentMode = REVERSE;
              currentTravel = SENDING;
              currentDir = RIGHT;
              count = 60;
            }
            break;
        case ORIENTATE:
            count--;
            if (count <= 0){
              currentMode = MOVING_TO_SQUARE;
            }
            break;
        case REVERSE:
            count--;
            if (count <= 0){
              if (jars == 0) System.exit(0);
              currentMode = ORIENTATE;
              count = 40;
            }
            break;     
      }
           
      // REACT: Move motors according to the MODE
      switch(currentMode) {
        case WAITING_FOR_JAR: //System.out.println("Robot 2: WAITING_FOR_JAR");
            if (currentDir == NONE) {leftMotor.setVelocity(MAX_SPEED/4); rightMotor.setVelocity(MAX_SPEED/4);}
            else if (currentDir == LEFT) {leftMotor.setVelocity(-MAX_SPEED/4); rightMotor.setVelocity(MAX_SPEED/4);}
            else if (currentDir == RIGHT) {leftMotor.setVelocity(MAX_SPEED/4); rightMotor.setVelocity(-MAX_SPEED/4);}
            
            break;
          
        case MOVING_TO_SQUARE: //System.out.println("Robot 2: MOVING_TO_SQUARE");
            //double values[] = TranslationField.getSFVec3f();
            //int x = (int)(values[0]*100) + WORLD_WIDTH/2;
            //int y = -(int)(values[2]*100) + WORLD_HEIGHT/2;
            if (currentTravel == SENDING) openCloseGripper(CLOSE);
            int x = 0;
            int y = 0;
            
            moveToCenter(x, y, lidarValues);
            break;
          
        case DROP_OFF: //System.out.println("Robot 2: DROP_OFF");
            if (count == 100) liftLowerGripper(DOWN);
            if (count == 50) openCloseGripper(OPEN);
            leftMotor.setVelocity(0);
            rightMotor.setVelocity(0);
	 break;
        case PICK_UP:
            if (count == 100) openCloseGripper(CLOSE);
            else if (count == 50) liftLowerGripper(UP);
            leftMotor.setVelocity(MAX_SPEED/16);
            rightMotor.setVelocity(MAX_SPEED/16);
            break;
        case MOVE_TO_DROP_OFF:  
            moveToCenter(0,0, lidarValues);
            break;
        case ORIENTATE:
            if (currentDir == LEFT) {leftMotor.setVelocity(-MAX_SPEED/4); rightMotor.setVelocity(MAX_SPEED/4);}
            else if (currentDir == RIGHT) {leftMotor.setVelocity(MAX_SPEED/4); rightMotor.setVelocity(-MAX_SPEED/4);}
            break;
        case WAIT:
            leftMotor.setVelocity(0);
            rightMotor.setVelocity(0);
            break;
        case REVERSE:
            leftMotor.setVelocity(-MAX_SPEED/2);
            rightMotor.setVelocity(-MAX_SPEED/2);
            break;
      }
    }
  }
}
