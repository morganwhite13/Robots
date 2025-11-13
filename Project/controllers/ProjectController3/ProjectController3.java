// Author: Morgan White SN: 101184799 CO-Authors: Devon Robitaille (SN: 101031827), Warren Currie SN: 100939912
import java.awt.Point; 
import java.util.ArrayList;
import com.cyberbotics.webots.controller.Camera;
import com.cyberbotics.webots.controller.DistanceSensor;
import com.cyberbotics.webots.controller.TouchSensor;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Compass;
import com.cyberbotics.webots.controller.Robot;
import com.cyberbotics.webots.controller.Field;
import com.cyberbotics.webots.controller.Node;
import com.cyberbotics.webots.controller.Supervisor;

public class ProjectController3 {

  private static final byte    CAMERA_WIDTH = 64;
  private static final byte    CAMERA_HEIGHT = 64;
  private static final double  GRIPPER_MOTOR_MAX_SPEED = 0.1;
  
  private static double  MAX_SPEED = 5;//ADDED BY ME MIGHT havE TO REMOVE
  static int WORLD_WIDTH = 1000;
  static int WORLD_HEIGHT = 500;//MAYBE REMOVE
  static final int EPSILON = 15;
  static final int HEIGHT_CAPTURED = CAMERA_HEIGHT;
  static int jarsToGo = 4;
  
  
  // Various modes for the robot to be in
  static final byte AAAA  = 0;
  static final byte BBBB  = 1;
  static final byte CCCC  = 2;
    
  private static Supervisor      robot;
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
  private static Camera          camera;
  private static Field           TranslationField;
  
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

  // This is where it all begins
  public static void main(String[] args) {
    robot = new Supervisor();
    
    // Code required for being able to get the robot's location
    Node    robotNode = robot.getSelf();
    TranslationField = robotNode.getField("translation");
    
    int timeStep = (int) Math.round(robot.getBasicTimeStep());
    
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
    
    // Prepare the camera
    camera = new Camera("camera");
    camera.enable(timeStep);
    
    // Prepare the Compass sensor
    compass = robot.getCompass("compass");
    compass.enable(timeStep);
    
    // Prepare the jar detecting sensor
    jarDetectedSensor= new TouchSensor("touch sensor");
    jarDetectedSensor.enable(timeStep);
    
    
    
    
    
    hardcodedPathPushing();
    
    
    int holding = 100;
    
    while (holding > 50 && robot.step(timeStep) != -1) {
    
    
      openCloseGripper(0.099f);
      liftLowerGripper(0.001f);
      holding--;
    
    }
    
    findJar();
    
    
    toStart();
    
    placeJar();
    
    while (jarsToGo >= 0) {
    
      hardcodedPath();
      findJar();
    
      toStart();
      
      placeJar();
      
      
    
    }
    
    
    System.exit(0);
    
    
    
  }
  
  
  
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
  
  
  
  
  static private int[][] countColor(int[][] r, int[][] g, int[][] b) {
    int[][] results = new int[2][3];
    // 0 => <=CAMERA_WIDTH/3
    // 1 => CAMERA_WIDTH/3 => CAMERA_WIDTH - CAMERA_WIDTH/3
    // 2 => >= CAMERA_WIDTH - CAMERA_WIDTH/3
    
    int green_maximum = 80;
    int green_minimum = 50;
    
    int blue_maximum = 20;
    int blue_minimum = 80;
    
    // left side
    results[0][0] = 0; // green
    results[1][0] = 0; // blue
    // center
    results[0][1] = 0; // green
    results[1][1] = 0; // blue
    // right side
    results[0][2] = 0; // green
    results[1][2] = 0; // blue
    
    for (int j = 0; j < HEIGHT_CAPTURED; j++) {
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
  
  
  
  
  private static void findJar() {
  
  
    
  
  
    int timeStep = (int) Math.round(robot.getBasicTimeStep());
    
    int holding = 100;
    
    holding = 100;
    int seenGreen = -100;
    
    
    double values[] = TranslationField.getSFVec3f();
    double y = -(values[2]*100);
    
    
    while (robot.step(timeStep) != -1 && holding > -50) {
    
      // Read Camera
      int[] image = camera.getImage();
      
      int[][] r = new int[CAMERA_WIDTH][HEIGHT_CAPTURED];
      int[][] g = new int[CAMERA_WIDTH][HEIGHT_CAPTURED];
      int[][] b = new int[CAMERA_WIDTH][HEIGHT_CAPTURED];
      
      for (int i = 0; i < CAMERA_WIDTH; i++) {
        for (int j = -HEIGHT_CAPTURED/2; j < HEIGHT_CAPTURED/2; j++) {
          r[i][j+HEIGHT_CAPTURED/2] = Camera.imageGetRed(image, CAMERA_WIDTH, i, CAMERA_HEIGHT/2+j);
          g[i][j+HEIGHT_CAPTURED/2] = Camera.imageGetGreen(image, CAMERA_WIDTH, i, CAMERA_HEIGHT/2+j);
          b[i][j+HEIGHT_CAPTURED/2] = Camera.imageGetBlue(image, CAMERA_WIDTH, i, CAMERA_HEIGHT/2+j);
        }
      }
      
      // [0][0] = green left side
      // [0][1] = green front side
      // [0][2] = green right side
      // [1][0] = blue left side
      // [1][1] = blue front side
      // [1][2] = blue right side
      int[][] colorCount = countColor(r, g, b);
      
      // System.out.println(image);
      
      // Find object
    
      // SENSE: Read the sensors
      // boolean objectLeft    = colorCount[0][0] > colorCount[0][1] + EPSILON && colorCount[0][0] > colorCount[0][2] + EPSILON;
      // boolean objectCenter  = colorCount[0][1] > colorCount[0][0] + EPSILON && colorCount[0][1] > colorCount[0][2] + EPSILON;
      // boolean objectRight   = colorCount[0][2] > colorCount[0][0] + EPSILON && colorCount[0][2] > colorCount[0][1] + EPSILON;
      
      // int maxDist = 10;
      // boolean objectLeft    = colorCount[0][0] > colorCount[0][1] + EPSILON && colorCount[0][0] > colorCount[0][2] + EPSILON && leftAngledSensor.getValue() < maxDist;
      // boolean objectCenter  = colorCount[0][1] > colorCount[0][0] + EPSILON && colorCount[0][1] > colorCount[0][2] + EPSILON && rightAheadSensor.getValue() < maxDist;
      // boolean objectRight   = colorCount[0][2] > colorCount[0][0] + EPSILON && colorCount[0][2] > colorCount[0][1] + EPSILON && rightAngledSensor.getValue() < maxDist;
      
      boolean objectLeft    = colorCount[0][0] > colorCount[0][1] + EPSILON && colorCount[0][0] > colorCount[0][2] + EPSILON;
      boolean objectCenter  = colorCount[0][1] > colorCount[0][0] + EPSILON && colorCount[0][1] > colorCount[0][2] + EPSILON;
      boolean objectRight   = colorCount[0][2] > colorCount[0][0] + EPSILON && colorCount[0][2] > colorCount[0][1] + EPSILON;
      if (!objectLeft && !objectCenter && !objectRight) {
        int enoughGreen = 10;
      
        if (colorCount[0][0] > colorCount[0][1] && colorCount[0][0] > colorCount[0][2] && colorCount[0][0] > enoughGreen) {
        
          objectLeft = true;
        
        } else if (colorCount[0][1] > colorCount[0][0] && colorCount[0][1] > colorCount[0][2] && colorCount[0][1] > enoughGreen) {
        
          objectCenter = true;
        
        } else if (colorCount[0][2] > colorCount[0][0] && colorCount[0][2] > colorCount[0][1] && colorCount[0][2] > enoughGreen) {
        
          objectRight = true;
        
        } else if (colorCount[0][1] > enoughGreen*2) {
        
          objectCenter = true;
        
        }
      
      }
      
      values = TranslationField.getSFVec3f();
      y = -(values[2]*100);
      
      boolean dropoffLeft    = colorCount[1][0] > colorCount[1][1] + EPSILON && colorCount[1][0] > colorCount[1][2] + EPSILON;
      boolean dropoffCenter  = colorCount[1][1] > colorCount[1][0] + EPSILON && colorCount[1][1] > colorCount[1][2] + EPSILON;
      boolean dropoffRight   = colorCount[1][2] > colorCount[1][0] + EPSILON && colorCount[1][2] > colorCount[1][1] + EPSILON;
      
      boolean inContact = jarDetectedSensor.getValue() == 1.0f;
      
      
      
      int compassAng = getCompassReadingInDegrees(compass);
      // System.out.println("ANG IS " + compassAng);
      boolean inRange = compassAng < -10 && compassAng > -170;
      
      
      if (y < -99990) {
      
        // System.out.println("Crossed BOundary ");
      
      } else if (inRange) {
      
      
        if (inContact || holding < 100) {
        
          // System.out.println("COLOR THING IS " + colorCount[0][0] + " XX " + colorCount[0][1] + " YYY " + colorCount[0][2]);
        
          if (seenGreen > 100) {
            // leftMotor.setVelocity(MAX_SPEED*(holding/100));
            // rightMotor.setVelocity(MAX_SPEED*(holding/100));
            // leftMotor.setVelocity(MAX_SPEED);
            // rightMotor.setVelocity(MAX_SPEED);
            // leftMotor.setVelocity(MAX_SPEED/4);
            // rightMotor.setVelocity(MAX_SPEED/4);
            leftMotor.setVelocity(0);
            rightMotor.setVelocity(0);
              // leftMotor.setVelocity(MAX_SPEED/40);
              // rightMotor.setVelocity(MAX_SPEED/40);
          
            if (holding < 75) {
            
              // leftMotor.setVelocity(MAX_SPEED/2);
              // rightMotor.setVelocity(MAX_SPEED/2);
              
              leftMotor.setVelocity(0);
              rightMotor.setVelocity(0);
              // leftMotor.setVelocity(MAX_SPEED*(holding/100));
              // rightMotor.setVelocity(MAX_SPEED*(holding/100));
              openCloseGripper(0.01f);
              
            }
            
            holding -= 1;
          // holding = 0;
          // break;
          } else {
          
            leftMotor.setVelocity(0);
            rightMotor.setVelocity(0);
            seenGreen++;
          
          
          }
          
        
        } else if (objectLeft) {//TURN LEFT
        
          if (seenGreen > 100) {
            leftMotor.setVelocity(-MAX_SPEED/4);
            rightMotor.setVelocity(MAX_SPEED/4);
          } else {
          
            
            leftMotor.setVelocity(0);
            rightMotor.setVelocity(0);
          
          }
          seenGreen++;
        
        } else if (objectRight) {//TURN Right
        
        
          if (seenGreen > 100) {
            leftMotor.setVelocity(MAX_SPEED/4);
            rightMotor.setVelocity(-MAX_SPEED/4);
          } else {
          
            
            leftMotor.setVelocity(0);
            rightMotor.setVelocity(0);
          
          }
          seenGreen++;
        
        } else if (objectCenter) {//go forwards
        
          if (seenGreen > 100) {
            leftMotor.setVelocity(MAX_SPEED/4);
            rightMotor.setVelocity(MAX_SPEED/4);
          } else {
          
            
            leftMotor.setVelocity(0);
            rightMotor.setVelocity(0);
          
          }
          
          seenGreen++;
        
        } else {//just spin waiting to see it
      
          leftMotor.setVelocity(-MAX_SPEED/2);
          rightMotor.setVelocity(MAX_SPEED/2);
          // seenGreen = 0;
      
        }
        
      } else {//just spin waiting to see it
      
        leftMotor.setVelocity(-MAX_SPEED/2);
        rightMotor.setVelocity(MAX_SPEED/2);
        // seenGreen = 0;
      
      }
      
    
    
    }
    
    holding = 100;
    
    while (robot.step(timeStep) != -1 && holding > 0) {
    
    
      holding--;
      liftLowerGripper(-0.0499f);
      openCloseGripper(0.01f);
      leftMotor.setVelocity(0);
      rightMotor.setVelocity(0);
    
    }
    
  
  
  }
  
  
  
  private static void hardcodedPathPushing() {
  

    // Get the actual robot location and direction
    double values[] = TranslationField.getSFVec3f();
    double x = (values[0]*100);
    double y = -(values[2]*100);
    double a = getCompassReadingInDegrees(compass);
  
  
    
    ArrayList<Point> path = new ArrayList<>();
    
    int bffr = 35;
    
    path.add(new Point((int)x, (int)y));
    path.add(new Point(-980+60+bffr, 530));//first box to the left
    path.add(new Point(-970+60+bffr, 800));//two boxes move up
    path.add(new Point(-814, 555));//original spot
    path.add(new Point(-814, 190+30+bffr+120));//bring two down
    // path.add(new Point(-814, 370));//bring two down
    path.add(new Point(-441-30-60-bffr, 190+30+bffr+120));//go to the right
    path.add(new Point(-441-30-bffr, 800));//bring two up
    path.add(new Point(-145-30-bffr, 447+50));//in middle kind of
    path.add(new Point(-145-30-bffr, 122));//bring box down
    path.add(new Point(-145-30-bffr, 278));//go back up
    path.add(new Point(565-150-60-bffr, 278));//go right and push box
    path.add(new Point(565-150-60-bffr, 0+60+bffr));//go down and push box
    path.add(new Point(543-30-bffr, 144));//go right and up
    path.add(new Point(991-20-60-bffr, 144));//go right and push box
    path.add(new Point(875, 100));//go left and down
    path.add(new Point(850, 60));//go to end of line
    	
    

    int timeStep = (int) Math.round(robot.getBasicTimeStep());
    // Run the robot
    while (robot.step(timeStep) != -1) {
      // Check to make sure that the path has been obtained.  If so, start/continue moving
      if ((path != null) && (path.size() > 1)) {
        for (int i=1; i<path.size(); i++) {
          // Get the current location
          values = TranslationField.getSFVec3f();
          x = (values[0]*100);
          y = -(values[2]*100);
          
          makeHardcodedTurn(x, y, path.get(i).x, path.get(i).y);
          
          moveHardcodedAhead(x, y, path.get(i).x, path.get(i).y);
          
        }
        path = null; // Reset for the next time
      } else {
      
        break;
      
      }
    }
    
  
  
  
  }
  
  // This method moves forward from point (x1,y1) to point (x2, y2)
  private static boolean moveHardcodedAhead(double x1, double y1, double x2, double y2) {
  
  
    
    double xDiff = x2 - x1;
    double yDiff = y2 - y1;
    
    // Go forward
    leftMotor.setVelocity(MAX_SPEED);
    rightMotor.setVelocity(MAX_SPEED);
    //System.out.println("Moving Forward from ("+x1+","+y1+") to ("+x2+","+y2+")");
    
    double desiredDistance = Math.sqrt((xDiff)*(xDiff) + (yDiff)*(yDiff));
    int timeStep = (int) Math.round(robot.getBasicTimeStep());
    
    
    // Now wait until the final position has been reached
    while(robot.step(timeStep) != -1) {
    

    
    
      double values[] = TranslationField.getSFVec3f();
      double x = (values[0]*100);
      double y = -(values[2]*100);
      if (Math.sqrt((x-x1)*(x-x1) + (y-y1)*(y-y1)) >= desiredDistance) 
        break;
      
      
      // mapper.updatePosition(x, y);
    }
    // Stop the motors
    leftMotor.setVelocity(0);
    rightMotor.setVelocity(0); 
    return false;
  }
	
  // This method spins from point (x1,y1) to point (x2, y2)
  private static void makeHardcodedTurn(double x1, double y1, double x2, double y2) {
    double xDiff = x2 - x1;
    double yDiff = y2 - y1;
    int startAngle = getCompassReadingInDegrees(compass);
    //System.out.println("Start Angle is " + startAngle);

    int turn = (int)(Math.atan2(yDiff, xDiff) * 180 / Math.PI);
    turn = (turn - startAngle) % 360;
    if (turn < -180)
      turn += 360;
    else if (turn > 180)
      turn -= 360;
    //System.out.println("Turn Amount is " + turn);
    
    int finalAngle = (startAngle + turn +360)%360;
    //System.out.println("Final angle is " + finalAngle);

    if (turn > 0) {// Turn left
      leftMotor.setVelocity(-MAX_SPEED);
      rightMotor.setVelocity(MAX_SPEED);
    }
    else { // turn right
      leftMotor.setVelocity(MAX_SPEED);
      rightMotor.setVelocity(-MAX_SPEED);   
    }
    
    
    // System.out.println("Turning from " + startAngle + " Going to " + finalAngle);
    
    int timeStep = (int) Math.round(robot.getBasicTimeStep());
    // Now wait until the final angle has been reached
    while(robot.step(timeStep) != -1) {
    
      int degrees = (getCompassReadingInDegrees(compass)+360)%360;
      // if (degrees == finalAngle) 
        // break;
      if (degrees > finalAngle - 3 && degrees < finalAngle + 3) 
        break;
       
      int distTurningRight = (degrees - finalAngle + 360)%360;
      int distTurningLeft = (finalAngle - degrees + 360)%360;
      
      if (distTurningRight < distTurningLeft) {
      
        leftMotor.setVelocity(MAX_SPEED);
        rightMotor.setVelocity(-MAX_SPEED);   
      
      
      } else {
      
        leftMotor.setVelocity(-MAX_SPEED);
        rightMotor.setVelocity(MAX_SPEED);
      
      }
      // if ((degrees+360)%360 > finalAngle - 3 && (degrees+360)%360 < finalAngle + 3) 
        // break;
    }
    
    // Stop the motors
    leftMotor.setVelocity(0);
    rightMotor.setVelocity(0); 
  }
  
  
  private static void toStart() {
  

    // Get the actual robot location and direction
    double values[] = TranslationField.getSFVec3f();
    double x = (values[0]*100);
    double y = -(values[2]*100);
    double a = getCompassReadingInDegrees(compass);
  
  
    
    ArrayList<Point> path = new ArrayList<>();
    
    int bffr = 35;
    
    path.add(new Point((int)x, (int)y));
    path.add(new Point(850, 100));//go up
    path.add(new Point(565+150-30-bffr, 144));//go below wall
    path.add(new Point(0, 144));//go beside wall kindve
    path.add(new Point(-75, 278));//go to opening
    path.add(new Point(-215, 278));//pass through opening
    path.add(new Point(-340, 605));//open area above L
    path.add(new Point(-590, 510));//open are beside L kindve
    path.add(new Point(-590, 390));//open are beside L kindve down now
    path.add(new Point(-815, 410));//above 2 pushed boxes
    path.add(new Point(-850, 602));//beside 5 jar placement things
    
   
    
    
    	
    

    int timeStep = (int) Math.round(robot.getBasicTimeStep());
    // Run the robot
    while (robot.step(timeStep) != -1) {
      // Check to make sure that the path has been obtained.  If so, start/continue moving
      if ((path != null) && (path.size() > 1)) {
        for (int i=1; i<path.size(); i++) {
          // Get the current location
          values = TranslationField.getSFVec3f();
          x = (values[0]*100);
          y = -(values[2]*100);
          
          MAX_SPEED = 1;
          makeHardcodedTurn(x, y, path.get(i).x, path.get(i).y);
          MAX_SPEED = 5;
          moveHardcodedAhead(x, y, path.get(i).x, path.get(i).y);
          
        }
        path = null; // Reset for the next time
      } else {
      
        break;
      
      }
    }
    
  
  
  
  }
  
  
  
  private static void placeJar() {
  

    // Get the actual robot location and direction
    double values[] = TranslationField.getSFVec3f();
    double x = (values[0]*100);
    double y = -(values[2]*100);
    double a = getCompassReadingInDegrees(compass);
  
  
    
    ArrayList<Point> path = new ArrayList<>();
    
    int bffr = 35;
    
    
    
    path.add(new Point((int)x, (int)y));
    int yVal = 602+jarsToGo*30+5;
    path.add(new Point(-850, yVal));//beside 5 jar placement things
    path.add(new Point(-900, yVal));//beside 5 jar placement things
    path.add(new Point(-915, yVal));//beside 5 jar placement things
    path.add(new Point(-967+31-5, yVal));//beside 5 jar placement things
    
    
    
    // path.add(new Point(850, 100));//go up
    // path.add(new Point(565+150-30-bffr, 144));//go below wall
    // path.add(new Point(0, 144));//go beside wall kindve
    // path.add(new Point(-75, 278));//go to opening
    // path.add(new Point(-215, 278));//pass through opening
    // path.add(new Point(-340, 605));//open area above L
    // path.add(new Point(-590, 510));//open are beside L kindve
    // path.add(new Point(-590, 390));//open are beside L kindve down now
    // path.add(new Point(-815, 410));//above 2 pushed boxes
    // path.add(new Point(-915, 720));//beside 5 jar placement things
    
   
    
    
    	
    

    int timeStep = (int) Math.round(robot.getBasicTimeStep());
    // Run the robot
    while (robot.step(timeStep) != -1) {
      // Check to make sure that the path has been obtained.  If so, start/continue moving
      if ((path != null) && (path.size() > 1)) {
        for (int i=1; i<path.size(); i++) {
          // Get the current location
          values = TranslationField.getSFVec3f();
          x = (values[0]*100);
          y = -(values[2]*100);
          
          MAX_SPEED = 1;
          makeHardcodedTurn(x, y, path.get(i).x, path.get(i).y);
          MAX_SPEED = 5;
          moveHardcodedAhead(x, y, path.get(i).x, path.get(i).y);
          
        }
        path = null; // Reset for the next time
      } else {
      
        break;
      
      }
    }
    
    int holding = 100;
    
    while (holding > 50 && robot.step(timeStep) != -1) {
    
    
      openCloseGripper(0.099f);
      liftLowerGripper(0.001f);
      holding--;
    
    }
    
    
    while (holding > 0 && robot.step(timeStep) != -1) {
    
      leftMotor.setVelocity(-MAX_SPEED);
      rightMotor.setVelocity(-MAX_SPEED);
      holding--;
    
    }
    
    jarsToGo--;
    
  
  
  
  }
  
  
  private static void hardcodedPath() {
  

    // Get the actual robot location and direction
    double values[] = TranslationField.getSFVec3f();
    double x = (values[0]*100);
    double y = -(values[2]*100);
    double a = getCompassReadingInDegrees(compass);
  
  
    
    ArrayList<Point> path = new ArrayList<>();
    
    
    int bffr = 35;
    
    path.add(new Point((int)x, (int)y));
    path.add(new Point(-850, 602));//beside 5 jar placement things
    path.add(new Point(-815, 410));//above 2 pushed boxes
    path.add(new Point(-590, 390));//open are beside L kindve down now
    path.add(new Point(-590, 510));//open are beside L kindve
    path.add(new Point(-340, 605));//open area above L
    path.add(new Point(-220, 400));//in L kindve
    path.add(new Point(-215, 278));//pass through opening
    path.add(new Point(-75, 278));//go to opening
    path.add(new Point(0, 144));//go beside wall kindve
    path.add(new Point(565-150-60-bffr, 144));//go below wall
    path.add(new Point(850, 100));//go up
    path.add(new Point(850, 60));//go to end of line
    	

    	

    int timeStep = (int) Math.round(robot.getBasicTimeStep());
    // Run the robot
    while (robot.step(timeStep) != -1) {
      // Check to make sure that the path has been obtained.  If so, start/continue moving
      if ((path != null) && (path.size() > 1)) {
        for (int i=1; i<path.size(); i++) {
          // Get the current location
          values = TranslationField.getSFVec3f();
          x = (values[0]*100);
          y = -(values[2]*100);
          
          // System.out.println("going to " + path.get(i));
          
          makeHardcodedTurn(x, y, path.get(i).x, path.get(i).y);
          
          moveHardcodedAhead(x, y, path.get(i).x, path.get(i).y);
          
        }
        path = null; // Reset for the next time
      } else {
      
        break;
      
      }
    }
    
  
  
  
  }
  
  
  
  
}
