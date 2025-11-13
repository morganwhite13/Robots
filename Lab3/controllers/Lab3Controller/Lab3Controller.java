// Author: Morgan White SN: 101184799, COAUTHOR: Liam Lowndes SN: 101041818


import com.cyberbotics.webots.controller.Camera;
import com.cyberbotics.webots.controller.Accelerometer;
import com.cyberbotics.webots.controller.DistanceSensor;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Robot;

public class Lab3Controller {

  static final byte CAMERA_WIDTH  = 52;
  static final byte CAMERA_HEIGHT = 39;
  
  static final byte WANDER      = 0;
  static final byte HOME_IN     = 1;
  static final byte PUSH_BALL   = 2;
  static final byte TURN_AROUND = 3;
  static final byte GO_FORWARD  = 4;
  static final byte AVOID       = 5;
  
  static final byte LEFT        = 0;
  static final byte RIGHT       = 1;
  static final byte NONE        = 2;
  static final byte FULL_SPEED  = 6;
  
  static final double  MAX_SPEED = 5;  // maximum speed of the epuck robot
  
  public static void main(String[] args) {
    Robot robot = new Robot();
    int timeStep = (int) Math.round(robot.getBasicTimeStep());

    // Get the motors
    Motor leftMotor = robot.getMotor("left wheel motor");
    Motor rightMotor = robot.getMotor("right wheel motor");
    leftMotor.setPosition(Double.POSITIVE_INFINITY);
    rightMotor.setPosition(Double.POSITIVE_INFINITY);

    // Get and enable the distance sensors
    DistanceSensor leftAheadSensor = robot.getDistanceSensor("ps7"); 
    DistanceSensor rightAheadSensor = robot.getDistanceSensor("ps0"); 
    leftAheadSensor.enable(timeStep);
    rightAheadSensor.enable(timeStep);

    // Get and enable the accelerometer
    Accelerometer accelerometer = new Accelerometer("accelerometer");
    accelerometer.enable(timeStep);
    
    // Set up the camera
    Camera camera = new Camera("camera");
    camera.enable(timeStep);
    
    int leftSpeed, rightSpeed;
    leftSpeed = rightSpeed = 0;
    
    byte currentMode = WANDER;
    int curveCount = 0;
    int curveDir = 0;
    
    //int[] image = camera.getImage();
    int r;
    int g;
    int b;
    int redness = 0;
    
    byte turnCount = 0;
    byte turnDirection = NONE;
    
    double[] accelValues = new double[3];
    
    double[] xReadings = new double[10];
    double[] yReadings = new double[10];
    int readCounter = 0;
    
    int spinDir = 0;
    
    while (robot.step(timeStep) != -1) {
    
    
      accelValues = accelerometer.getValues();
      xReadings[readCounter%10] = accelValues[0];
      yReadings[readCounter%10] = accelValues[1];
      readCounter++;
      double xAvg = 0;
      double yAvg = 0;
      for (int i = 0; i < 10; i++) {
      
        xAvg += xReadings[i];
        yAvg += yReadings[i];
      
      }
      xAvg /= 10;
      yAvg /= 10;
      
    
      int[] image = camera.getImage();
      // SENSE: Read the sensors
      leftAheadSensor = robot.getDistanceSensor("ps7"); 
      rightAheadSensor = robot.getDistanceSensor("ps0"); 
      //System.out.println(curveCount +"currebtnide" + currentMode);
      //System.out.println(redness);
      
      for (int i = 0; i < 52; i++) {
      
        r = Camera.imageGetRed(image, 52, i, 19);  // 52 is image width
        g = Camera.imageGetGreen(image, 52, i, 19);
        b = Camera.imageGetBlue(image, 52, i, 19); 
    
        //if (((r > 60) && (g < 100) && (b < 100)) && (( r > b) && (r > g))) {
        if (((r > 60) && (g < 100) && (b < 100))) {
          redness++;
        
        }
      
      }
      
      // THINK: Make a decision as to what MODE to be in
      switch(currentMode) {
        case WANDER:
          if (redness > 3) {
            //red detected or whatever
            currentMode = HOME_IN;
            curveCount = 0;
            curveDir = 0;
            //System.out.println("sssssss");
           
          } else if (leftAheadSensor.getValue() > 80 || rightAheadSensor.getValue() > 80) {
          
              currentMode = AVOID;
              curveCount = 0;
              curveDir = 0;
              //System.out.println("cccc");
          
          
          } else if (yAvg > 0.2 || yAvg < -0.2 || xAvg > 0.2 || xAvg < -0.2) {
          
            currentMode = TURN_AROUND;
            curveCount = 0;
            curveDir = 0;
            //System.out.println("AAAAAAA");
          
          } else if (curveCount == 0) {
          
            //System.out.println("SOMEHTING"+ curveCount +"currebtnide" + currentMode);
            if (Math.random() > 0.8) {
              curveCount = (int) (Math.random()*50 + 25);
              if (Math.random() > 0.5) {
                
                curveDir = 1;
              
              } else {
                curveDir = 2;
              
              }
            }
          
          }
          //System.out.println(curveCount + "dddd" + currentMode);
        
          break;
        case HOME_IN:
          if (!(redness > 0)) {
            //red detected or whatever
            currentMode = WANDER;
           
          } else if (leftAheadSensor.getValue() > 80 || rightAheadSensor.getValue() > 80) {
          
            currentMode = PUSH_BALL;
          
          } else if (yAvg < -0.2) {
          
            currentMode = TURN_AROUND;
          
          }
          break;
        case PUSH_BALL:
          if (!(leftAheadSensor.getValue() > 80 || rightAheadSensor.getValue() > 80)) {
            currentMode = WANDER;
          } else if (yAvg < -0.2) {
          //acceloremeter
            currentMode = TURN_AROUND;
          
          }
          break;
        case TURN_AROUND:
          if (yAvg < 0.2 && yAvg > -0.2 && xAvg < 0.2 && xAvg > -0.2) {
            currentMode = WANDER;
            spinDir = 0;
          
          } else if (yAvg > 0.2 && xAvg < 0.5 && xAvg > -0.5) {
          
            currentMode = GO_FORWARD;
            spinDir = 0;
          
          } else if (spinDir == 0) {
          
          
            if (Math.random() > 0.5) {
              
              spinDir = 1;
            
            } else {
              spinDir = 2;
            
            }
          }
        
          break;
        case GO_FORWARD:
          if (yAvg < 0.2 && yAvg > -0.2) {
          
            currentMode = WANDER;
          
          } else if (yAvg < 0.2 || xAvg > 0.2 || xAvg < -0.2) {
          
            currentMode = TURN_AROUND;
          
          }
        
          break;
        case AVOID:
          if (!(leftAheadSensor.getValue() > 80 || rightAheadSensor.getValue() > 80)) {
          
            currentMode = WANDER;
            curveDir = 0;
          
          } else if (curveDir == 0) {
            
             if (Math.random() > 0.5) {
              
              curveDir = 1;
            
            } else {
              curveDir = 2;
            
            }
          
          }
          break;
      }
      
      // REACT: Move motors according to the MODE
      switch(currentMode) {
        case WANDER: 
        /*
          if (curveCount > 0) {
            if (curveDir == 1) {
              curveLeft(leftMotor, rightMotor);
            
            } else {
              curveRight(leftMotor, rightMotor);
            }
            curveCount--;
          } else {
          
            moveStraight(leftMotor, rightMotor);
          
          }
          */
          if (turnCount > 0) {
            if (turnDirection == LEFT) {
              //leftSpeed -= 2;
              leftMotor.setVelocity(-2);
              rightMotor.setVelocity(MAX_SPEED);
            } else {
              //rightSpeed -= 2;
              leftMotor.setVelocity(MAX_SPEED);
              rightMotor.setVelocity(-2);
            }
            turnCount--;
            if (--turnCount == 0) 
              turnDirection = NONE;
          }
          else {
            if ((byte)(Math.random()*5) == 0) {// turn 20% of the time
              turnDirection = (byte)(Math.random()*2);
              turnCount = (byte)(Math.random()*51+25);
            }
          }
          break;
        case HOME_IN:
        
          int leftCount = 0;
          for (int i = 0; i < 17; i++) {
    
            r = Camera.imageGetRed(image, 52, i, 19);  // 52 is image width
            g = Camera.imageGetGreen(image, 52, i, 19);
            b = Camera.imageGetBlue(image, 52, i, 19); 
        
            //if (((r > 80) && (g < 100) && (b < 100)) && (( r > b) && (r > g))) {
            if (((r > 30) && (g < 30) && (b < 30))) {
            
              leftCount++;
            
            }
        
          }
          
          int centerCount = 0;
          for (int i = 17; i < 35; i++) {
    
            r = Camera.imageGetRed(image, 52, i, 19);  // 52 is image width
            g = Camera.imageGetGreen(image, 52, i, 19);
            b = Camera.imageGetBlue(image, 52, i, 19); 
        
            //if (((r > 80) && (g < 100) && (b < 100)) && (( r > b) && (r > g))) {
            if (((r > 30) && (g < 30) && (b < 30))) {
            
              centerCount++;
            
            }
        
          }
          
          int rightCount = 0;
          for (int i = 35; i < 52; i++) {
    
            r = Camera.imageGetRed(image, 52, i, 19);  // 52 is image width
            g = Camera.imageGetGreen(image, 52, i, 19);
            b = Camera.imageGetBlue(image, 52, i, 19); 
        
            //if (((r > 80) && (g < 100) && (b < 100)) && (( r > b) && (r > g))) {
            if (((r > 30) && (g < 30) && (b < 30))) {
            
              rightCount++;
            
            }
        
          }
          
          int buffer = 5;
          if (leftCount > rightCount + buffer) {
          
            curveLeft(leftMotor, rightMotor);
          
          } else if (rightCount > leftCount + buffer) {
          
            curveRight(leftMotor, rightMotor);
          
          } else {
          
            moveStraight(leftMotor, rightMotor);
          
          }
          
          //IDK
        
          break;
        case PUSH_BALL: 
        //we changed the push ball code so that instead of just moving straight it instead does exactly what the home-in code does as this made it much better at pushing the balls off the edge
          //moveStraight(leftMotor, rightMotor);
          
          
          int leftCount2 = 0;
          for (int i = 0; i < 17; i++) {
    
            r = Camera.imageGetRed(image, 52, i, 19);  // 52 is image width
            g = Camera.imageGetGreen(image, 52, i, 19);
            b = Camera.imageGetBlue(image, 52, i, 19); 
        
            //if (((r > 80) && (g < 100) && (b < 100)) && (( r > b) && (r > g))) {
            if (((r > 80) && (g < 100) && (b < 100))) {
            
              leftCount2++;
            
            }
        
          }
          
          int centerCount2 = 0;
          for (int i = 17; i < 35; i++) {
    
            r = Camera.imageGetRed(image, 52, i, 19);  // 52 is image width
            g = Camera.imageGetGreen(image, 52, i, 19);
            b = Camera.imageGetBlue(image, 52, i, 19); 
        
            //if (((r > 80) && (g < 100) && (b < 100)) && (( r > b) && (r > g))) {
            if (((r > 80) && (g < 100) && (b < 100))) {
            
              centerCount2++;
            
            }
        
          }
          
          int rightCount2 = 0;
          for (int i = 35; i < 52; i++) {
    
            r = Camera.imageGetRed(image, 52, i, 19);  // 52 is image width
            g = Camera.imageGetGreen(image, 52, i, 19);
            b = Camera.imageGetBlue(image, 52, i, 19); 
        
            //if (((r > 80) && (g < 100) && (b < 100)) && (( r > b) && (r > g))) {
            if (((r > 80) && (g < 100) && (b < 100))) {
            
              rightCount2++;
            
            }
        
          }
          
          int buffer2 = 5;
          if (leftCount2 > rightCount2 + buffer2) {
          
            curveLeft(leftMotor, rightMotor);
          
          } else if (rightCount2 > leftCount2 + buffer2) {
          
            curveRight(leftMotor, rightMotor);
          
          } else {
          
            moveStraight(leftMotor, rightMotor);
          
          }
          
          
          break;
        case TURN_AROUND:
          if (leftAheadSensor.getValue() > 80 || rightAheadSensor.getValue() > 80) {
          
            moveBack(leftMotor, rightMotor);
          
          } else if (spinDir == 1) {
          
            spinLeft(leftMotor, rightMotor);
          
          } else {
          
            spinRight(leftMotor, rightMotor);
            
          
          }
          
          break;
        case GO_FORWARD: 
          moveStraight(leftMotor, rightMotor);
          break;
        case AVOID: 
        //we changed the avoid code from being random into being based on the amount detected from the left and right ahead sensors so it could avoid walls easier
        /*
          if (curveDir == 1) {
            pivotLeft(leftMotor, rightMotor);
            
          
          } else {
            pivotRight(leftMotor, rightMotor);
          }*/
          if (rightAheadSensor.getValue() > 80 && rightAheadSensor.getValue() > leftAheadSensor.getValue()) {
          
            pivotLeft(leftMotor, rightMotor);
          
          } else {
            pivotRight(leftMotor, rightMotor);
          }
          
          
      }

      //leftMotor.setVelocity(leftSpeed);
      //rightMotor.setVelocity(rightSpeed);
      redness = 0;
    }
  }
  
  
     
  public static void moveStraight(Motor leftMotor, Motor rightMotor) {
  
    leftMotor.setVelocity(MAX_SPEED);
    rightMotor.setVelocity(MAX_SPEED);
  
    //System.out.println("straight state");
  
  
  
  }
  
  public static void moveBack(Motor leftMotor, Motor rightMotor) {
  
    leftMotor.setVelocity(-MAX_SPEED);
    rightMotor.setVelocity(-MAX_SPEED);
  
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
  
  
    
  public static void pivotLeft(Motor leftMotor, Motor rightMotor) {
  
    leftMotor.setVelocity(0.1*MAX_SPEED);
    rightMotor.setVelocity(MAX_SPEED);
  
    //System.out.println("PIVOTING");
  
  //System.out.println("spin right state");
  
  }
  
  
  
  
  
  
  
  
  
  
  
}
