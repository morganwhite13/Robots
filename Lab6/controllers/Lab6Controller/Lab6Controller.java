// Author: Morgan White (SN:101184799) & COAuthor Osamah Alqahtaini SN: 101179283

import com.cyberbotics.webots.controller.Camera;
import com.cyberbotics.webots.controller.PositionSensor;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Robot;

public class Lab6Controller {

  //*******************
  // Camera dimensions
  //*******************
  static final byte CAMERA_WIDTH = 52;
  static final byte CAMERA_HEIGHT = 39;
  
  //****************************
  // The FIXED beacon locations
  //****************************
  static final int  x1 = -60, y1 =  30;  // RED Beacon location
  static final int  x2 =   0, y2 = -70;  // GREEN Beacon location
  static final int  x3 =  60, y3 =   0;  // BLUE Beacon location
  
  //*****************************************************************************************************
  // The locations to visit in sequence.  These numbers do not match the numbers in the LAB instructions.
  // They have been tweaked slightly due to the innacurate movements of the robot.  DO NOT change any
  // of them because they have been adjusted so that your estimations will be as accurate as possible. 
  //*****************************************************************************************************
  static final int     x[] = {0, -30, 10, 41, 72, 73, 94, 84,  86,  55,  15,   3, -28, -49, -71, -77};
  static final int     y[] = {0,  42, 41, 62, 62, 11, 12, -4, -50, -41, -64, -34, -24, -25,  -6, -68};
  
  static final double  MAX_SPEED      = 1;    // Need to go this slow for accurate position estimation
  static final double  WHEEL_RADIUS   = 2.05; // cm 
  static final double  WHEEL_BASE     = 5.80; // cm
  
  
  
  
  
  //*******************************************
  // Variables needed by the various functions
  //*******************************************
  static Robot            Epuck;
  static Motor            LeftMotor, RightMotor;
  static PositionSensor   LeftEncoder, RightEncoder;
  static Camera           Cam;
  static int              TimeStep;
  static double           PreviousReading = 0;
  static double startAngle = 90;  // Angle that that robot starts at (i.e., 90 degrees)

  
  


  
  //*************************************************************************************
  // Set each motor to a specific speed and wait until the left sensor reaches 
  // the specified number of radians.  DO NOT CHANGE IT.
  //*************************************************************************************
  private static void move(double leftSpeed, double rightSpeed, double thisManyRadians) {
    LeftMotor.setVelocity(leftSpeed);
    RightMotor.setVelocity(rightSpeed);
    while(true) {
      double reading = RightEncoder.getValue() - PreviousReading;
      if ((thisManyRadians > 0) && (reading > thisManyRadians))
        break;
      if ((thisManyRadians < 0) && (reading < thisManyRadians)) 
        break;
      Epuck.step(TimeStep);
    }
    //Epuck.step(TimeStep);
    LeftMotor.setVelocity(0);
    RightMotor.setVelocity(0);
    Epuck.step(TimeStep);
    PreviousReading = RightEncoder.getValue();
    Epuck.step(TimeStep);
  }
  
  
  
  
  //*************************************************************************************
  // This method moves forward from point (x1,y1) to point (x2, y2).  DO NOT CHANGE IT.
  //*************************************************************************************
  private static void moveAhead(double x1, double y1, double x2, double y2) {
    double xDiff = x2 - x1;
    double yDiff = y2 - y1;
    double radiansToTurn = (Math.sqrt(xDiff*xDiff + yDiff*yDiff) / WHEEL_RADIUS);

    move(MAX_SPEED, MAX_SPEED, radiansToTurn);
  }
	
  //*************************************************************************************
  // This method spins from point (x1,y1) to point (x2, y2).  DO NOT CHANGE IT.
  //*************************************************************************************
  private static void makeTurn(double x1, double y1, double x2, double y2) {
    double xDiff = x2 - x1;
    double yDiff = y2 - y1;
    double turn = Math.atan2(yDiff, xDiff) * 180 / Math.PI;
    turn = (turn - startAngle) % 360;
    if (turn < -180)
      turn += 360;
    else if (turn > 180)
      turn -= 360;

    // Determine how many radians to spin for
    double radiansToTurn = (turn * Math.PI / 180.0) * (1.0/(WHEEL_RADIUS/WHEEL_BASE*2.0));
    if (turn > 0) 
      move(-MAX_SPEED, MAX_SPEED, radiansToTurn);
    else
      move(MAX_SPEED, -MAX_SPEED, radiansToTurn);
    startAngle += turn;
  }


  //*************************************************************************************
  // This method spins clockwise 360 degrees looking for the three beacons.   
  // It then calculates its position based on triangulation.  The angle
  // passed in is the angle that the robot started at before spinning. 
  // The point number can be used for printing to indicate which point is
  // being calculated at this time. 
  //*************************************************************************************
  private static void calculatePosition(double angle, byte pointNumber) {
    // SPIN for 360 degrees DO NOT CHANGE THIS CODE
    double spinRadians = (Math.PI*WHEEL_BASE) / WHEEL_RADIUS;
    LeftMotor.setVelocity(-MAX_SPEED);
    RightMotor.setVelocity(MAX_SPEED);

    //*************************************************************************************
    // ADD ANY INTIALIZATION CODE YOU WANT HERE
    //*************************************************************************************
    double angles[] = {-999, -999, -999};
    int r;
    int g;
    int b;
    boolean redUnfound = true;
    boolean greenUnfound = true;
    boolean blueUnfound = true;
    
    
    // Do NOT alter the looping code!!! ... just insert inside it at the location shown
    double   reading = 0;
    while ((Epuck.step(TimeStep) != -1) && (reading < spinRadians)) {
      reading = RightEncoder.getValue() - PreviousReading;

      int[] image = Cam.getImage();
      
    //*************************************************************************************
    // INSERT YOUR BEACON-FINDING CODE HERE
    //*************************************************************************************
      
      r = Camera.imageGetRed(image, 52, CAMERA_WIDTH/2, 19);
      g = Camera.imageGetGreen(image, 52, CAMERA_WIDTH/2, 19);
      b = Camera.imageGetBlue(image, 52, CAMERA_WIDTH/2, 19); 
      
      if (r > 60 && g < 50 && b < 50 && redUnfound) {
      
      
      
        boolean findingRed = true;
        int leftRedness = 0;
        for (int i = CAMERA_WIDTH/2 - 1; i > -1 && findingRed; i--) {
          
          r = Camera.imageGetRed(image, 52, CAMERA_WIDTH/2, 19);
          g = Camera.imageGetGreen(image, 52, CAMERA_WIDTH/2, 19);
          b = Camera.imageGetBlue(image, 52, CAMERA_WIDTH/2, 19); 
        
          if (r > 60 && g < 50 && b < 50) {
          
            leftRedness++;
          
          } else {
          
          
            findingRed = false;
          
          
          }
        
        
        }
      
        findingRed = true;
        int rightRedness = 0;
        for (int i = CAMERA_WIDTH/2 + 1; i < CAMERA_WIDTH && findingRed; i++) {
          
          r = Camera.imageGetRed(image, 52, CAMERA_WIDTH/2, 19);
          g = Camera.imageGetGreen(image, 52, CAMERA_WIDTH/2, 19);
          b = Camera.imageGetBlue(image, 52, CAMERA_WIDTH/2, 19); 
        
          if (r > 60 && g < 50 && b < 50) {
          
            rightRedness++;
          
          } else {
          
          
            findingRed = false;
          
          
          }
        
        
        }
      
      
        if (leftRedness + 2 > rightRedness && leftRedness - 2 < rightRedness) {
        
        
          angles[0] = fixA(angle + (reading / spinRadians * 360));
          //System.out.println(angles[0]);
          redUnfound = false;
        
        }
      
      
      
      } else if (g > 60 && r < 50 && b < 50 && greenUnfound) {
      
      
        boolean findingGreen = true;
        int leftGreenness = 0;
        for (int i = CAMERA_WIDTH/2 - 1; i > -1 && findingGreen; i--) {
          
          r = Camera.imageGetRed(image, 52, CAMERA_WIDTH/2, 19);
          g = Camera.imageGetGreen(image, 52, CAMERA_WIDTH/2, 19);
          b = Camera.imageGetBlue(image, 52, CAMERA_WIDTH/2, 19); 
        
          if (r > 60 && g < 50 && b < 50) {
          
            leftGreenness++;
          
          } else {
          
          
            findingGreen = false;
          
          
          }
        
        
        }
      
        findingGreen = true;
        int rightGreenness = 0;
        for (int i = CAMERA_WIDTH/2 + 1; i < CAMERA_WIDTH && findingGreen; i++) {
          
          r = Camera.imageGetRed(image, 52, CAMERA_WIDTH/2, 19);
          g = Camera.imageGetGreen(image, 52, CAMERA_WIDTH/2, 19);
          b = Camera.imageGetBlue(image, 52, CAMERA_WIDTH/2, 19); 
        
          if (r > 60 && g < 50 && b < 50) {
          
            rightGreenness++;
          
          } else {
          
          
            findingGreen = false;
          
          
          }
        
        
        }
      
      
        if (leftGreenness + 2 > rightGreenness && leftGreenness - 2 < rightGreenness) {
        
        
          angles[1] = fixA(angle + (reading / spinRadians * 360));
          //System.out.println(angles[1]);
          greenUnfound = false;
        
        }
      
      
      
      } else if (b > 60 && r < 50 && g < 50 && blueUnfound) {
      
      
        boolean findingBlue = true;
        int leftBlueness = 0;
        for (int i = CAMERA_WIDTH/2 - 1; i > -1 && findingBlue; i--) {
          
          r = Camera.imageGetRed(image, 52, CAMERA_WIDTH/2, 19);
          g = Camera.imageGetGreen(image, 52, CAMERA_WIDTH/2, 19);
          b = Camera.imageGetBlue(image, 52, CAMERA_WIDTH/2, 19); 
        
          if (r > 60 && g < 50 && b < 50) {
          
            leftBlueness++;
          
          } else {
          
          
            findingBlue = false;
          
          
          }
        
        
        }
      
        findingBlue = true;
        int rightBlueness = 0;
        for (int i = CAMERA_WIDTH/2 + 1; i < CAMERA_WIDTH && findingBlue; i++) {
          
          r = Camera.imageGetRed(image, 52, CAMERA_WIDTH/2, 19);
          g = Camera.imageGetGreen(image, 52, CAMERA_WIDTH/2, 19);
          b = Camera.imageGetBlue(image, 52, CAMERA_WIDTH/2, 19); 
        
          if (r > 60 && g < 50 && b < 50) {
          
            rightBlueness++;
          
          } else {
          
          
            findingBlue = false;
          
          
          }
        
        
        }
      
      
        if (leftBlueness + 1 > rightBlueness && leftBlueness - 1 < rightBlueness) {
        
        
          angles[2] = fixA(angle + (reading / spinRadians * 360));
          //System.out.println(angles[2]);
          blueUnfound = false;
        
        }
      
      
      
      }
      
      
      
      
      
      
      
    // DO NOT CHANGE THE NEXT SIX LINES OF CODE!!
    }
    LeftMotor.setVelocity(0);
    RightMotor.setVelocity(0);
    Epuck.step(TimeStep);
    PreviousReading = RightEncoder.getValue();
    Epuck.step(TimeStep);
    
    
    
    //*************************************************************************************
    // YOU MAY WANT TO PRINT OUT SOMETHING HERE
    //*************************************************************************************
    
    if (!redUnfound && !blueUnfound && !greenUnfound) {
      double xPrime1 = x1 - x2;
      double yPrime1 = y1 - y2;
      double xPrime3 = x3 - x2;
      double yPrime3 = y3 - y2;
      
      double coTangent12 = 1 / Math.tan(Math.toRadians(angles[1])-Math.toRadians(angles[0]));
      double coTangent23 = 1 / Math.tan(Math.toRadians(angles[2])-Math.toRadians(angles[1]));
      double coTangent31 = (1 - coTangent12 * coTangent23) / (coTangent12 + coTangent23);
      
      double xPrime12 = xPrime1 + coTangent12 * yPrime1;
      double yPrime12 = yPrime1 - coTangent12 * xPrime1;
      double xPrime23 = xPrime3 - coTangent23 * yPrime3;
      double yPrime23 = yPrime3 + coTangent23 * xPrime3;
      double xPrime31 = (xPrime3 + xPrime1) + coTangent31 * (yPrime3 - yPrime1);
      double yPrime31 = (yPrime3 + yPrime1) - coTangent31 * (xPrime3 - xPrime1);
      
      double k31 = xPrime1 * xPrime3 + yPrime1 * yPrime3 + coTangent31 * (xPrime1*yPrime3 - xPrime3*yPrime1);
      
      double distance = (xPrime12 - xPrime23) * (yPrime23 - yPrime31) - (yPrime12 - yPrime23) * (xPrime23 - xPrime31);
      
      if (distance == 0) {
      
        System.out.println("(x" + pointNumber + ", y" + pointNumber + ") Cant Compute");
      
      } else {
      
      
        
        double robotX = x2 + k31 * (yPrime12 - yPrime23) / distance;
        double robotY = y2 + k31 * (xPrime23 - xPrime12) / distance;
        System.out.println("(x" + pointNumber + ", y" + pointNumber + ") = (" + (Math.round(robotX * 10.0) / 10.0) + ", " + (Math.round(robotY * 10.0) / 10.0) + ")");
        System.out.println("distance  " + distance);
      
      }
    
    } else {
    
      System.out.println("(x" + pointNumber + ", y" + pointNumber + ") Cant Compute");
    
    
    }
    //System.out.println("robot x " + robotX);
  //  System.out.println("robot y " + robotY);
    
//    System.out
    
    
    
    
    
  }


  //*******************************************************************************************
  // DO NOT CHANGE THE MAIN FUNCTION ... EXCEPT FOR COMMENTING THINGS OUT WHILE YOU ARE TESTING
  //*******************************************************************************************
  public static void main(String[] args) {
    Epuck = new Robot();
    TimeStep = (int) Math.round(Epuck.getBasicTimeStep());

    // Get the motors
    LeftMotor = Epuck.getMotor("left wheel motor");
    RightMotor = Epuck.getMotor("right wheel motor");
    LeftMotor.setPosition(Double.POSITIVE_INFINITY);// indicates continuous rotation servo
    RightMotor.setPosition(Double.POSITIVE_INFINITY);// indicates continuous rotation servo
    LeftMotor.setVelocity(0);
    RightMotor.setVelocity(0);
    
    // Get the encoders
    LeftEncoder = Epuck.getPositionSensor("left wheel sensor");
    RightEncoder = Epuck.getPositionSensor("right wheel sensor");
    LeftEncoder.enable(TimeStep);
    RightEncoder.enable(TimeStep);
    
    // Set up the camera
    Cam = new Camera("camera");
    Cam.enable(TimeStep);
    
    // Travel through the points in the array one at a time in sequence and determine the location at each point by using triangulation
    for (byte i=0; i<x.length - 1; i++) {
      calculatePosition(startAngle, i);
      makeTurn(x[i], y[i], x[i+1], y[i+1]);
      moveAhead(x[i], y[i], x[i+1], y[i+1]);
    }
    calculatePosition(startAngle, (byte)(x.length - 1));  // Get the last one  
  }
  
  
  public static double fixA(double a) {
  
    if (a > 180) {
    
      return a - 360;
    
    } else if (a < -180) {
    
      return a + 360;
    
    }
    return a;
  
  
  }
  
  
  
  
}




  
