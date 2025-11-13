// Author: Morgan White (SN: 101184799) & COAUTHOR David Zhou SN: 101128523
 
import com.cyberbotics.webots.controller.DistanceSensor;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Robot;
import com.cyberbotics.webots.controller.Device;
import com.cyberbotics.webots.controller.Node;
import com.cyberbotics.webots.controller.LightSensor;

public class Lab7Controller {

  // Various modes that the robot may be in
  static final byte    STRAIGHT = 0;
  static final byte    SPIN_LEFT = 1;
  static final byte    SPIN_RIGHT = 2;
  static final byte    CURVE_LEFT = 3;
  static final byte    CURVE_RIGHT = 4;


  static final double  MAX_SPEED = 6.28;  // maximum speed of the epuck robot
  //static final double  MAX_SPEED = 0.01;  // maximum speed of the epuck robot


  static int calibrationValues[] = {300, 720, 822 , 842, 525, 767, 800, 858, 644};

  static String[] CELL_NAMES = {"TOP_LEFT", "TOP_CENTER", "TOP_RIGHT", 
                                "MIDDLE_LEFT", "MIDDLE_CENTER", "MIDDLE_RIGHT", 
                                "BOTTOM_LEFT", "BOTTOM_CENTER", "BOTTOM_RIGHT", "N/A"};

  static int[][] X_OFFSETS  = {{0, 1, -1, 0, 1, -1, 0, 1, -1, 0},
		         {-1, 0, 1, -1, 0, 1, -1, 0, 1, 0},
		         {1, -1, 0, 1, -1, 0, 1, -1, 0, 0},
		         {0, 1, -1, 0, 1, -1, 0, 1, -1, 0},
		         {-1, 0, 1, -1, 0, 1, -1, 0, 1, 0},
		         {1, -1, 0, 1, -1, 0, 1, -1, 0, 0},
		         {0, 1, -1, 0, 1, -1, 0, 1, -1, 0},
		         {-1, 0, 1, -1, 0, 1, -1, 0, 1, 0},
		         {1, -1, 0, 1, -1, 0, 1, -1, 0, 0},
		         {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};

  static int[][] Y_OFFSETS  = {{0, 0, 0, -1, -1, -1, 1, 1, 1, 0},
    		         {0, 0, 0, -1, -1, -1, 1, 1, 1, 0},
		         {0, 0, 0, -1, -1, -1, 1, 1, 1, 0},
		         {1, 1, 1, 0, 0, 0, -1, -1, -1, 0},
		         {1, 1, 1, 0, 0, 0, -1, -1, -1, 0},
		         {1, 1, 1, 0, 0, 0, -1, -1, -1, 0},
		         {-1, -1, -1, 1, 1, 1, 0, 0, 0, 0},
		         {-1, -1, -1, 1, 1, 1, 0, 0, 0, 0},
		         {-1, -1, -1, 1, 1, 1, 0, 0, 0, 0},
		         {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};

  static int[][] A_OFFSETS  = {{0, 0, 180, -90, -45, -135, 90, 45, 135, 0},
		         {180, 0, 0, -135, -90, -45, 135, 90, 45, 0},
		         {0, 180, 0, -45, -135, -90, 45, 135, 90, 0},
		         {90, 45, 135, 0, 0, 180, -90, -45, -135, 0},
		         {135, 90, 45, 180, 0, 0, -135, -90, -45, 0},
		         {45, 135, 90, 0, 180, 0, -45, -135, -90, 0},
		         {-90, -45, -135, 90, 45, 135, 0, 0, 180, 0},
		         {-135, -90, -45, 135, 90, 45, 180, 0, 0, 0},
		         {-45, -135, -90, 45, 135, 90, 0, 180, 0, 0},
		         {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};


  // This main function causes the robot to wander around avoiding obstacles and
  // calculating its position based on identifying changes in grid locations
  public static void main(String[] args) {
 
    Robot robot = new Robot();
    int timeStep = (int) Math.round(robot.getBasicTimeStep());

    // Get the motors
    Motor leftMotor = robot.getMotor("left wheel motor");
    Motor rightMotor = robot.getMotor("right wheel motor");
    leftMotor.setPosition(Double.POSITIVE_INFINITY);// indicates continuous rotation servo
    rightMotor.setPosition(Double.POSITIVE_INFINITY);// indicates continuous rotation servo
    leftMotor.setVelocity(0);
    rightMotor.setVelocity(0);

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

    // Get the Centered ground sensor
    DistanceSensor  groundSensor = null;
    int   numDevices = robot.getNumberOfDevices();
    for (int i=0; i<numDevices; i++) {
      Device d = (robot.getDeviceByIndex(i));
      if (d.getName().equals("gs1")) {
        groundSensor = (DistanceSensor)d;
        groundSensor.enable(timeStep);
      }
    }
    
    // Initialize the logic variables for turning
    byte    lastTurn = STRAIGHT;
    byte    desiredTurn = STRAIGHT;
    byte    turnDecision;
    double  leftSpeed, rightSpeed;

    // Initialize the variables for position estimation
    int x = 0, y = 0, a = 90;
    int prevTile = 7;
    double[] curTile = new double[5];
    int counter = 0;
    
    // Go forever, calculting position as moving
    while (robot.step(timeStep) != -1) {
      // SENSE: Read the distance sensors
      boolean leftAheadObj  = leftAheadSensor.getValue() > 80;
      boolean leftAngleObj  = leftAngledSensor.getValue() > 80;
      boolean rightAheadObj = rightAheadSensor.getValue() > 80;
      boolean rightAngleObj = rightAngledSensor.getValue() > 80;
      boolean leftSideObj   = leftSideSensor.getValue() > 80;
      boolean rightSideObj  = rightSideSensor.getValue() > 80;
      
      
      
      // SENSE: Read the ground sensor and get the grid color index
      // ... INSERT YOUR CODE HERE ...
      double reading = groundSensor.getValue();
      curTile[counter%curTile.length] = reading;
      double tileAvg = 0;
      for (int i = 0; i < curTile.length; i++) {
      
        tileAvg += curTile[i];
      
      }
      tileAvg /= curTile.length;
      tileAvg = reading;
      //System.out.println(tileAvg + "    " + reading);
      
      
      double curMin = 15;
      int minInd = 9;
      for (int i = 0; i < calibrationValues.length; i++) {
      
        double curVal = Math.abs(tileAvg - calibrationValues[i]);
        if (curVal < curMin) {
        
          curMin = curVal;
          minInd = i;
          //System.out.println(curMin);
          
        }
      
      
      }
      
      if (minInd != 9 && minInd != prevTile) {
      
        x += X_OFFSETS[prevTile][minInd] * 10;
        y += Y_OFFSETS[prevTile][minInd] * 10;
        
        a = A_OFFSETS[prevTile][minInd];
        
        if (a < -180) {
          a += 360;
        } else if (a > 180) {
          a -= 360;
        }
        
        
        //System.out.println(minInd);
      
      }
      
      

      // THINK: Check for obstacle and decide how we need to turn
      switch(desiredTurn) {
        case SPIN_LEFT: 
          if (!rightAheadObj && !rightAngleObj && !leftAheadObj && !leftAngleObj)
            desiredTurn = STRAIGHT;
          break;
        case SPIN_RIGHT: 
          if (!leftAheadObj && !leftAngleObj && !rightAheadObj && !rightAngleObj)
            desiredTurn = STRAIGHT;
          break;
        case CURVE_LEFT:
          if (!rightSideObj || rightAheadObj || rightAngleObj || leftAheadObj || leftAngleObj)
            desiredTurn = STRAIGHT;
          break;
        case CURVE_RIGHT: 
          if (!leftSideObj || rightAheadObj || rightAngleObj || leftAheadObj || leftAngleObj)
            desiredTurn = STRAIGHT;
          break;
        default: 
          if (rightAheadObj || leftAheadObj) {
            if ((int)(Math.random()*2) % 2 == 0)
              desiredTurn = SPIN_LEFT;
            else
              desiredTurn = SPIN_RIGHT;
          }
          else if (rightAngleObj)
            desiredTurn = SPIN_LEFT;
          else if (leftAngleObj)
            desiredTurn = SPIN_RIGHT;
          else if (leftSideObj && rightSideObj)
            desiredTurn = STRAIGHT;
          else if (rightSideObj)
            desiredTurn = CURVE_LEFT;
          else if (leftSideObj)
            desiredTurn = CURVE_RIGHT;
          break;
      }
      
      
      
      // THINK: Compute and display the (x,y) location and the estimated angle
      // ... INSERT YOUR CODE HERE ...
      
      System.out.println("(x, y, a) = (" + x + ", " + y + ", " + a + ")");

      prevTile = minInd;

      // REACT: Move motors accordingly
      switch(desiredTurn) {
        case SPIN_LEFT:
          leftSpeed  = -0.5 * MAX_SPEED;
          rightSpeed = 0.5 * MAX_SPEED;
          break;
        case SPIN_RIGHT:
          leftSpeed  = 0.5 * MAX_SPEED;
          rightSpeed = -0.5 * MAX_SPEED;
          break;
        case CURVE_LEFT:
          leftSpeed  = 0.75 * MAX_SPEED;
          rightSpeed = 1 * MAX_SPEED;
          break;
        case CURVE_RIGHT:
          leftSpeed  = 1 * MAX_SPEED;
          rightSpeed = 0.75 * MAX_SPEED;
          break;
        default:
          leftSpeed  = 1 * MAX_SPEED;
          rightSpeed = 1 * MAX_SPEED;
          break;
      }
      leftMotor.setVelocity(leftSpeed);
      rightMotor.setVelocity(rightSpeed);
      
      counter++;
    }
  }
  
  
  
}
