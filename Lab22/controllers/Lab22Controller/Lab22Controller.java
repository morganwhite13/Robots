// Author: Morgan White SN: 101184799 && Hilliard Sample SN: 191151944
import java.awt.Point; 
import java.util.ArrayList;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Compass;
import com.cyberbotics.webots.controller.Robot;
import com.cyberbotics.webots.controller.Lidar;
import com.cyberbotics.webots.controller.Field;
import com.cyberbotics.webots.controller.Node;
import com.cyberbotics.webots.controller.Supervisor;
import com.cyberbotics.webots.controller.Display;

public class Lab22Controller {

  //private static final double  GRIPPER_MOTOR_MAX_SPEED = 0.1;
  private static final float   ADJACENCY_TOLERANCE = 0.15f; // 15cm to be considered on same wall
  private static final float   TOO_CLOSE_DISTANCE = 0.52f; // 50cm to be considered too close
  private static final float   TOO_CLOSE_SIDE_DISTANCE = 0.30f; // 25cm to be considered too close
  private static final int     MAX_SPEED    = 12;
  private static final int     WORLD_WIDTH  = 1000;
  private static final int     WORLD_HEIGHT = 1000;
  
  private static Supervisor  robot;
  private static Compass     compass;
  private static Motor       leftMotor;
  private static Motor       rightMotor;
  
  private static Field       TranslationField;
  
  // The app for displaying the lidar data
  private static LidarDisplayApp  displayApp;

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
	  
  
  // This is where it all begins
  public static void main(String[] args) {
    // create the Robot instance.
    robot = new Supervisor();
    
    // Code required for being able to get the robot's location
    Node    robotNode = robot.getSelf();
    TranslationField = robotNode.getField("translation");
    
    // get the time step of the current world.
    int TimeStep = (int) Math.round(robot.getBasicTimeStep());
    
    // Set up the display app for the lidar
    displayApp = new LidarDisplayApp(500, 500, robot.getDisplay("display"));

    // You should insert a getDevice-like function in order to get the
    // instance of a device of the robot. Something like:
    leftMotor = robot.getMotor("left wheel");
    rightMotor = robot.getMotor("right wheel");
    leftMotor.setPosition(Double.POSITIVE_INFINITY);
    rightMotor.setPosition(Double.POSITIVE_INFINITY);
    leftMotor.setVelocity(0);
    rightMotor.setVelocity(0); 
    
    // Prepare the Compass sensor
    compass = robot.getCompass("compass");
    compass.enable(TimeStep);
    
    //  Prepare the Lidar sensor
    Lidar lidar = new Lidar("Sick LMS 291");
    lidar.enable(TimeStep);

    //int lidarWidth = lidar.getHorizontalResolution();
    //int lidarLayers = lidar.getNumberOfLayers();
    //double maxRange = lidar.getMaxRange();
    
    /*System.out.println("Horizontal Field of View = " + lidar.getFov()*180/Math.PI); // 180
    System.out.println("Horizontal Resolution = " + lidarWidth); // 180
    System.out.println("Number of Layers = " + lidarLayers);     // 1
    System.out.println("Maximum Range = " + maxRange);           // 80*/
    
    float lidarValues[] = null;
    
    // Run the robot
    while (robot.step(TimeStep) != -1) {
      // Get the actual robot location
      double values[] = TranslationField.getSFVec3f();
      int x = (int)(values[0]*100) + WORLD_WIDTH/2;
      int y = -(int)(values[2]*100) + WORLD_HEIGHT/2;
      lidarValues = lidar.getRangeImage();
      displayApp.applyLidarReading(lidarValues, x, y, getCompassReadingInDegrees());
      
      
      // WRITE YOUR CODE HERE
      int leftx = -1;
      int lefty = -1;
      for (int i = 1; i < lidarValues.length; i++) {
      
        // int px = (int) (lidarValues[i-1] * 100 * Math.cos(Math.toRadians(getCompassReadingInDegrees() + 90 - i-1)));
        // int py = (int) (lidarValues[i-1] * 100 * Math.sin(Math.toRadians(getCompassReadingInDegrees() + 90 - i-1)));
        
        // int px2 = (int) (lidarValues[i] * 100 * Math.cos(Math.toRadians(getCompassReadingInDegrees() + 90 - i)));
        // int py2 = (int) (lidarValues[i] * 100 * Math.sin(Math.toRadians(getCompassReadingInDegrees() + 90 - i)));
      
        //if ((lidarValues[i] - lidarValues[i+1] > ADJACENCY_TOLERANCE || lidarValues[i+1] - lidarValues[i] > ADJACENCY_TOLERANCE) && left == null) {
        // if ((java.awt.geom.Point2D.distance(px,py,px2,py2) > ADJACENCY_TOLERANCE) && leftx == -1) {
        // if (lidarValues[i] - lidarValues[i-1] > ADJACENCY_TOLERANCE && java.awt.geom.Point2D.distance(px,py,px2,py2) >= 80 && java.awt.geom.Point2D.distance(px,py,px2,py2) <= 100) {
        if (lidarValues[i] - lidarValues[i-1] > ADJACENCY_TOLERANCE) {
        
          leftx = (int) (lidarValues[i-1] * 100 * Math.cos(Math.toRadians(getCompassReadingInDegrees() + 90 - i-1)));
          lefty = (int) (lidarValues[i-1] * 100 * Math.sin(Math.toRadians(getCompassReadingInDegrees() + 90 - i-1)));
          break;
          // System.out.println("MADE IT TO LEFT " + leftx + " y is " + lefty);
        
        
        }
      
      
      }
      
      int rightx = -1;
      int righty = -1;
      for (int i = lidarValues.length - 2; i > 0 ; i--) {
      
        // int px = (int) (lidarValues[i+1] * 100 * Math.cos(Math.toRadians(getCompassReadingInDegrees() + 90 - i+1)));
        // int py = (int) (lidarValues[i+1] * 100 * Math.sin(Math.toRadians(getCompassReadingInDegrees() + 90 - i+1)));
        
        // int px2 = (int) (lidarValues[i] * 100 * Math.cos(Math.toRadians(getCompassReadingInDegrees() + 90 - i)));
        // int py2 = (int) (lidarValues[i] * 100 * Math.sin(Math.toRadians(getCompassReadingInDegrees() + 90 - i)));
      
        // //if ((lidarValues[i] - lidarValues[i+1] > ADJACENCY_TOLERANCE || lidarValues[i+1] - lidarValues[i] > ADJACENCY_TOLERANCE) && left == null) {
        // if ((java.awt.geom.Point2D.distance(px,py,px2,py2) > ADJACENCY_TOLERANCE) && rightx == -1) {
        // if ((java.awt.geom.Point2D.distance(px,py,px2,py2) > ADJACENCY_TOLERANCE)) {
        
          
          // rightx = px;
          // righty = py;
          // System.out.println("MADE IT TO RIGHT " + righty + " x is " + rightx);
        
        
        // }
      
        // if (lidarValues[i] - lidarValues[i+1] > ADJACENCY_TOLERANCE) {
        
          // System.out.println(java.awt.geom.Point2D.distance(px,py,px2,py2));
        // }
        
        // if (lidarValues[i] - lidarValues[i+1] > ADJACENCY_TOLERANCE && java.awt.geom.Point2D.distance(px,py,px2,py2) >= 80 && java.awt.geom.Point2D.distance(px,py,px2,py2) <= 100) {
        if (lidarValues[i] - lidarValues[i+1] > ADJACENCY_TOLERANCE) {
        
          rightx = (int) (lidarValues[i+1] * 100 * Math.cos(Math.toRadians(getCompassReadingInDegrees() + 90 - i+1)));
          righty = (int) (lidarValues[i+1] * 100 * Math.sin(Math.toRadians(getCompassReadingInDegrees() + 90 - i+1)));
          break;
          // System.out.println("MADE IT TO LEFT " + leftx + " y is " + lefty);
        
        
        }
      
      
      }
      
      // System.out.println("MADE IT TO LEFT " + leftx + " y is " + lefty);
      
      
      
      
      
      if (leftx != -1 && rightx != -1 && java.awt.geom.Point2D.distance(leftx,lefty,rightx,righty) >= 80 && java.awt.geom.Point2D.distance(leftx,lefty,rightx,righty) <= 100) {
      
        displayApp.setDoorwayPoints(leftx, lefty, rightx, righty, x, y);
      
      
      } else {
      
          displayApp.setDoorwayPoints(0,0,0,0, x, y);
      
      
      }
      
      
      int cx = -1;
      int cy = -1;
      for (int i = 0; i < lidarValues.length; i++) {
      
        cx += (int) (lidarValues[i] * 100 * Math.cos(Math.toRadians(getCompassReadingInDegrees() + 90 - i)));
        cy += (int) (lidarValues[i] * 100 * Math.sin(Math.toRadians(getCompassReadingInDegrees() + 90 - i)));
      
      }
      cx /= lidarValues.length;
      cy /= lidarValues.length;
      cx += x;
      cy += y;
      
      
      displayApp.setWayPoint(cx, cy, x, y);
      
      if (lidarValues[179] < TOO_CLOSE_SIDE_DISTANCE) {
      
        turnAway(MAX_SPEED/2, MAX_SPEED);
      
      } else if (lidarValues[0] < TOO_CLOSE_SIDE_DISTANCE) {
        turnAway(MAX_SPEED, MAX_SPEED/2);
      
      } else if (lidarValues[90] < TOO_CLOSE_DISTANCE) {
        turnAway(MAX_SPEED/-2, MAX_SPEED/2);
      
      } else {
      
        moveFrom(x, y, cx, cy);
      
      
      }
      
    }
  }
}
