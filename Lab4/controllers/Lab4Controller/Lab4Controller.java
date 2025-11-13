// Author: Morgan White (SN: 101184799) & CoAUTHOR: Gabrielle Latrielle SN: 101073284

import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Robot;
import com.cyberbotics.webots.controller.PositionSensor;

public class Lab4Controller {

  static final double  MAX_SPEED      = 1;  //6.28 is maximum but value of 1 is required for simulator to work properly for kinematics
  static final double  WHEEL_RADIUS   = 2.05; // cm 
  static final double  WHEEL_BASE     = 5.80; // cm
  
  static Robot           Epuck;
  static Motor           LeftMotor;
  static Motor           RightMotor;
  static PositionSensor  LeftEncoder;
  static PositionSensor  RightEncoder;
  static double          LeftReading, RightReading, PreviousLeft, PreviousRight;
  static int             TimeStep;


  // Set each motor to a specific speed and wait for te given amount of seconds
  // Then stop the motors and update the position sensor readings.
  private static void Move(double leftSpeed, double rightSpeed, double seconds) {
    LeftMotor.setVelocity(leftSpeed);
    RightMotor.setVelocity(rightSpeed);
    for (double time = 0.0; time<seconds; time += (TimeStep/1000.0)) {
      Epuck.step(TimeStep);
    }
    LeftMotor.setVelocity(0);
    RightMotor.setVelocity(0);
    Epuck.step(TimeStep);
    LeftReading = LeftEncoder.getValue() - PreviousLeft;
    RightReading = RightEncoder.getValue() - PreviousRight;
    PreviousLeft = LeftEncoder.getValue();
    PreviousRight = RightEncoder.getValue();
    Epuck.step(TimeStep);
  }
  
  
  public static void main(String[] args) {
    Epuck = new Robot();
    TimeStep = (int) Math.round(Epuck.getBasicTimeStep());
    
    System.out.println("Time Step = " + TimeStep);

    // Get the motors
    LeftMotor = Epuck.getMotor("left wheel motor");
    RightMotor = Epuck.getMotor("right wheel motor");
    LeftMotor.setPosition(Double.POSITIVE_INFINITY);// indicates continuous rotation servo
    RightMotor.setPosition(Double.POSITIVE_INFINITY);// indicates continuous rotation serv
    LeftMotor.setVelocity(0);
    RightMotor.setVelocity(0);
    
    // Get the encoders
    LeftEncoder = Epuck.getPositionSensor("left wheel sensor");
    RightEncoder = Epuck.getPositionSensor("right wheel sensor");
    LeftEncoder.enable(TimeStep);
    RightEncoder.enable(TimeStep);
    PreviousLeft = 0; PreviousRight = 0;
    
    // Store the (x, y) location amd angle (degrees) estimate
    double x = 0.0, y = 0.0, a = Math.toRadians(90.0), r = 0.0, td = 0.0, d = 0.0;
    int curMove = 0;
    
    a = fixA(a);
    
    // Move the robot forward for 5 seconds
    Move(MAX_SPEED, MAX_SPEED, 5);
    
    d = LeftReading * WHEEL_RADIUS;
    x += d*Math.cos(a);
    y += d*Math.sin(a);
    curMove++;
    
    a = fixA(a);
    System.out.println("(x" + curMove + ", y" + curMove + ", a" + curMove + ") = (" + (Math.round(x * 10.0) / 10.0) + ", " + (Math.round(y * 10.0) / 10.0) + ", " + (Math.round(Math.toDegrees(a) * 10.0) / 10.0) + ")");
    
    
    // Spin the robot right for 6 seconds
    Move(MAX_SPEED, -MAX_SPEED, 6);
    
    td = (RightReading - LeftReading) * WHEEL_RADIUS / WHEEL_BASE;
    a += td;
    curMove++;
    
    a = fixA(a);
    System.out.println("(x" + curMove + ", y" + curMove + ", a" + curMove + ") = (" + (Math.round(x * 10.0) / 10.0) + ", " + (Math.round(y * 10.0) / 10.0) + ", " + (Math.round(Math.toDegrees(a) * 10.0) / 10.0) + ")");
    
    
    // Curve the robot right for 10 seconds with right speed 0.2 less than full left speed
    Move(MAX_SPEED, MAX_SPEED-0.2, 10);
    
    r = WHEEL_BASE * (LeftReading/ (RightReading-LeftReading)) + WHEEL_BASE/2;
    td = (RightReading-LeftReading) * WHEEL_RADIUS / WHEEL_BASE;
    
    x = r*Math.cos(td)*Math.sin(a)+r*Math.cos(a)*Math.sin(td) + x - r*Math.sin(a);
    y = r*Math.sin(td)*Math.sin(a)-r*Math.cos(a)*Math.cos(td) + y + r*Math.cos(a);
    a += td;
    curMove++;
    
    
    a = fixA(a);
    System.out.println("(x" + curMove + ", y" + curMove + ", a" + curMove + ") = (" + (Math.round(x * 10.0) / 10.0) + ", " + (Math.round(y * 10.0) / 10.0) + ", " + (Math.round(Math.toDegrees(a) * 10.0) / 10.0) + ")");
    
    
    
    // Curve the robot left for 20 seconds with left speed 1/3 of full right speed
    Move(MAX_SPEED/3, MAX_SPEED, 20);
    
    r = WHEEL_BASE * (LeftReading/ (RightReading-LeftReading)) + WHEEL_BASE/2;
    td = (RightReading-LeftReading) * WHEEL_RADIUS / WHEEL_BASE;
    
    x = r*Math.cos(td)*Math.sin(a)+r*Math.cos(a)*Math.sin(td) + x - r*Math.sin(a);
    y = r*Math.sin(td)*Math.sin(a)-r*Math.cos(a)*Math.cos(td) + y + r*Math.cos(a);
    a += td;
    curMove++;
    
    a = fixA(a);
    System.out.println("(x" + curMove + ", y" + curMove + ", a" + curMove + ") = (" + (Math.round(x * 10.0) / 10.0) + ", " + (Math.round(y * 10.0) / 10.0) + ", " + (Math.round(Math.toDegrees(a) * 10.0) / 10.0) + ")");
    

    // Move the robot forward for 10 seconds
    Move(MAX_SPEED, MAX_SPEED, 10);
    d = LeftReading * WHEEL_RADIUS;
    x += d*Math.cos(a);
    y += d*Math.sin(a);
    curMove++;
    
    a = fixA(a);
    System.out.println("(x" + curMove + ", y" + curMove + ", a" + curMove + ") = (" + (Math.round(x * 10.0) / 10.0) + ", " + (Math.round(y * 10.0) / 10.0) + ", " + (Math.round(Math.toDegrees(a) * 10.0) / 10.0) + ")");
    
    
    // Spin the robot left for 7.5 seconds
    Move(-MAX_SPEED, MAX_SPEED, 7.5);
    
    td = (RightReading - LeftReading) * WHEEL_RADIUS / WHEEL_BASE;
    a += td;
    curMove++;
    
    a = fixA(a);
    System.out.println("(x" + curMove + ", y" + curMove + ", a" + curMove + ") = (" + (Math.round(x * 10.0) / 10.0) + ", " + (Math.round(y * 10.0) / 10.0) + ", " + (Math.round(Math.toDegrees(a) * 10.0) / 10.0) + ")");
    

    // Move the robot forward for 20 seconds
    Move(MAX_SPEED, MAX_SPEED, 20);
    d = LeftReading * WHEEL_RADIUS;
    x += d*Math.cos(a);
    y += d*Math.sin(a);
    curMove++;
    
    a = fixA(a);
    System.out.println("(x" + curMove + ", y" + curMove + ", a" + curMove + ") = (" + (Math.round(x * 10.0) / 10.0) + ", " + (Math.round(y * 10.0) / 10.0) + ", " + (Math.round(Math.toDegrees(a) * 10.0) / 10.0) + ")");
    
    

  }
  
  public static double fixA(double a) {
  
    if (a > Math.toRadians(180)) {
    
      return a - Math.toRadians(360);
    
    } else if (a < Math.toRadians(-180)) {
    
      return a + Math.toRadians(360);
    
    }
    return a;
  
  
  }
  
  
  
}
