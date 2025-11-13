import java.util.*;
import java.awt.*;
import java.io.*;

public class Trace {
  private static final int VERTEX_RADIUS = 3;
  private static final int VERTEX_DIAMETER = VERTEX_RADIUS*2;
  
  private float  scale = 1.0f;
  private int    xMax=-9999, yMax=-9999, xMin=9999, yMin=9999;
  
  
  private ArrayList<Point>   actualPoints;
  private ArrayList<Point>   estimatePoints;

  public Trace() { 
    actualPoints = new ArrayList<Point>();
    estimatePoints = new ArrayList<Point>();
  }

  public void appendBoundaryPoint(int x, int y) {
    actualPoints.add(new Point(x, y));
    if (x > xMax) xMax = x;
    else if (x < xMin) xMin = x;
    if (y > yMax) yMax = y;
    else if (y < yMin) yMin = y;
  }
  
  public void appendEstimatePoint(int x, int y) {
    estimatePoints.add(new Point(x, y));
    if (x > xMax) xMax = x;
    else if (x < xMin) xMin = x;
    if (y > yMax) yMax = y;
    else if (y < yMin) yMin = y;
  }

  public int transX(int val) {
    return (int)((val - xMin + VERTEX_DIAMETER) * scale);
  }
  public int transY(int val) {
    return (int)((val - yMin + VERTEX_DIAMETER) * scale);
  }

  public void draw(Graphics aPen) {
    // Adjust the scale to fit everything into the window
    int xDiff = xMax - xMin;
    int yDiff = yMax - yMin;
    float xScale = 1.0f, yScale = 1.0f;
    if (xDiff > 0)
      scale = (TrackerApp.WindowWidth) / (float)(xDiff+20); // 20 pixel margin
    if (yDiff > 0) {
      yScale = (TrackerApp.WindowHeight) / (float)(yDiff+20);
      if (yScale > xScale)
        scale = yScale;
    }

    aPen.setColor(Color.lightGray);
    aPen.fillRect(0, 0, TrackerApp.WindowWidth, TrackerApp.WindowHeight);
  
    if (actualPoints.size() > 1) {
      Polygon p = new Polygon();
      for (int i=0; i<actualPoints.size(); i++) 
        p.addPoint(transX(actualPoints.get(i).x), transY(actualPoints.get(i).y));             
      aPen.setColor(Color.white);
      aPen.fillPolygon(p);
    }
    // Now draw the estimate if there is more than one point
    if (estimatePoints.size() > 1) {
      aPen.setColor(Color.black);
      for (int i=0; i<estimatePoints.size()-1; i++) 
        aPen.drawLine(transX(estimatePoints.get(i).x), transY(estimatePoints.get(i).y), 
                      transX(estimatePoints.get(i+1).x), transY(estimatePoints.get(i+1).y));
    }
    aPen.setColor(Color.red);
    try {
      for (Point p: estimatePoints)
        aPen.fillOval(transX(p.x)-VERTEX_RADIUS, transY(p.y)-VERTEX_RADIUS, 
                      VERTEX_DIAMETER, VERTEX_DIAMETER);
      aPen.setColor(Color.black);
      for (Point p: estimatePoints)
        aPen.drawOval(transX(p.x)-VERTEX_RADIUS, transY(p.y)-VERTEX_RADIUS, 
                      VERTEX_DIAMETER, VERTEX_DIAMETER);
    } catch (java.util.ConcurrentModificationException ex) {
      // Do nothing please
    }
  }
}
