import java.awt.*;
import java.awt.event.*;
import javax.swing.*;
import java.io.*;
import java.awt.Dimension;
import java.awt.event.ComponentAdapter;
import java.awt.event.ComponentEvent;

public class TrackerApp extends JFrame {
  private static final long serialVersionUID = 99999999;
  
  static int WindowWidth = 400;
  static int WindowHeight = 400;
  
  private TrackerPanel  tracePanel;

  public TrackerApp() {
    super("Tracker App");
    add(tracePanel = new TrackerPanel());

    addComponentListener(new ComponentAdapter() {
      public void componentResized(ComponentEvent e) {
        WindowWidth = e.getComponent().getSize().width;
        WindowHeight = e.getComponent().getSize().height-30;
      }
    });
    setDefaultCloseOperation(EXIT_ON_CLOSE);
    setSize(WindowWidth, WindowHeight+30);
    setVisible(true);
  }
  
  
  void addActualLocation(int x, int y) { 
    tracePanel.getTrace().appendBoundaryPoint(x, TrackerApp.WindowHeight-y);
    tracePanel.update();
  }
  void addEstimatedLocation(int x, int y) { 
    tracePanel.getTrace().appendEstimatePoint(x, TrackerApp.WindowHeight-y);
    tracePanel.update();
  }
  
  
}
