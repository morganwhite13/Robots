import java.awt.*;
import java.awt.event.*;
import javax.swing.*;

public class TrackerPanel extends JPanel {
  private static final long serialVersionUID = 99999999;
  
  private Trace   aTrace;  // The model (i.e. the trace)

  public TrackerPanel() {
    aTrace = new Trace();
    setBackground(Color.white);
  }
  
  public Dimension getPreferredSize() {
    return new Dimension(TrackerApp.WindowWidth, TrackerApp.WindowHeight);
  }

  public Trace getTrace() { return aTrace; }
  public void setTrace(Trace t) { aTrace = t; update(); }

  public void update() {
    repaint();
  }

  // This is the method that is responsible for displaying the trace
  public void paintComponent(Graphics aPen) {
    super.paintComponent(aPen);
    aTrace.draw(aPen);
  }
}
