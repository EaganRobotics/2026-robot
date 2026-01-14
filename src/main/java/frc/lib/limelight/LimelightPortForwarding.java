package frc.lib.limelight;

import edu.wpi.first.net.PortForwarder;

public class LimelightPortForwarding {
  public static void setup() {
    // https://docs.limelightvision.io/docs/docs-limelight/getting-started/best-practices
    for (int port = 5800; port <= 5807; port++) {
      PortForwarder.add(port, "limelight.local", port);
    }
  }
}
