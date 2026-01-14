package frc.lib.infrastructure;

import edu.wpi.first.wpilibj.RobotBase;
import java.net.InetAddress;
import java.net.NetworkInterface;

public class RobotInstance {

  public static String getMacAddressStr() {
    // Return fake MAC address for all simulated robots
    if (RobotBase.isSimulation()) {
      return "Simulator";
    }

    // Otherwise get real address
    InetAddress address;
    try {
      address = InetAddress.getLocalHost();
      NetworkInterface networkInterface = NetworkInterface.getByInetAddress(address);
      byte[] mac = networkInterface.getHardwareAddress();
      StringBuilder stringBuilder = new StringBuilder();
      for (int i = 0; i < mac.length; i++) {
        stringBuilder.append(String.format("%02X%s", mac[i], (i < mac.length - 1) ? "-" : ""));
      }
      return stringBuilder.toString();
    } catch (Exception e) {
      e.printStackTrace();
      return "Error:" + e.toString();
    }
  }
}
