package frc.lib.replay;

import edu.wpi.first.util.datalog.DataLogIterator;
import edu.wpi.first.util.datalog.DataLogReader;
import edu.wpi.first.util.datalog.DataLogRecord;
import java.io.IOException;
import java.util.HashMap;

public final class WPILogReadMACAddress {

  public static final String EXTRA_HEADER = "AdvantageKit";
  public static final String ENTRY_METADATA = "{\"source\":\"AdvantageKit\"}";

  /**
   * Intended only to be used a single time to retrieve the MAC address of the robot used during
   * this log. This is for robot code selection during replay
   *
   * @param filename The name of the log file, can use log path given by AdvantageKit's find log
   *     file
   * @return The MAC address, null if could not find or load.
   */
  public static String get(String filename) {
    // Open log file
    DataLogReader reader;
    try {
      reader = new DataLogReader(filename);
    } catch (IOException e) {
      return null;
    }

    // Exit if invalid
    if (!reader.isValid()) {
      return null;
    } else if (!reader.getExtraHeader().equals(EXTRA_HEADER)) {
      return null;
    }

    // Create iterator and reset
    String macAddress = null;
    DataLogIterator iterator = reader.iterator();
    HashMap<Integer, String> entryIdToValueMap = new HashMap<>();
    HashMap<String, Integer> entryNametoIdMap = new HashMap<>();

    while (iterator.hasNext()) {
      DataLogRecord record;
      try {
        record = iterator.next();
      } catch (Exception e) {
        break;
      }

      // For some reason control records happen after start records?
      // Only enter "/RealMetadata" entries since that's where we'll keep
      // RobotMACAddress
      if (record.isControl()
          && record.isStart()
          && record.getStartData().metadata.equals(ENTRY_METADATA)
          && record.getStartData().name.startsWith("/RealMetadata")) {
        entryNametoIdMap.put(record.getStartData().name, record.getStartData().entry);
        continue;
      } else {
        // All entries can be gotten as a string, only add valid strings to the map
        // since we're
        // searching for a MAC address
        String entryAsString = record.getString();
        if (!entryAsString.trim().isEmpty()) {
          entryIdToValueMap.put(record.getEntry(), entryAsString);
        }
      }
    }

    var id = entryNametoIdMap.get("/RealMetadata/RobotMACAddress");
    if (id != null) {
      macAddress = entryIdToValueMap.get(id);
    }

    return macAddress;
  }
}
