// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.lib.simulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import frc.lib.infrastructure.RobotContainer;
import java.util.Optional;
import java.util.function.Supplier;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class SimConstants {

  /* Simulation mode, initial position, and robot container */

  /*
   * This gets the value of environment variable AKIT_SIM_MODE otherwise sets
   * SIM_MODE to
   * Mode.SIM
   */
  public static final Mode SIM_MODE =
      Optional.ofNullable(System.getenv("AKIT_SIM_MODE")).map(Mode::valueOf).orElse(Mode.SIM);

  public static final Pose2d SIM_INITIAL_FIELD_POSE = new Pose2d(3, 3, new Rotation2d());
  public static final Supplier<RobotContainer> SIM_ROBOT_SUPPLIER =
      () -> new frc.robot26.RobotContainer();

  public static final boolean IS_MAC =
      System.getProperty("os.name").toLowerCase().indexOf("mac") >= 0;

  /* DO NOT CHANGE */
  public static final Mode CURRENT_MODE = RobotBase.isReal() ? Mode.REAL : SIM_MODE;

  public enum Mode {
    /** Running on a real robot. */
    REAL(null),

    /** Running a physics simulator. */
    SIM("SIM"),

    /** Replaying from a log file. */
    REPLAY("REPLAY");

    Mode(String env) {}
  }
}
