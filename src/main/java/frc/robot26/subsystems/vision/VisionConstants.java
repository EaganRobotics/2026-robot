// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot26.subsystems.vision;

import static edu.wpi.first.units.Units.Inches;

import edu.umd.cs.findbugs.annotations.SuppressFBWarnings;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionConstants {
  // AprilTag layout
  public static final AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // Camera names, must match names configured on coprocessor
  public static final String limelightFront = "limelight-front";

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  public static final Transform3d robotToCameraBack =
      new Transform3d(0.2514, 0.2924, 0.2153, new Rotation3d(0.0, 23.2, 0.0));
  public static final Transform3d robotToCameraTop =
      new Transform3d(
          Inches.of(-4.069), Inches.of(11.615), Inches.of(28.204), new Rotation3d(0.0, 15, 6.76));

  // Basic filtering thresholds

  public static final double maxAmbiguity = 0.3;
  public static final double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static final double linearStdDevBaseline = 0.02; // Meters
  public static final double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  @SuppressFBWarnings("MS_PKGPROTECT")
  public static final double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        1.0 // Camera 1
      };

  // Multipliers to apply for MegaTag 2 observations
  public static final double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static final double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY; // No rotation
  // data
  // available
}
