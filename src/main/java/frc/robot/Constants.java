// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class ArmSubsystemConstants {
    public static final int ID = 4;

    public static final double GEARING = 100.0;
    public static final int CURRENT_LIMIT = 40;
    public static final boolean INVERTED = false;

    public static final double MAX_SPEED_RAD_PER_SEC = Units.degreesToRadians(180.0);
    public static final double MAX_ACCEL_RAD_PER_SEC = Units.degreesToRadians(360.0);
    public static final double POS_CONV_FACTOR = Units.rotationsToRadians(1.0) / GEARING;
    public static final double VELOCITY_CONV_FACTOR = Units.rotationsPerMinuteToRadiansPerSecond(1.0) / GEARING;

    public static final double STRAIGHT_FORWARD_STATE_RAD = 0.0;
    public static final double STRAIGHT_UP_STATE_RAD = Math.PI / 2.0;

    public static final double HARD_STOP_STATE_RAD = Math.PI;
  }
}
