// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import frc.robot.utils.ModuleConstants;
import frc.robot.utils.ModuleConstants.MotorLocation;
import frc.robot.utils.ModuleConstants.MotorType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public final class Shooter {
    public static final double SPEAKER_SPEED = .7;
    public static final double AMP_SPEED = 0.25;
    public static final double REVERSE_SPEED = -0.5;
  }

  // public final class Channel {
  //     public static final double LOAD_SPEED = -.4;
  //     public static final double SHOOT_SPEED = -.55;
  //     public static final double THROUGH_SPEED = -1;
  //     public static final double REVERSE_SPEED = .6;
  // }

  public final class Control {

    // Sensitivity for speed meter
    public static final double DIRECTIONAL_SPEED_METER_LOW = 0.25;
    public static final double DIRECTIONAL_SPEED_METER_HIGH = 4.0;
    public static final double SPIN_SPEED_METER_LOW = 0.5;
    public static final double SPIN_SPEED_METER_HIGH = 2.4;

    // Sensitivies for directional controls (XY) and spin (theta)
    public static final double JOSYSTICK_DIRECTIONAL_SENSITIVITY = 1;
    public static final double JOYSTICK_SPIN_SENSITIVITY = 2;
    public static final double JOYSTICK_X_THRESHOLD = 0.15;
    public static final double JOYSTICK_Y_THRESHOLD = 0.15;
    public static final double JOYSTICK_SPIN_THRESHOLD = 0.3;

    // Thresholds for directional controls (XY) and spin (theta)
    public static final double XBOX_DIRECTIONAL_SENSITIVITY = 1;
    public static final double XBOX_X_THRESHOLD = 0.15;
    public static final double XBOX_Y_THRESHOLD = 0.15;
    public static final double XBOX_SPIN_THRESHOLD = 0.3;

    public static final double XBOX_SPIN_ROT_THRESHOLD = 0.1;
    public static final double XBOX_SPIN_ROT_SENSITIVITY = 1.0;

    // Tablet drive constants
    public final class Tablet {
      // Will fill in later, but for now it's convenient to have it in the TabletDrive
      public static final double PRESSURE_THRESHOLD = 0.2;
      public static final double MIN_SPEED = 0.2;
      public static final double STEEPNESS =
          2.6; // Linear = 1, <1 = faster scaling, >1 = slower scaling
    }
  }

  public final class Frame {

    /**
     * Distance between wheels (width aka between left and right and length aka between front and
     * back). Used for calculating wheel locations on the robot
     */
    public static final double ROBOT_WHEEL_DISTANCE_WIDTH = 0.56515;

    public static final double ROBOT_WHEEL_DISTANCE_LENGTH = 0.56515;
  }

  public final class Differential {

    // Works out module locations
    private static final double locX = Frame.ROBOT_WHEEL_DISTANCE_WIDTH / 2;
    private static final double locY = Frame.ROBOT_WHEEL_DISTANCE_LENGTH / 2;
    public static final double locDist = Math.sqrt(locX * locX + locY * locY);

    // Gear ratios for falcon and kraken
    public static final double NEO_GEAR_RATIO = 10;

    // PID Values
    public static final double NEO_KP = 1.5;
    public static final double NEO_KI = 0;
    public static final double NEO_KD = 0.3;

    public static final double NEO_FEEDFORWARD_KS = 0.4;
    public static final double NEO_FEEDFORWARD_KV = 2.4408;

    // Wheel diameter
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(6);

    /** The max speed the robot is allowed to travel */
    public static final double robotMaxSpeed = 7.0;

    /** The max jerk of the robot below which the pose is certain (in G/s) */
    public static final double MaxPoseCertaintyJerk = 80;

    // Module Creation

    /**
     * ===================== NOTE !!!!!! ======================== THESE ARE BACKUP CONSTANTS - NOT
     * USED IF EVERYTHING WORKS EDIT deploy/differential/motors.json instead
     */

    // #region BACKUP
    public static final ModuleConstants BACKUP_leftConstants =
        new ModuleConstants(2, 1, false, MotorType.Neo);

    public static final ModuleConstants BACKUP_rightConstants =
        new ModuleConstants(4, 3, true, MotorType.Neo);

    // #endregion

    public static final ModuleConstants leftConstants =
        ModuleConstants.fromConfig(MotorLocation.Left, MotorType.Neo);

    public static final ModuleConstants rightConstants =
        ModuleConstants.fromConfig(MotorLocation.Right, MotorType.Neo);
  }
}
