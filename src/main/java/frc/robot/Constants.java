// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
    public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
    public static final double LOOP_TIME = 0.13; //s, 20ms + 110ms sprk max velocity lag
    public static final double MAX_SPEED = Units.feetToMeters(14.5);

    public static final double coeffecitfriction = 3.5;

    // OR
    // public static final double coeffecitfriction = 2.8;

    public static final double INTAKE_SPEED = 1;
    public static final double OUTTAKE_SPEED = 1;

    public static final double INTAKE_GOING_UP_TO_DEALGEANATE = 0.025;
    public static final double INTAKE_PIVOT_DOWN_P = 0.02;
    public static final double INTAKE_PIVOT_UP_P = 0.08;
    public static final double INTAKE_PIVOT_I = 0.04;
    public static final double INTAKE_PIVOT_D = 0.002;

    public static final double CLIMB_P = 0.08;
    public static final double CLIMB_I = 0;
    public static final double CLIMB_D = 0;

    public static final double DEALGAENATOR_P = 0.85;
    public static final double DEALGAENATOR_I = 0.02;
    public static final double DEALGAENATOR_D = 0;

    // Maximum speed of the robot in meters per second, used to limit acceleration.

    //  public static final class AutonConstants
    //  {
    //
    //    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
    //    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
    //  }

    public static final class DrivebaseConstants {

        // Hold time on motor brakes when disabled
        public static final double WHEEL_LOCK_TIME = 10; // seconds
    }

    public static class OperatorConstants {

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double JOYSTICK_FLOOR = 0.3;
    public static final double TURN_CONSTANT    = 6;
  }

}
