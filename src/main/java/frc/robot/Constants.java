// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
* The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
* constants. This class should not be used for any other purpose. All constants should be declared
* globally (i.e. public static). Do not put anything functional in this class.
*
* <p>It is advised to statically import this class (or one of its inner classes) wherever the
* constants are needed, to reduce verbosity.
*/
public final class Constants {
  public static final class DrivetrainConstants {
    public static final double kS = 0.16734; // Volts
    public static final double kV = 2.6713;  // Volt Seconds per Meter
    public static final double kA = 0.27882; // Volt Seconds Squared per Meter
    
    public static final double kPVel = 6.9201; // Volt Seconds per Meter
    
    public static final double TRACK_WIDTH = 0.5461; // Meters
    public static final DifferentialDriveKinematics KINEMATICS = 
        new DifferentialDriveKinematics(TRACK_WIDTH);
    
    public static final double MAX_VELOCITY = 3.6;
    public static final double MAX_ACCEL = 3;
    

    public static final double GEAR_RATIO = 364d / 36d;

    public static final double WHEEL_CIRCUMFRENCE = 0.489;
    public static final double B = 1.4d; //2
    public static final double ZETA = 0.7d;
  }
}
