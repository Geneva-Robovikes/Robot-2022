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

    // ~~~~~~ values from system identification ~~~~~~ //
    public static final double ksVolts = 0.55274;
    public static final double kvVoltSecondsPerMeter = 2.2988;
    public static final double kaVoltSecondsSquaredPerMeter = 0.37267;
    public static final double kPDriveVel = .0000014057;
    public static final double kTrackWidthMeters = 0.50165;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackWidthMeters);
    
    // ~~~~~~ Make these values the same as the ones in pathweaver.json ~~~~~~ //
    public static final double kMaxSpeedMetersPerSecond = 1;
    public static final double kMaxAccelerationMetersPerSecondSquared = 0.4;
    
    // ~~~~~~ Reccomended robot values ~~~~~~ //
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
}