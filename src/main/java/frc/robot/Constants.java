// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public final class DriveConstants{
        public static final double kRampRate = 0.65;
        public static final double kMaxDrive = 0.95;
        public static final double kDeadBand = 0.15;
        public static final int kLeftMotor01CanBusID = 10;
        public static final int kLeftMotor02CanBusID = 11;
        public static final int kRightMotor03CanBusID = 12;
        public static final int kRightMotor04CanBusID = 13;
    }

    public static final class OIConstants{
        public static final int kDriveControllerPort = 0;
        public static final int kTestAButton = XboxController.Button.kA.value;
        public static final int kTestBButton = XboxController.Button.kB.value;
    }
}
