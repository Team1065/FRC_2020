/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveConstants {
        public static final int kLeftFrontMotorPort = 1;
        public static final int kLeftBackMotorPort = 3;
        public static final int kRightFrontMotorPort = 2;
        public static final int kRightBackMotorPort = 4;

        // 80 is default for NEO on drivetrain. can be set more conservative or agressive if needed
        public static final int kCurrentLimit  = 80;

        public static final int kEncoderCPR = 42; //NEO encoder 42 CPR
        public static final double kWheelDiameterInches = 6.0; //TODO: get actual value
        public static final double kWheelGearRatio = 1.0; //TODO: get actual value. Need gear ratio since the encoder is attached to the motor
        public static final double kEncoderDistancePerPulse =
            // Assumes the encoders are directly mounted on the wheel shafts
            ((kWheelDiameterInches * Math.PI) / (double) kEncoderCPR) * kWheelGearRatio;

        public static final double kStraightDriveP = 0.1;//TODO: Tune
        public static final double kStraightDriveI = 0;
        public static final double kStraightDriveD = 0;
    }

    public static final class OIConstants {
        public static final int kLeftjoystickPort = 0;
        public static final int kRightjoystickPort = 1;
        public static final int kCopilotDsPort = 2;

        public static final int kIntakeInPort = 4;
        public static final int kIntakeOutPort = 4;
    }

    public static final class CellManiputalionConstants {
        public static final int kIntakeMotorPort = 1;
        public static final int kQueueMotorPort = 2;
        public static final int kConveyorMotorPort = 3;

        public static final int kTopSensorPort = 4;
        public static final int kMiddleTopSensorPort = 3;
        public static final int kMiddleSensorPort = 2;
        public static final int kMiddleBottomSensorPort = 1;
        public static final int kBottomSensorPort = 0;
}
