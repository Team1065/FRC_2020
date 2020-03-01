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
        public static final int kLeftFrontMotorPort = 1;//SPARK Max
        public static final int kLeftBackMotorPort = 3;//SPARK Max
        public static final int kRightFrontMotorPort = 2;//SPARK Max
        public static final int kRightBackMotorPort = 4;//SPARK Max

        // 80 is default for NEO on drivetrain. can be set more conservative or agressive if needed
        public static final int kCurrentLimit  = 60;

        public static final int kEncoderCPR = 42; //NEO encoder 42 CPR
        public static final double kWheelDiameterInches = 6.0; //TODO: get actual value
        public static final double kWheelGearRatio = 1.0; //TODO: get actual value. Need gear ratio since the encoder is attached to the motor
        public static final double kEncoderDistancePerPulse =
            // Assumes the encoders are directly mounted on the wheel shafts
            ((kWheelDiameterInches * Math.PI) / (double) kEncoderCPR) * kWheelGearRatio;

        public static final double kStraightDriveP = 0.1;
        public static final double kStraightDriveI = 0;
        public static final double kStraightDriveD = 0.005;
    }

    public static final class ShooterConstants {
        public static final int kMasterMotorPort = 5;//SPARK Max
        public static final int kSlaveMotorPort = 6;//SPARK Max
        public static final int kFeederMotorPort = 7;//SPARK Max
        public static final int kHoodServo1Port = 0;//PWM
        public static final int kHoodServo2Port = 1;//PWM

        public static final int kEncoderCPR = 42; //NEO encoder 42 CPR
        public static final double kP = 0.000230;//TODO: Tune
        public static final double kI = 0.000001;
        public static final double kD = 0;
        public static final double kIZone = 330;
        public static final double kFF = 0.000165;
        public static final double kMaxOutput = 1;
        public static final double kMinOutput = 0;
        public static final double kAllowedError  = 300;//TODO: Tune

        public static final double kShooterSpeed1 = 3000;
        public static final double kShooterSpeed2 = 4000;
        public static final double kShooterSpeed3 = 5500;

        public static final double kShooterHoodAngle1 = 0;//TODO: Tune
        public static final double kShooterHoodAngle2 = 0;//TODO: Tune
        public static final double kShooterHoodAngle3 = 0;//TODO: Tune


        public static final double kDefaultHoodAngle  = 0;//TODO: Tune

        public static final int kCurrentLimit  = 80;//TODO: Might need to tune
    }

    public static final class CellManiputalionConstants {
        public static final int kIntakeMotorPort = 1;//VictorSPX
        public static final int kQueueMotorPort = 2;//VictorSPX
        public static final int kConveyorMotorPort = 3;//VictorSPX

        public static final int kTopSensorPort = 4;
        public static final int kMiddleTopSensorPort = 3;
        public static final int kMiddleSensorPort = 2;
        public static final int kMiddleBottomSensorPort = 1;
        public static final int kBottomSensorPort = 0;
    }

    public static final class TurretConstants {
        public static final int kTurretMotor = 4;//VictorSPX

        public static final int kLeftSensorPort = 6;
        public static final int kRightSensorPort = 5;

        public static final double kP  = 0.05;//TODO: Tune
    }

    public static final class ClimberConstants {
        public static final int kClimberMasterMotor = 5;//VictorSPX
        public static final int kClimberSlaveMotor = 6;//VictorSPX

        public static final int kBottomSensorPort = 7;

        public static final double kP  = 0.07;//TODO: Tune
    }

    public static final class OIConstants {
        public static final int kLeftjoystickPort = 0;
        public static final int kRightjoystickPort = 1;
        public static final int kCopilotDsPort = 2;

        //right joystick inputs
        public static final int kstraightDrivePort = 2;
        public static final int kshootPort = 1;

        //copilot inputs
        public static final int kVisionSwitchPort = 1;//old climber switch
        public static final int kClimberSwitchPort = 2;//old Compressor switch
        public static final int kIntakeInPort = 4;
        public static final int kIntakeOutPort = 5;
        public static final int kTurretJoystickXPort = 4;
        public static final int kClimberJoystickYPort = 2;
    }

    public static final class LightingConstants {

        public static final int backLeft_LEDPin = 5;
        public static final int backRight_LEDPin = 6;
        public static final int frontLeft_LEDPin = 7;
        public static final int frontRight_LEDPin = 8;
        public static final int turret_LEDPin = 9;

        public static final int LED_Count_BackLeft = 39;
        public static final int LED_Count_BackRight = 39;
        public static final int LED_Count_FrontLeft = 40;
        public static final int LED_Count_FrontRight = 40;
        public static final int LED_Count_Turret = 40;

    }
}