/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class TurnToAngle extends ProfiledPIDCommand {
  /**
   * Creates a new TurnToAngle.
   */
  public TurnToAngle(double targetAngleDegrees, DriveSubsystem drive) {
    super(
        // The ProfiledPIDController used by the command
        new ProfiledPIDController(
            // The PID gains
            0.05, 0, 0.005, //TODO: Tune
            // The motion profile constraints
            new TrapezoidProfile.Constraints(100, 300)), //TODO: Tune
        // This should return the measurement
        drive::getHeading,
        // This should return the goal (can also be a constant)
        targetAngleDegrees,
        // This uses the output
        (output, setpoint) -> {
          drive.arcadeDrive(0, output);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
    // Configure additional PID options by calling `getController` here.
    // Set the controller to be continuous (because it is an angle controller)
    getController().enableContinuousInput(-180, 180);
    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    getController().setTolerance(5, 10); //TODO: Tune
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atGoal();
  }
}
