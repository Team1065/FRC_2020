/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveToDistance extends CommandBase {
  private DriveSubsystem m_drive;
  private PIDController distancePID = new PIDController(0.085, 0, 0);//TODO: tune
  private PIDController rotationPID = new PIDController(0.1, 0, 0.005);//TODO: tune

  public DriveToDistance(double targetDistance, DriveSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    addRequirements(drive);
    distancePID.setSetpoint(targetDistance);
    distancePID.setTolerance(5, 2);//TODO: tune
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.resetEncoder();
    rotationPID.setSetpoint(m_drive.getHeading());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = distancePID.calculate(m_drive.getDistance());
    double maxSpeed = .55;//TODO: tune
    if(speed > maxSpeed)
      speed = maxSpeed;
    else if(speed < -maxSpeed)
      speed = -maxSpeed;

    double rotspeed = rotationPID.calculate(m_drive.getHeading());
    double maxrotSpeed = .8;//TODO: tune
    if(speed > maxSpeed)
      speed = maxSpeed;
    else if(speed < -maxSpeed)
      speed = -maxSpeed;

    m_drive.arcadeDrive( speed, rotspeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return distancePID.atSetpoint();
  }
}
