/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CellManipulation;

public class Shoot extends CommandBase {
  private CellManipulation m_cellManipulation;
  public Shoot(CellManipulation cellManipulation) {
    m_cellManipulation = cellManipulation;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(cellManipulation);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_cellManipulation.setIntake(0);
    m_cellManipulation.setQueue(.35);
    m_cellManipulation.setConveyor(.45);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_cellManipulation.setIntake(0);
    m_cellManipulation.setQueue(0);
    m_cellManipulation.setConveyor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
