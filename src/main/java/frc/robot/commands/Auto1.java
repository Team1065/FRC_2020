/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.CellManipulation;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class Auto1 extends SequentialCommandGroup {
  /**
   * Creates a new Auto1.
   */
  public Auto1(Shooter shooter, CellManipulation cellManipulation, DriveSubsystem drive) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(
      new ResetDriveSensors(drive),
      //set shooter speed and shoot once it is up to speed for x seconds
      /*
      new ParallelRaceGroup(
        new SetShooterSpeed(4000, 0, shooter),
        new SequentialCommandGroup(
          new WaitUntilCommand(shooter::upToSpeed),
          new Shoot(cellManipulation).withTimeout(3)
        )
      )
      ,*/
      //set intake in and drive to pick up a group of cells
      new ParallelRaceGroup(
        //new IntakeIn(cellManipulation),
        new SequentialCommandGroup(
          new TurnToAngle(-45, drive),
          new DriveToDistance(-20, drive),
          new WaitCommand(1),
          new DriveToDistance(20, drive),
          new WaitCommand(1),
          new TurnToAngle(0, drive)
        )
      )/*,
      new ParallelRaceGroup(
        new SetShooterSpeed(4000, 0, shooter),
        new SequentialCommandGroup(
          new WaitUntilCommand(shooter::upToSpeed),
          new Shoot(cellManipulation).withTimeout(3)
        )
      )*/
    );
  }
}
