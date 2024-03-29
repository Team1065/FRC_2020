/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

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
public class AutoShootTrench extends SequentialCommandGroup {
  /**
   * Creates a new Auto2.
   */
  public AutoShootTrench(Shooter shooter, CellManipulation cellManipulation, DriveSubsystem drive) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(
      new ResetDriveSensors(drive),
      new ParallelRaceGroup(
        new IntakeInandDown(true,true,cellManipulation),
        new SequentialCommandGroup(
          new WaitCommand(.5),
          new DriveToDistance(80, drive),
          new DriveToDistance(-60, drive),
          new TurnToAngle(150, drive)
        )
      ),
      new ParallelRaceGroup(
        new SetShooterSpeed(5500, 0.7, shooter),
        new SequentialCommandGroup(
          new WaitCommand(1),
          new Shoot(cellManipulation).withTimeout(10)
        )
      )
    );
  }
}
