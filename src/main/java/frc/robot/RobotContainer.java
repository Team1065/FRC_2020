/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.CellManipulation;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_drive = new DriveSubsystem();
  private final CellManipulation m_cellManipulation = new CellManipulation();
  private final Vision m_vision = new Vision();
  private final Shooter m_shooter = new Shooter();

  private final Joystick m_leftJoystick = new Joystick(OIConstants.kLeftjoystickPort);
  private final Joystick m_rightJoystick = new Joystick(OIConstants.kRightjoystickPort);
  private final Joystick m_copilotDS = new Joystick(OIConstants.kCopilotDsPort);

  private static double m_driveStraightSetpoint = 0;

  private final PIDCommand straightDriveCommand =  new PIDCommand(
      new PIDController(DriveConstants.kStraightDriveP, DriveConstants.kStraightDriveI,
                        DriveConstants.kStraightDriveD),
      // Close the loop on the turn rate
      m_drive::getHeading,
      // Setpoint is 0
      () -> m_driveStraightSetpoint,
      // Pipe the output to the turning controls
      output -> m_drive.arcadeDrive(-m_rightJoystick.getY(), output),
      // Require the robot drive
      m_drive);

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    m_drive.setDefaultCommand(
      new RunCommand( () -> m_drive.tankDrive(-m_leftJoystick.getY(), -m_rightJoystick.getY()) , m_drive) );

    //TODO: Update default command
    m_vision.setDefaultCommand(new RunCommand( () -> m_vision.updateStatus(), m_vision) );

    m_shooter.setDefaultCommand(
      new RunCommand( () -> m_shooter.tune() , m_shooter) );

    m_cellManipulation.setDefaultCommand(
      new RunCommand( () -> {
        if (m_copilotDS.getRawButton(OIConstants.kIntakeOutPort)){
          m_cellManipulation.setIntake(-.5);
          m_cellManipulation.setQueue(-.5);
          m_cellManipulation.setConveyor(-.5);
        }
        else if (m_copilotDS.getRawButton(OIConstants.kIntakeInPort)){
          if (m_cellManipulation.getHighestSensorActive() < 2){
            //get cells into the conveyor if there are non there yet
            m_cellManipulation.setIntake(.5);
            m_cellManipulation.setQueue(.5);
            m_cellManipulation.setConveyor(.5);
          }
          else if (m_cellManipulation.getHighestSensorActive() < 5){
            if(m_cellManipulation.getBottomSensor()){
              //move cells up if there is space in the conveyor and there is a cell in the queue
              m_cellManipulation.setIntake(.5);
              m_cellManipulation.setQueue(.5);
              m_cellManipulation.setConveyor(.5);
            }
            else{
              //stop the conveyor until there is a cell in the to be queued
              m_cellManipulation.setIntake(.5);
              m_cellManipulation.setQueue(.5);
              m_cellManipulation.setConveyor(0);
            }
          }
          else{
            if(m_cellManipulation.getBottomSensor()){
              //reverse intake since the conveyor is full and there is a cell in the queue
              m_cellManipulation.setIntake(-.5);
              m_cellManipulation.setQueue(0);
              m_cellManipulation.setConveyor(0);
            }
            else{
              //queue the last cell
              m_cellManipulation.setIntake(.5);
              m_cellManipulation.setQueue(.5);
              m_cellManipulation.setConveyor(0);
            }
          }
        }
        else{
          m_cellManipulation.setIntake(0);
          m_cellManipulation.setQueue(0);
          m_cellManipulation.setConveyor(0);
        }
      }
      , m_cellManipulation)
    );
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //Drive Straight
    new JoystickButton(m_rightJoystick, OIConstants.kstraightDrivePort).whenHeld( straightDriveCommand.beforeStarting( () -> m_driveStraightSetpoint = m_drive.getHeading(), m_drive ) );

    new JoystickButton(m_rightJoystick, OIConstants.kshootPort).whenHeld( new RunCommand( () -> {
        if(true){//TODO: change to shooter up to speed
          m_cellManipulation.setIntake(0);
          m_cellManipulation.setQueue(.5);
          m_cellManipulation.setConveyor(.5);
        }
        else{
          m_cellManipulation.setIntake(0);
          m_cellManipulation.setQueue(0);
          m_cellManipulation.setConveyor(0);
        }
      }, m_cellManipulation)
    );
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new InstantCommand();
  }
}
