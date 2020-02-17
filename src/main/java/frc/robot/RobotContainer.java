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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.CellManipulation;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Lighting;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
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
  private final Turret m_turret = new Turret();
  //private final Lighting m_lighting = new Lighting();

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

    //TODO: Update default command to change between drive mode and vision mode
    m_vision.setDefaultCommand(new RunCommand( () -> m_vision.updateStatus(), m_vision) );

    //TODO: Update default command to add m_shooter.setHoodAngle(getDesiredShooterHoodAngle());
    m_shooter.setDefaultCommand(
      new RunCommand( () -> m_shooter.setSetpoint(getDesiredShooterSpeed()), m_shooter) );

    //TODO: Update default command to select between manual and vision movement
    m_turret.setDefaultCommand(
      new RunCommand( () -> m_turret.setSpeed(getDesiredTurretSpeed()), m_turret) );

    m_cellManipulation.setDefaultCommand(
      new RunCommand( () -> {
        double intakeSpeed = .4;
        double queueSpeed = .45;
        double conveyorSpeed = .65;
        if (m_copilotDS.getRawButton(OIConstants.kIntakeOutPort)){
          m_cellManipulation.setIntake(-intakeSpeed);
          m_cellManipulation.setQueue(-queueSpeed);
          m_cellManipulation.setConveyor(-conveyorSpeed);
        }
        else if (m_copilotDS.getRawButton(OIConstants.kIntakeInPort)){
          if (m_cellManipulation.getHighestSensorActive() < 2){
            //get cells into the conveyor if there are non there yet
            m_cellManipulation.setIntake(intakeSpeed);
            m_cellManipulation.setQueue(queueSpeed);
            m_cellManipulation.setConveyor(conveyorSpeed);
          }
          else if (m_cellManipulation.getHighestSensorActive() < 5){
            if(m_cellManipulation.getBottomSensor()){
              //move cells up if there is space in the conveyor and there is a cell in the queue
              m_cellManipulation.setIntake(intakeSpeed);
              m_cellManipulation.setQueue(queueSpeed);
              m_cellManipulation.setConveyor(conveyorSpeed);
            }
            else{
              //stop the conveyor until there is a cell in the to be queued
              m_cellManipulation.setIntake(intakeSpeed);
              m_cellManipulation.setQueue(queueSpeed);
              m_cellManipulation.setConveyor(0);
            }
          }
          else{
            if(m_cellManipulation.getBottomSensor()){
              //reverse intake since the conveyor is full and there is a cell in the queue
              m_cellManipulation.setIntake(-intakeSpeed);
              m_cellManipulation.setQueue(0);
              m_cellManipulation.setConveyor(0);
            }
            else{
              //queue the last cell
              m_cellManipulation.setIntake(intakeSpeed);
              m_cellManipulation.setQueue(queueSpeed);
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

    //Shoot trigger
    new JoystickButton(m_rightJoystick, OIConstants.kshootPort).whenHeld( new RunCommand( () -> {
        if (m_copilotDS.getRawButton(OIConstants.kIntakeOutPort)){
          m_cellManipulation.setIntake(-.4);
        }
        else if(m_copilotDS.getRawButton(OIConstants.kIntakeInPort)){
          m_cellManipulation.setIntake(.4);
        }
        else{
          m_cellManipulation.setIntake(0);
        }

        if(m_shooter.upToSpeed()){
          m_cellManipulation.setQueue(.45);
          m_cellManipulation.setConveyor(.65);
        }
        else{
          m_cellManipulation.setQueue(0);
          m_cellManipulation.setConveyor(0);
        }
      }, m_cellManipulation)
    );
  }

  public double getDesiredTurretSpeed(){
    //TODO: Tune
    double turretStickX = m_copilotDS.getRawAxis(OIConstants.kTurretJoystickXPort);
    if(turretStickX < 0.03){
      return -0.5;
    }
    else if(turretStickX > 0.07){
      return 0.5;
    }
    else{
      return 0;
    }
  }

  public double getDesiredShooterSpeed(){
    double speed;
    double knobValue = m_copilotDS.getRawAxis(1);
    double threshold = 0.010;
    
    //If Shooter Knob is at 1
    if(knobValue < 0 + threshold){
        speed = 0;
    }
    //If Shooter Knob is at 2
    else if(knobValue >= 0.024 - threshold && knobValue < 0.024 + threshold){
        speed = ShooterConstants.kShooterSpeed1;
    }
    //If Shooter Knob is at 3
    else if(knobValue >= 0.055 - threshold && knobValue < 0.055 + threshold){
        speed = ShooterConstants.kShooterSpeed2;
    }
    //If Shooter Knob is at 4
    else if(knobValue >= 0.087 - threshold && knobValue < 0.087 + threshold){
        speed = ShooterConstants.kShooterSpeed3;
    }
    else
    {
        speed = 0;
    }
  
    return speed;
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
