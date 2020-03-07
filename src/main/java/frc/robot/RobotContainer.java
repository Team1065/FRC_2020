/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.commands.Auto1;
import frc.robot.commands.Auto2;
import frc.robot.commands.Auto3;
import frc.robot.subsystems.CellManipulation;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveSubsystem;
//import frc.robot.subsystems.Lighting;
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
  private final Climber m_climber = new Climber();
  //private final Lighting m_lighting = new Lighting();
  private final Compressor m_compressor = new Compressor();

  // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();

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

    // Add commands to the autonomous command chooser
    m_chooser.setDefaultOption("Default Auto", new Auto1(m_shooter, m_cellManipulation, m_drive));
    m_chooser.addOption("Auto1", new Auto1(m_shooter, m_cellManipulation, m_drive));
    m_chooser.addOption("Auto2", new Auto2(m_shooter, m_cellManipulation, m_drive));
    m_chooser.addOption("Auto3", new Auto3(m_shooter, m_cellManipulation, m_drive));
    SmartDashboard.putData("Auto modes", m_chooser);
    
    //default drive
    m_drive.setDefaultCommand(
      new RunCommand( () -> m_drive.tankDrive(-m_leftJoystick.getY(), -m_rightJoystick.getY()) , m_drive) );

    //default Vision
    m_vision.setDefaultCommand(new RunCommand( () -> {
      m_vision.updateStatus();
      if(m_copilotDS.getRawButton(OIConstants.kVisionSwitchPort)){
        //vision Mode
        m_vision.setDriveMode(false);
      }
      else{
        //Drive Mode
        m_vision.setDriveMode(true);
      }
    }, m_vision) );

    //default shooter
    m_shooter.setDefaultCommand(
      new RunCommand( () -> {
        //m_shooter.tune();
        m_shooter.setSetpoint(getDesiredShooterSpeed());
        m_shooter.setHoodAngle(getDesiredShooterHoodAngle());
      }, m_shooter) );

    //default turret
    m_turret.setDefaultCommand(
      new RunCommand( () ->{
        if(m_copilotDS.getRawButton(OIConstants.kVisionSwitchPort)){
          //vision Control
          double headingError = m_vision.getTargetHorizontalOffset();
          double turretSpeed = TurretConstants.kP * headingError;
          double minSpeed = 0.09;
          double treshold = 0.25;

          //if we are not within the treshold and the set speed is too low set it to min speed
          if(Math.abs(turretSpeed) < minSpeed && Math.abs(headingError) > treshold){
            if(turretSpeed < 0){
              turretSpeed = -minSpeed;
            }
            else{
              turretSpeed = minSpeed;
            }
          }
          m_turret.setSpeed(turretSpeed);
        }
        else{
          //Manual Control
          m_turret.setSpeed(getDesiredTurretSpeed());
        }
        
      }, m_turret) );

    //default cell manipulation
    m_cellManipulation.setDefaultCommand(
      new RunCommand( () -> {
        m_cellManipulation.setIntakeSolenoid(m_copilotDS.getRawButton(OIConstants.kIntakeSolenoidPort));

        //anti jam mode when left trigger and thumb button are pressed at the same time
        if(m_leftJoystick.getRawButton(OIConstants.kstraightDrivePort) && m_leftJoystick.getRawButton(OIConstants.kshootPort)){
          m_cellManipulation.antiJam();
        }
        else{
          m_cellManipulation.sensorControl(m_copilotDS.getRawButton(OIConstants.kIntakeInPort), m_copilotDS.getRawButton(OIConstants.kIntakeOutPort));
        }
      }
      , m_cellManipulation));
      
      //default climber Command
      m_climber.setDefaultCommand(
        new RunCommand( () -> {
          double commandedSpeed = getDesiredClimberSpeed();
          //if we are looking for the bottom sensor and it is active then stop the climber
          if(m_climber.lookForBottomSwitch() && m_climber.getBottomSensorActive() && commandedSpeed>0){
            commandedSpeed = 0;
          }
          //only move climber if the compressor switch is set
          if(m_copilotDS.getRawButton(OIConstants.kClimberSwitchPort)){
            m_climber.setClimberSpeed(commandedSpeed);
          }
          else{
            m_climber.setClimberSpeed(0);
          }
        }, m_climber));
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //Compressor Control
    new JoystickButton(m_copilotDS, OIConstants.kCompressorSwitchPort).whenHeld( straightDriveCommand.beforeStarting( () -> m_driveStraightSetpoint = m_drive.getHeading(), m_drive ) );

    //Drive Straight
    new JoystickButton(m_rightJoystick, OIConstants.kstraightDrivePort).whenHeld( new RunCommand( () -> m_compressor.setClosedLoopControl(false)).andThen( () -> m_compressor.setClosedLoopControl(true) ) );

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

        //Dont check if we are at speed just that it has been commanded to something other than 0 to maintain a smoother shooting stream
        //Might want to slow down the conveyor and queue speed if we are feeding the balls too quickly
        if(getDesiredShooterSpeed() > 100){
          m_cellManipulation.setQueue(.4);
          m_cellManipulation.setConveyor(.6);
        }
        else{
          m_cellManipulation.setQueue(0);
          m_cellManipulation.setConveyor(0);
        }
      }, m_cellManipulation)
    );
  }

  public double getDesiredTurretSpeed(){
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

  public double getDesiredClimberSpeed(){
    double ClimberStickY = m_copilotDS.getRawAxis(OIConstants.kClimberJoystickYPort);
    SmartDashboard.putNumber("[Climber] Stick Y", ClimberStickY);
    if(ClimberStickY < 0.035){
      return -0.8;
    }
    else if(ClimberStickY > 0.075){
      return 0.8;
    }
    else{
      return 0;
    }
  }

  public double getDesiredShooterHoodAngle(){
    double angle;
    double knobValue = m_copilotDS.getRawAxis(1);
    double threshold = 0.010;
    
    //If Shooter Knob is at 1
    if(knobValue < 0.024 - threshold){
      angle = 0;
    }
    //If Shooter Knob is at 2
    else if(knobValue >= 0.024 - threshold && knobValue < 0.024 + threshold){
      angle = ShooterConstants.kShooterHoodAngle1;
    }
    //If Shooter Knob is at 3
    else if(knobValue >= 0.024 + threshold && knobValue < 0.055 + threshold){
      angle = ShooterConstants.kShooterHoodAngle2;
    }
    //If Shooter Knob is at 4
    else if(knobValue >= 0.055 + threshold){
      angle = ShooterConstants.kShooterHoodAngle3;
    }
    else
    {
      angle = 0;
    }
  
    return angle;
  }

  public double getDesiredShooterSpeed(){
    double speed;
    double knobValue = m_copilotDS.getRawAxis(1);
    double threshold = 0.010;
    
    //If Shooter Knob is at 1
    if(knobValue < 0.024 - threshold){
        speed = 0;
    }
    //If Shooter Knob is at 2
    else if(knobValue >= 0.024 - threshold && knobValue < 0.024 + threshold){
        speed = ShooterConstants.kShooterSpeed1;
    }
    //If Shooter Knob is at 3
    else if(knobValue >= 0.024 + threshold && knobValue < 0.055 + threshold){
        speed = ShooterConstants.kShooterSpeed2;
    }
    //If Shooter Knob is at 4
    else if(knobValue >= 0.055 + threshold){
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
    //todo pick auto mode from dashboard
    return m_chooser.getSelected();
  }
}
