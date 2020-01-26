/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  private final CANSparkMax m_leftFrontMotor = new CANSparkMax (DriveConstants.kLeftFrontMotorPort, MotorType.kBrushless);
  private final CANSparkMax m_leftBackMotor = new CANSparkMax (DriveConstants.kLeftBackMotorPort, MotorType.kBrushless);
  private final CANSparkMax m_rightFrontMotor = new CANSparkMax (DriveConstants.kRightFrontMotorPort, MotorType.kBrushless);
  private final CANSparkMax m_rightBackMotor = new CANSparkMax (DriveConstants.kRightBackMotorPort, MotorType.kBrushless);

  private AHRS m_gyro;

  // The motors on the left side of the drive.
  private final SpeedControllerGroup m_leftMotors = new SpeedControllerGroup(m_leftFrontMotor,m_leftBackMotor);

  // The motors on the right side of the drive.
  private final SpeedControllerGroup m_rightMotors = new SpeedControllerGroup(m_rightFrontMotor,m_rightBackMotor);

  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  public DriveSubsystem() {
    configureSpark(m_leftFrontMotor);
    configureSpark(m_leftBackMotor);
    configureSpark(m_rightFrontMotor);
    configureSpark(m_rightBackMotor);

    try {
        m_gyro = new AHRS(SPI.Port.kMXP);
      } catch (RuntimeException ex ) {
        DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
      }
  }
  
  private void configureSpark(CANSparkMax sparkMax) {
    sparkMax.restoreFactoryDefaults();
    //sparkMax.enableVoltageCompensation(12.0s);
    sparkMax.setOpenLoopRampRate(0.08);
    sparkMax.setSmartCurrentLimit(DriveConstants.kCurrentLimit);
    sparkMax.setIdleMode(CANSparkMax.IdleMode.kBrake);
  }

  @Override
  public void periodic() {
    updateStatus();
  }

  public void tankDrive(double left, double right){
    m_drive.tankDrive(left, right);
  }

  public void arcadeDrive(double forward, double rotate){
    m_drive.arcadeDrive(forward, rotate);
  }

  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  public double getHeading(){
    try{
      return m_gyro.getAngle();
    } catch (RuntimeException ex ) {
          return 0;
    }
  }

  public void resetHeading(){
    try{
      m_gyro.reset();
    } catch (RuntimeException ex ) {
    }
  }

  public void updateStatus(){
    SmartDashboard.putNumber("[DT] Heading", getHeading());
  }
}
