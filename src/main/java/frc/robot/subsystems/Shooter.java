/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  private final CANSparkMax m_masterMotor = new CANSparkMax(ShooterConstants.kMasterMotorPort, MotorType.kBrushless);
  private final CANSparkMax m_slaveMotor = new CANSparkMax(ShooterConstants.kSlaveMotorPort, MotorType.kBrushless);
  private final CANPIDController m_pidController;
  private final CANEncoder m_encoder;
  private double m_kP, m_kI, m_kD, m_kFF, m_setpoint;

  private final VictorSPX m_feederMotor =  new VictorSPX(ShooterConstants.kFeederMotorPort);

  public Shooter() {
    configureSpark(m_masterMotor);
    configureSpark(m_slaveMotor);
    m_slaveMotor.follow(m_masterMotor);

    m_pidController = m_masterMotor.getPIDController();
    m_encoder = m_masterMotor.getEncoder();

    // set PID coefficients
    m_pidController.setP(ShooterConstants.kP);
    m_pidController.setI(ShooterConstants.kI);
    m_pidController.setD(ShooterConstants.kD);
    m_pidController.setFF(ShooterConstants.kFF);
    m_pidController.setOutputRange(ShooterConstants.kMinOutput, ShooterConstants.kMaxOutput);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("[Shooter] P Gain", ShooterConstants.kP);
    SmartDashboard.putNumber("[Shooter] I Gain", ShooterConstants.kI);
    SmartDashboard.putNumber("[Shooter] D Gain", ShooterConstants.kD);
    SmartDashboard.putNumber("[Shooter] I Zone", ShooterConstants.kI);
    SmartDashboard.putNumber("[Shooter] Feed Forward", ShooterConstants.kFF);
    SmartDashboard.putNumber("[Shooter] Setpoint", 0);
  }

  private void configureSpark(CANSparkMax sparkMax) {
    sparkMax.restoreFactoryDefaults();
    sparkMax.setSmartCurrentLimit(ShooterConstants.kCurrentLimit);
    sparkMax.setIdleMode(CANSparkMax.IdleMode.kCoast);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setFeeder(double speed){
    m_feederMotor.set(ControlMode.PercentOutput,speed);
  }

  public void setSetpoint(double setpoint){
    m_pidController.setReference(setpoint, ControlType.kVelocity);

    m_setpoint = setpoint;

    if(m_setpoint < 1){
      setFeeder(0.5);
    }
    else{
      setFeeder(0);
    }
  }

  public boolean upToSpeed() {
    double curVel = m_encoder.getVelocity();
    return Math.abs(curVel - m_setpoint) < ShooterConstants.kAllowedError;
  }

  public void tune () {
    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("[Shooter] P Gain", ShooterConstants.kP);
    double i = SmartDashboard.getNumber("[Shooter] I Gain", ShooterConstants.kI);
    double d = SmartDashboard.getNumber("[Shooter] D Gain", ShooterConstants.kD);
    double ff = SmartDashboard.getNumber("[Shooter] Feed Forward", ShooterConstants.kFF);
    double setpoint = SmartDashboard.getNumber("[Shooter] Setpoint", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != m_kP)) { m_pidController.setP(p); m_kP = p; }
    if((i != m_kI)) { m_pidController.setI(i); m_kI = i; }
    if((d != m_kD)) { m_pidController.setD(d); m_kD = d; }
    if((ff != m_kFF)) { m_pidController.setFF(ff); m_kFF = ff; }
    if((setpoint != m_setpoint)) { m_pidController.setReference(setpoint, ControlType.kVelocity); m_setpoint = setpoint; }

    SmartDashboard.putNumber("[Shooter] Velocity", m_encoder.getVelocity());

    if(m_setpoint < 1){
      setFeeder(0.5);
    }
    else{
      setFeeder(0);
    }
  }
}
