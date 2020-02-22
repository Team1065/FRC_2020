/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  public final VictorSPX m_climberMotorMaster = new VictorSPX(ClimberConstants.kClimberMasterMotor);
  public final VictorSPX m_climberMotorSlave = new VictorSPX(ClimberConstants.kClimberSlaveMotor);
  public final DigitalInput m_bottomSensor = new DigitalInput(ClimberConstants.kBottomSensorPort);

  public double m_bottomSwitchFalseCount = 0;

  public Climber() {
    m_climberMotorSlave.follow(m_climberMotorMaster);
    m_climberMotorSlave.setInverted(true);
    m_climberMotorSlave.setNeutralMode(NeutralMode.Brake);
    
    m_climberMotorMaster.setNeutralMode(NeutralMode.Brake);
  }

  public boolean getBottomSensorActive(){
    return !m_bottomSensor.get();
  }

  public void setClimberSpeed(double speed){
    m_climberMotorMaster.set(ControlMode.PercentOutput,speed);
  }

  public boolean lookForBottomSwitch(){
    return m_bottomSwitchFalseCount > 100;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateStatus();

    if(DriverStation.getInstance().isEnabled()){
      if(!getBottomSensorActive()){
        m_bottomSwitchFalseCount++;
      }
    }
  }

  public void updateStatus(){
    SmartDashboard.putBoolean("[Climber] Bottom Limit", getBottomSensorActive());
    SmartDashboard.putBoolean("[Climber] Look For Bottom Limit", lookForBottomSwitch());
  }
}
