/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CellManiputalionConstants;

public class CellManipulation extends SubsystemBase {
  private final VictorSP m_intakeMotor = new VictorSP(CellManiputalionConstants.kIntakeMotorPort);
  private final VictorSP m_queueMotor = new VictorSP(CellManiputalionConstants.kQueueMotorPort);
  private final VictorSP m_conveyorMotor = new VictorSP(CellManiputalionConstants.kConveyorMotorPort);

  private final DigitalInput m_topSensor = new DigitalInput(CellManiputalionConstants.kTopSensorPort);
  private final DigitalInput m_middleTopSensor = new DigitalInput(CellManiputalionConstants.kMiddleTopSensorPort);
  private final DigitalInput m_middleSensor = new DigitalInput(CellManiputalionConstants.kMiddleSensorPort);
  private final DigitalInput m_middleBottomSensor = new DigitalInput(CellManiputalionConstants.kMiddleBottomSensorPort);
  private final DigitalInput m_bottomSensor = new DigitalInput(CellManiputalionConstants.kBottomSensorPort);

  public CellManipulation() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateStatus();
  }

  public void setIntake(double speed){
    m_intakeMotor.set(speed);
  }

  public void setQueue(double speed){
    m_queueMotor.set(speed);
  }

  public void setConveyor(double speed){
    m_conveyorMotor.set(speed);
  }

  public int getHighestSensorActive(){
    if(m_topSensor.get()){
      return 5;
    }
    else if(m_middleTopSensor.get()){
      return 4;
    }
    else if(m_middleSensor.get()){
      return 3;
    }
    else if(m_middleBottomSensor.get()){
      return 2;
    }
    else if(m_bottomSensor.get()){
      return 1;
    }
    else{
      return 0;
    }
  }

  public boolean getBottomSensor(){
    return m_bottomSensor.get();
  }

  public void updateStatus(){
    SmartDashboard.putNumber("[CM] Highest Sensor Active", getHighestSensorActive());
    SmartDashboard.putBoolean("[CM] Bottom Sensor Active", getBottomSensor());
  }
}