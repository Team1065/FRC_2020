/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  private final NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
  private final NetworkTableEntry tv = limelightTable.getEntry("tv");
  private final NetworkTableEntry tx = limelightTable.getEntry("tx");
  private final NetworkTableEntry ty = limelightTable.getEntry("ty");
  private final NetworkTableEntry ta = limelightTable.getEntry("ta");

  public Vision() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateStatus();
  }

  public boolean isTargetValid(){
    return tv.getBoolean(false);
  }

  public double getTargetHorizontalOffset(){
    return tx.getDouble(0.0);
  }

  public double getTargetVerticalOffset(){
    return ty.getDouble(0.0);
  }

  public double getTargetArea(){
    return ta.getDouble(0.0);
  }

  public void updateStatus(){
    SmartDashboard.putBoolean("[Vision] Valid Target", isTargetValid());
    SmartDashboard.putNumber("[Vision] Target Horizontal Offset", getTargetHorizontalOffset());
    SmartDashboard.putNumber("[Vision] Target Vertical Offset", getTargetVerticalOffset());
    SmartDashboard.putNumber("[Vision] Target Area", getTargetArea());
  }
}