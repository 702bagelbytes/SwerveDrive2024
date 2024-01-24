// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {
  public NetworkTable table;
  public NetworkTableEntry tx;
  public NetworkTableEntry ty;
  public NetworkTableEntry tv;
  public NetworkTableEntry ta;
  public NetworkTableEntry pipeline;
  public NetworkTableEntry getpipe;
  public NetworkTableEntry tclass;

  /** Creates a new LimelightSubsystem. */
  public LimelightSubsystem() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
    tv = table.getEntry("tv");
    pipeline = table.getEntry("pipeline");
    getpipe = table.getEntry("getpipe");
    tclass = table.getEntry("tclass");
  }

  public double getTargetX() {
    return tx.getNumber(0).doubleValue();
  }

  public double getTargetY() {
    return ty.getNumber(0).doubleValue();
  }

  public double getTargetA() {
    return ta.getNumber(0).doubleValue();
  }

  public boolean IsTargetAvailable() {
    return tv.getNumber(0).intValue() == 1 ? true : false;
  }

  public void setPipeline(int value) {
    pipeline.setNumber(value);
  }

  public int getPipeline() {
    return getpipe.getNumber(0).intValue();
  }

  public int getClassifier() {
    return tclass.getNumber(0).intValue();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("tx", getTargetX());
  }
}
