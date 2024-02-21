// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class LimelightSubsystem extends SubsystemBase {
  public NetworkTable table;
  public NetworkTableEntry tx;
  public NetworkTableEntry ty;
  public NetworkTableEntry tv;
  public NetworkTableEntry ta;
  public NetworkTableEntry pipeline;
  public NetworkTableEntry getpipe;
  public NetworkTableEntry tclass;
  public NetworkTableEntry botpose_wpiblue;
  public NetworkTableEntry botpose_wpired;
  public NetworkTableEntry botpose;
  public NetworkTableEntry camMode;
  public NetworkTableEntry targetpose_cameraspace;

  /** Creates a new LimelightSubsystem. */
  public LimelightSubsystem() {
    table = NetworkTableInstance.getDefault().getTable("limelight-james");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
    tv = table.getEntry("tv");
    pipeline = table.getEntry("pipeline");
    getpipe = table.getEntry("getpipe");
    tclass = table.getEntry("tclass");
    camMode = table.getEntry("camMode");
    botpose = table.getEntry("botpose");
    botpose_wpiblue = table.getEntry("botpose_wpiblue");
    botpose_wpired = table.getEntry("botpose_wpired");
    targetpose_cameraspace = table.getEntry("targetpose_cameraspace");

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

  public double getBotPoseX() {
    double pose[] = botpose.getDoubleArray(new double[6]);
    return pose[0];
  }

  public double getBotPoseY() {
    double pose[] = botpose.getDoubleArray(new double[6]);
    return pose[1];
  }

  public double getBotPoseXTeamRelative() {
    double pose[] = RobotContainer.Color == "red" ? botpose_wpired.getDoubleArray(new double[6])
        : botpose_wpiblue.getDoubleArray(new double[6]);
    return pose[0];
  }

  public double getBotPoseYTeamRelative() {
    double pose[] = RobotContainer.Color == "red" ? botpose_wpired.getDoubleArray(new double[6])
        : botpose_wpiblue.getDoubleArray(new double[6]);
    return pose[1];
  }

  public double getTargetPos(int value){
    double pos[] = targetpose_cameraspace.getDoubleArray(new double[6]);
    return pos[value];
  }

  public double TargetDistance(){
    return Math.sqrt(Math.pow(getTargetPos(0), 2) + Math.pow(getTargetPos(1), 2));
  }

  public void setCamMode(int value) {
    camMode.setDouble(value);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("tx", getTargetX());
    SmartDashboard.putNumber("tclass", getClassifier());
    SmartDashboard.putNumber("BotPoseX", getBotPoseX());
  }
}
