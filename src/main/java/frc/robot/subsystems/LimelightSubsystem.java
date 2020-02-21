/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; 
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LimelightSubsystem extends SubsystemBase {

  private boolean mSeesTarget = false;
  private NetworkTable mNetworkTable;
  private LedMode mLedMode = LedMode.PIPELINE;  // 0 - use pipeline mode, 1 - off, 2 - blink, 3 - on
  private double mXOffset;
  private double mYOffset;
  private double mArea;
  private double mLatency;

  /**
   * Creates a new Limelight.
   */
  public double getDistanceToVisionTarget() {
    return Constants.LimelightConstants.kLimelightCameraToVisionRetroreflectiveTargetHeight / Math.tan(Math.toRadians(getYTargetOffset()));
  }
  public LimelightSubsystem() {
    mNetworkTable = NetworkTableInstance.getDefault().getTable(Constants.LimelightConstants.kLimeTable);
  }

  public enum LedMode {
    PIPELINE, OFF, BLINK, ON
  }

  public void setLed(LedMode mode) {
    mLedMode = mode;
    mNetworkTable.getEntry("ledMode").setNumber(mode.ordinal());
  }

  public boolean seesTarget() {
      return mSeesTarget;
  }
  public double getXTargetOffset(){
    return mXOffset;
  }
  public double getYTargetOffset(){
    return mYOffset;
  }
  public double getArea(){
    return mArea;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    mLatency = mNetworkTable.getEntry("tl").getDouble(0);
    mXOffset = mNetworkTable.getEntry("tx").getDouble(0.0);
    mYOffset = mNetworkTable.getEntry("ty").getDouble(0.0);
    mArea = mNetworkTable.getEntry("ta").getDouble(0.0);
    mSeesTarget = mNetworkTable.getEntry("tv").getDouble(0) == 1.0;
    

    SmartDashboard.putBoolean(Constants.LimelightConstants.kLimeTable + ": Has Target", mSeesTarget);
    SmartDashboard.putNumber(Constants.LimelightConstants.kLimeTable + ": Pipeline Latency (ms)", mLatency);
    SmartDashboard.putNumber(Constants.LimelightConstants.kLimeTable + ": X ", mXOffset);
    SmartDashboard.putNumber(Constants.LimelightConstants.kLimeTable + ": Y", mYOffset);
     
  }
}
