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
import frc.robot.Constants.LimelightConstants;
import frc.robot.commands.vision.ShootingProfiles;;

public class LimelightSubsystem extends SubsystemBase {

  private NetworkTable mNetworkTable;
  private LedMode mLedMode = LedMode.PIPELINE; 
  private boolean m_isAligned = true;
  private boolean m_isAligning = false;
  private ShootingProfiles m_latestTargetProfile = new ShootingProfiles(3, 0, 0, 0, 0);

  /**
   * Creates a new Limelight.
   */
  public LimelightSubsystem() {
    mNetworkTable = NetworkTableInstance.getDefault().getTable(LimelightConstants.kLimeTable);
    setLed(mLedMode);
  }

  public enum LedMode {
    PIPELINE, OFF, BLINK, ON
  }

  public void setLed(LedMode mode) {
    mLedMode = mode;
    mNetworkTable.getEntry("ledMode").setNumber(mode.ordinal());
  }

  public double getLatency() {
    return mNetworkTable.getEntry("tl").getDouble(0);
  }

  public boolean seesTarget() {
    return mNetworkTable.getEntry("tv").getDouble(0) == 1.0;
  }

  public double getXTargetOffset() {
    return mNetworkTable.getEntry("tx").getDouble(0.0);
  }

  public double getYTargetOffset() {
    return mNetworkTable.getEntry("ty").getDouble(0.0);
  }

  public double getArea() {
    return mNetworkTable.getEntry("ta").getDouble(0.0);
  }

  public double getDistanceToVisionTarget() {
    return LimelightConstants.kLimelightToTargetHeight
        / Math.tan(Math.toRadians(getYTargetOffset() + LimelightConstants.kLimeLightTilt + LimelightConstants.kPanningOffest[getPipeline()]));
  }

  public void setPipeline(int index) {
    mNetworkTable.getEntry("pipeline").setDouble(index);
  }

  public int getPipeline() {
    return (int) mNetworkTable.getEntry("pipeline").getDouble(0.0);
  }

  @Override
  public void periodic() {
    if (LimelightConstants.kAutoZoom)
      setPipeline(getDistanceToVisionTarget() > LimelightConstants.k2XZoomCutoff
          ? getDistanceToVisionTarget() > LimelightConstants.k3XZoomCutoff ? 2 : 1
          : 0);
    if (!seesTarget())
      setPipeline(0);
    SmartDashboard.putString("profile", m_latestTargetProfile.toString());
    // SmartDashboard.putBoolean(LimelightConstants.kLimeTable + ": Has Target",
    // seesTarget());
    // SmartDashboard.putNumber(LimelightConstants.kLimeTable + ": Pipeline Latency
    // (ms)", getLatency());
    // SmartDashboard.putNumber(LimelightConstants.kLimeTable + ": X ",
    // getXTargetOffset());
    // SmartDashboard.putNumber(LimelightConstants.kLimeTable + ": Y",
    // getYTargetOffset());
    SmartDashboard.putNumber(LimelightConstants.kLimeTable + ": Dist",
    getDistanceToVisionTarget());
  }

  public void setAligned(boolean isAligned) {
    m_isAligned = isAligned;
  }

  public boolean isAligned() {
    return m_isAligned;
  }

  public void setAligning(boolean isAligning) {
    m_isAligning = isAligning;
  }

  public boolean isAligning() {
    return m_isAligning;
  }

  public void setActiveProfile(ShootingProfiles latestTargetProfile) {
    m_latestTargetProfile = latestTargetProfile;
  }

  public ShootingProfiles getLatestProfile() {
    return m_latestTargetProfile;
  }
}
