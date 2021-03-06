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
import frc.robot.Constants.DashboardConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.commands.vision.ShootingProfile;

public class LimelightSubsystem extends SubsystemBase {

  private NetworkTable mNetworkTable;
  private LedMode mLedMode = LedMode.PIPELINE;
  private boolean m_isAligned = true;
  private boolean m_isAligning = false;
  private ShootingProfile m_latestTargetProfile = new ShootingProfile(0, 0, 0, 0, 0, 0);
  private boolean hasSeenTarget = false;
  private double m_defaultDistance = LimelightConstants.kDefaultDistance;

  /**
   * Creates a new Limelight.
   */
  public LimelightSubsystem() {
    mNetworkTable = NetworkTableInstance.getDefault().getTable(LimelightConstants.kLimeTable);
    setLed(mLedMode);
    setPipeline(LimelightConstants.kDefaultPipeline);
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
    return (int) mNetworkTable.getEntry("tv").getDouble(0) == 1;
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
    if (!hasSeenTarget) {
      return m_defaultDistance;
    }
    return LimelightConstants.kLimelightToTargetHeight / Math.tan(Math.toRadians(
        getYTargetOffset() + LimelightConstants.kLimeLightTilt + LimelightConstants.kPanningOffest[getPipeline()]));
  }

  public void setPipeline(int index) {
    mNetworkTable.getEntry("pipeline").setDouble(index);
  }

  public int getPipeline() {
    return (int) mNetworkTable.getEntry("pipeline").getDouble(0.0);
  }

  @Override
  @SuppressWarnings("unused")
  public void periodic() {
    updatePipeline();

    if (seesTarget()) {
      hasSeenTarget = true;
    }

    if (LimelightConstants.kAutoLight) {
      setLed(isAligning() ? LedMode.PIPELINE : LedMode.OFF);
    }

    if (DashboardConstants.kLimelightTelemetry) {
      SmartDashboard.putNumber(LimelightConstants.kLimeTable + ": Pipeline Latency(ms)", getLatency());
      SmartDashboard.putNumber(LimelightConstants.kLimeTable + ": X ", getXTargetOffset());
      SmartDashboard.putNumber(LimelightConstants.kLimeTable + ": Y", getYTargetOffset());
    }
    SmartDashboard.putString("profile", m_latestTargetProfile.toString());
    SmartDashboard.putBoolean(LimelightConstants.kLimeTable + ": Has Target", seesTarget());
    SmartDashboard.putNumber(LimelightConstants.kLimeTable + ": Dist", getDistanceToVisionTarget());
  }

  private void updatePipeline() {
    if (LimelightConstants.kAutoColorVision) {
      setPipeline(LimelightConstants.kColorPipeline);
    } else {
      if (!isAligning()) {
        if (LimelightConstants.kAutoZoom) {
          autoZoom();
        } else {
          setPipeline(0);
        }
      }
    }
  }

  private void autoZoom() {
    if (!seesTarget()) {
      setPipeline(0);
    } else {
      int pipeline = 0;
      if (getDistanceToVisionTarget() > LimelightConstants.k2XZoomCutoff) {
        pipeline++;
      }
      if (getDistanceToVisionTarget() > LimelightConstants.k3XZoomCutoff) {
        pipeline++;
      }
      setPipeline(pipeline);
    }
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

  public void setActiveProfile(ShootingProfile latestTargetProfile) {
    m_latestTargetProfile = latestTargetProfile;
  }

  public ShootingProfile getLatestProfile() {
    return m_latestTargetProfile;
  }

  public void reset(double defaultDistance) {
    mLedMode = LedMode.PIPELINE;
    m_isAligned = true;
    m_isAligning = false;
    hasSeenTarget = false;
    m_defaultDistance = defaultDistance;
  }

  public void reset() {
    reset(LimelightConstants.kDefaultDistance);
  }
}
