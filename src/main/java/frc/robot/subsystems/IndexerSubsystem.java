/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DashboardConstants;
import frc.robot.Constants.IndexerConstants;

public class IndexerSubsystem extends SubsystemBase {

  private CANSparkMax m_leftIndexer;
  private CANSparkMax m_rightIndexer;

  /**
   * Creates a new IndexerSubsystem.
   */

  public IndexerSubsystem() {
    m_leftIndexer = new CANSparkMax(IndexerConstants.kLeftIndexerMotor, MotorType.kBrushless);
    m_rightIndexer = new CANSparkMax(IndexerConstants.kRightIndexMotor, MotorType.kBrushless);
    m_leftIndexer.setInverted(true);
    m_rightIndexer.setInverted(false);
  }

  public void enable() {
    enable(IndexerConstants.kMaxIndexSpeed);
  }

  public void enable(double speed) {
    m_leftIndexer.set(speed);
    m_rightIndexer.set(speed);
  }

  public void reverse() {
    m_leftIndexer.set(-IndexerConstants.kMaxIndexSpeed);
    m_rightIndexer.set(-IndexerConstants.kMaxIndexSpeed);
  }

  public void disable() {
    m_leftIndexer.set(0);
    m_rightIndexer.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (DashboardConstants.kIndexerTelemetry) {
      SmartDashboard.putNumber("right indexer", m_rightIndexer.get());
      SmartDashboard.putNumber("left indexer", m_leftIndexer.get());
      SmartDashboard.putNumber("Right Indexer Amps", m_rightIndexer.getOutputCurrent());
      SmartDashboard.putNumber("Left Indexer Amps", m_leftIndexer.getOutputCurrent());
    }

  }

  public boolean isCurrentSpike() {
    return (m_leftIndexer.getOutputCurrent() > IndexerConstants.kAcceptableCurrentSpike)
        || (m_rightIndexer.getOutputCurrent() > IndexerConstants.kAcceptableCurrentSpike);
  }

}
