/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;
import edu.wpi.first.wpilibj.PWMSparkMax;

public class IndexerSubsystem extends SubsystemBase {

  private PWMSparkMax m_leftIndexer;
  private PWMSparkMax m_rightIndexer;

  /**
   * Creates a new IndexerSubsystem.
   */

  public IndexerSubsystem() {
    m_leftIndexer = new PWMSparkMax(IndexerConstants.kLeftIndexerMotor);
    m_rightIndexer = new PWMSparkMax(IndexerConstants.kRightIndexMotor);
  }

  public void enable(){
    m_leftIndexer.set(IndexerConstants.kLeftIndexerMotor);
    m_rightIndexer.set(IndexerConstants.kRightIndexMotor);
  }

  public void disable(){
    m_leftIndexer.set(0);
    m_rightIndexer.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
