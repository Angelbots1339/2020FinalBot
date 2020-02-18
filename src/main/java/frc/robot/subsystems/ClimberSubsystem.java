/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  /**
   * Creates a new ClimberSubsystem.
   */

  private PWMSparkMax m_leftClimberMotor;
  private PWMSparkMax m_rightClimberMotor;

  public ClimberSubsystem() {
    m_leftClimberMotor = new PWMSparkMax(ClimberConstants.kLeftClimberMotor);
    m_rightClimberMotor = new PWMSparkMax(ClimberConstants.kRightClimberMotor);
  }

  public void enable(){
    m_leftClimberMotor.set(ClimberConstants.kClimberSpeed);
    m_rightClimberMotor.set(ClimberConstants.kClimberSpeed);
  }

  public void disable(){
    m_leftClimberMotor.set(0);
    m_rightClimberMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
