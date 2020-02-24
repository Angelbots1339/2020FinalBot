/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  /**
   * Creates a new Climber.
   */

  private CANSparkMax m_rightClimber;
  private CANSparkMax m_leftClimber;
  private double m_speed;
  private CANEncoder m_rightEncoder;
  private CANEncoder m_leftEncoder;

  /** TODO
   * Need feedback to Driver Station in case of improper setup
   * For Example, if climber is ran for xxx seconds but encoder does not register then kill the climb?
   * Mechanically, this setup would mean the motors are back-driving into the wrench ratchet
   */
  public ClimberSubsystem() {
    //m_rightEncoder = new CANEncoder(m_rightClimber);
    //m_leftEncoder = new CANEncoder(m_leftClimber);
    m_rightClimber = new CANSparkMax(ClimberConstants.kRightClimberMotor, MotorType.kBrushless);
    m_leftClimber = new CANSparkMax(ClimberConstants.kLeftClimberMotor, MotorType.kBrushless);

    m_leftClimber.setInverted(true);
    m_rightClimber.setInverted(false);

    m_leftClimber.setIdleMode(IdleMode.kBrake);
    m_rightClimber.setIdleMode(IdleMode.kBrake);
  }

  /*
  TODO - Needs to be verified for direction, enable() uses a negative value
  public void enable(double speed){
    m_rightClimber.set(speed);
    m_leftClimber.set(speed);
    m_speed = speed;
  }
  */

  public void enable(){
    m_rightClimber.set(-1 * ClimberConstants.kClimberSpeed);
    m_leftClimber.set(-1 * ClimberConstants.kClimberSpeed);
  }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left Climb Amp", m_leftClimber.getOutputCurrent());
    SmartDashboard.putNumber("Right Climb Amp", m_rightClimber.getOutputCurrent());
    SmartDashboard.putNumber("Climber Speed", m_speed);
    //SmartDashboard.putNumber("Left Climb Encoder", m_leftEncoder.getPosition());
    //SmartDashboard.putNumber("Right Climb Encoder", m_rightEncoder.getPosition());
    // This method will be called once per scheduler run
  }

public void disable() {
	m_rightClimber.set(0);
  m_leftClimber.set(0);
}
}
