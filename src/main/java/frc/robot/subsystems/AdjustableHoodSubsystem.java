/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodedShooterConstants;

public class AdjustableHoodSubsystem extends SubsystemBase {
  private CANSparkMax m_hoodMotor;
  private CANEncoder m_hoodEncoder;
  /**
   * Creates a new AdjustableHoodSubsystem.
   */
  public AdjustableHoodSubsystem() {
    m_hoodMotor = new CANSparkMax(HoodedShooterConstants.kHoodPort, MotorType.kBrushless);
    m_hoodEncoder = new CANEncoder(m_hoodMotor);
  }

  public double getEncoderPos(){
    return m_hoodEncoder.getPosition();
  }

  public void setEncoderZeroPos(){
    m_hoodEncoder.setPosition(0);
  }

  public void setMotorVelo(double velocity) {
    SmartDashboard.putNumber("Hood input", velocity);
    m_hoodMotor.set(velocity);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Hood Encoder: ", m_hoodEncoder.getPosition());
  }
}
