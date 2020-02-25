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
    m_hoodMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_hoodEncoder = new CANEncoder(m_hoodMotor);
    m_hoodMotor.setInverted(false);
  }

  public double getEncoderPos(){
    return m_hoodEncoder.getPosition();
  }

  public void resetEncoder(){
    m_hoodEncoder.setPosition(0);
  }

  public double getMotorCurrent() {
    return m_hoodMotor.getOutputCurrent();
  }

  public void setEncoderZeroPos(double subtractable){
    m_hoodEncoder.setPosition(0 - subtractable);
  }

  public void setMotorVelo(double velocity) {
    // preventing the velocity from exceeding a limit
    // if greater or less than limit, set to respective limit
    if(velocity > HoodedShooterConstants.maxVeloValue) {
      velocity = HoodedShooterConstants.maxVeloValue;
    } else if (velocity < (-1*HoodedShooterConstants.maxVeloValue)) {
      velocity = -1*HoodedShooterConstants.maxVeloValue;
    }
    SmartDashboard.putNumber("Hood input", velocity);

    //check stalling. If so, set velocity to 0 
    if(getMotorCurrent() > HoodedShooterConstants.maxNormalCurrent) {
      velocity = 0;
    }
  

    // Preventing the hood from moving past max and min points
    // Positive Velocity moves towards...... max?
    // Negative Velocity moves towards min
    if(getEncoderPos() >= HoodedShooterConstants.minEncoderValue  && velocity < 0) {
      m_hoodMotor.set(velocity);
    } else if (HoodedShooterConstants.maxEncoderValue >= getEncoderPos() && velocity > 0) {
      m_hoodMotor.set(velocity);
    } else {
      SmartDashboard.putNumber("Hood output", 0);
      m_hoodMotor.set(0);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
   SmartDashboard.putNumber("Hood Motor current:", m_hoodMotor.getOutputCurrent());
   SmartDashboard.putNumber("Hood Encoder: ", m_hoodEncoder.getPosition());
  }
}
