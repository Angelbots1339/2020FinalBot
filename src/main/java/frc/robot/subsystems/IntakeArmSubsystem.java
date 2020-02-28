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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeArmSubsystem extends SubsystemBase {
  private CANSparkMax m_motor;
  private CANEncoder m_Encoder;
  private String m_name;
  /**
   * Creates a new AdjustableHoodSubsystem.
   */
  public IntakeArmSubsystem(int motorID, boolean inverted, String name) {
    m_motor = new CANSparkMax(motorID, MotorType.kBrushless);
    m_motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_Encoder = new CANEncoder(m_motor);
    m_motor.setInverted(inverted);
    m_name = name;
  }

  public double getEncoderPos(){
    return m_Encoder.getPosition();
  }

  public void resetEncoder(){
    m_Encoder.setPosition(0);
  }

  public double getMotorCurrent() {
    return m_motor.getOutputCurrent();
  }

  public void setEncoderZeroPos(double subtractable){
    m_Encoder.setPosition(0 - subtractable);
  }

  public void setMotorVelo(double velocity) {
    // preventing the velocity from exceeding a limit
    // if greater or less than limit, set to respective limit
    if(velocity > IntakeConstants.kmaxVeloValue) {
      velocity = IntakeConstants.kmaxVeloValue;
    } else if (velocity < (-1*IntakeConstants.kmaxVeloValue)) {
      velocity = -1*IntakeConstants.kmaxVeloValue;
    }
    SmartDashboard.putNumber(m_name + "input", velocity);

    //check stalling. If so, set velocity to 0 
    if(getMotorCurrent() > IntakeConstants.kmaxNormalCurrent) {
      velocity = 0;
    }
  

    // Preventing the hood from moving past max and min points
    // Positive Velocity moves towards...... max?
    // Negative Velocity moves towards min
    if(getEncoderPos() >= IntakeConstants.kminEncoderValue  && velocity < 0) {
      m_motor.set(velocity);
    } else if (IntakeConstants.kmaxEncoderValue >= getEncoderPos() && velocity > 0) {
      m_motor.set(velocity);
    } else {
      SmartDashboard.putNumber(m_name + "output", 0);
      m_motor.set(0);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  //  SmartDashboard.putNumber(m_name + "motor current:", m_motor.getOutputCurrent());
  //  SmartDashboard.putNumber(m_name + "Encoder: ", m_Encoder.getPosition());
  }
}
