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

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants.HoodedShooterConstants;

public class HoodPIDSubsystem extends PIDSubsystem {
  private final CANSparkMax m_hood;
  private final CANEncoder m_Encoder;

  private final SimpleMotorFeedforward m_feedforward =
      new SimpleMotorFeedforward(HoodedShooterConstants.KSVolts,
                                 HoodedShooterConstants.KVVoltSecondsPerRotation);
  /**
   * Creates a new HoodPID.
   */
  public HoodPIDSubsystem(double setpoint) {
    super(new PIDController(HoodedShooterConstants.kP, HoodedShooterConstants.kI, HoodedShooterConstants.kD));
    m_hood = new CANSparkMax(HoodedShooterConstants.kHoodPort, MotorType.kBrushless);
    m_Encoder = new CANEncoder(m_hood);
    m_Encoder.setPosition(0);

    
    getController().setTolerance(HoodedShooterConstants.positionTolerance);

    // Regardless of what's passed in, clamp to the min and max
    MathUtil.clamp(setpoint,HoodedShooterConstants.kminEncoderValue, HoodedShooterConstants.kmaxEncoderValue);
    setSetpoint(setpoint);
  }

  @Override
  public void useOutput(double output, double setpoint) {
    output += m_feedforward.calculate(setpoint);
    output = MathUtil.clamp(output, -HoodedShooterConstants.kMaxHoodVolt, HoodedShooterConstants.kMaxHoodVolt);
    m_hood.setVoltage(output);
    SmartDashboard.putNumber("HoodOutput", output);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return m_Encoder.getPosition();
  }

  public boolean atSetpoint() {
    return m_controller.atSetpoint();
  }

  public void setSetpoint(double setpoint) {
    // Regardless of what's passed in, clamp to the min and max
    setpoint = MathUtil.clamp(setpoint, HoodedShooterConstants.kminEncoderValue,HoodedShooterConstants.kmaxEncoderValue);
    super.setSetpoint(setpoint);
  }

  public void periodic() {
    super.periodic();
    SmartDashboard.putNumber("HoodEncoder", getMeasurement());
    SmartDashboard.putNumber("HoodSet", getController().getSetpoint());
    SmartDashboard.putNumber("Hood Current", m_hood.getOutputCurrent());
    SmartDashboard.putNumber("Hood Current", m_hood.getMotorTemperature());
  }
}
