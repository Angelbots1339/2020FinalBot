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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants.IntakeConstants;
/**
 * currently testing this
 */

public class IntakeArmPID extends PIDSubsystem {
  private final CANSparkMax m_motor;
  private final CANEncoder m_Encoder;

  private final String m_name;
  private double setpoint = 0;

  /**
   * Creates a new Intake Arm PID.
   */
  public IntakeArmPID(int motorID, String name, boolean inverted) {
    super(new PIDController(IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD));
    m_motor = new CANSparkMax(motorID, MotorType.kBrushless);
    m_Encoder = new CANEncoder(m_motor);
    m_Encoder.setPosition(0);
    m_name = name;

    // Spark 8 - Right - Inverted 
    // Spark 9 - Left -  Not Inverted
    m_motor.setInverted(inverted);

    getController().setTolerance(IntakeConstants.positionTolerance);

    // Regardless of what's passed in, clamp to the min and max
    MathUtil.clamp(setpoint, IntakeConstants.kminEncoderValue, IntakeConstants.kmaxEncoderValue);
    setSetpoint(setpoint);
  }

  public double getSetpoint() {
    return setpoint;
  }

  @Override
  public void useOutput(double output, double setpoint) {
    output = MathUtil.clamp(output, -IntakeConstants.kIntakeArmMotorVolt, IntakeConstants.kIntakeArmMotorVolt);
    m_motor.setVoltage(output);
    SmartDashboard.putNumber(m_name + " Output", output);
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
    setpoint = MathUtil.clamp(setpoint, IntakeConstants.kminEncoderValue, IntakeConstants.kmaxEncoderValue);
    super.setSetpoint(setpoint);
    this.setpoint = setpoint;
  }

  public void periodic() {
    super.periodic();
    SmartDashboard.putNumber(m_name + " Encoder", getMeasurement());
    SmartDashboard.putNumber(m_name + " Set Point", getController().getSetpoint());
  }

  
}
