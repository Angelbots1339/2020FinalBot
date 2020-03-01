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
import frc.robot.Constants.DashboardConstants;
import frc.robot.Constants.IntakeConstants;

/**
 * currently testing this
 */

public class IntakeArmPID extends PIDSubsystem {
  private final CANSparkMax m_leftMotor;
  private final CANSparkMax m_rightMotor;
  private final CANEncoder m_rightEncoder;

  private double m_setpoint = 0;

  /**
   * Creates a new Intake Arm PID.
   */
  public IntakeArmPID() {
    super(new PIDController(IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD));
    m_leftMotor = new CANSparkMax(IntakeConstants.kLeftIntakeMoverMotor, MotorType.kBrushless);
    m_rightMotor = new CANSparkMax(IntakeConstants.kRightIntakeMoverMotor, MotorType.kBrushless);
    m_rightEncoder = new CANEncoder(m_rightMotor);
    m_rightEncoder.setPosition(0);
    // Spark 8 - Right - Inverted
    // Spark 9 - Left - Not Inverted
    m_leftMotor.setInverted(false);
    m_rightMotor.setInverted(true);
    m_leftMotor.follow(m_rightMotor, true);

    getController().setTolerance(IntakeConstants.kPositionTolerance);
    setSetpoint(m_setpoint);
  }

  public double getSetpoint() {
    return m_setpoint;
  }

  public void runIntakeArms() {

    /*
     * if(m_rightEncoder.getPosition() <= -1){
     * setSetpoint(IntakeConstants.kMaxEncoderValue); enable(); }else{
     * setSetpoint(IntakeConstants.kMinEncoderValue); enable(); }
     */
  }

  @Override
  public void useOutput(double output, double setpoint) {
    output = MathUtil.clamp(output, -IntakeConstants.kIntakeArmMotorVolt, IntakeConstants.kIntakeArmMotorVolt);
    m_rightMotor.setVoltage(output);
    SmartDashboard.putNumber("Right Intake Arm Output", output);
    SmartDashboard.putNumber("Left Intake Arm Output", m_leftMotor.get());
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return m_rightEncoder.getPosition();
  }

  public boolean atSetpoint() {
    return m_controller.atSetpoint();
  }

  public void setSetpoint(double setpoint) {
    // Regardless of what's passed in, clamp to the min and max
    setpoint = MathUtil.clamp(setpoint, IntakeConstants.kMinEncoderValue, IntakeConstants.kMaxEncoderValue);
    super.setSetpoint(setpoint);
    m_setpoint = setpoint;
  }

  public void periodic() {
    super.periodic();
    if (DashboardConstants.kIntakeArmTelemetry) {
      SmartDashboard.putNumber("Right Intake Arm Encoder", getMeasurement());
      SmartDashboard.putNumber("Left Intake Arm Current", m_leftMotor.getOutputCurrent());
      SmartDashboard.putNumber("Right Intake Arm Set Point", getController().getSetpoint());
    }
  }

  public void toggleSetpoint() {
    if (getSetpoint() < (IntakeConstants.kMinEncoderValue + IntakeConstants.kMaxEncoderValue) / 2) {
      setSetpoint(IntakeConstants.kMaxEncoderValue);
    } else {
      setSetpoint(IntakeConstants.kMinEncoderValue);
    }
  }

}