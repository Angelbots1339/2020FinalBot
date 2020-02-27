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

public class IntakeArmPID2 extends PIDSubsystem {
  private final CANSparkMax m_rightMotor;
  private final CANSparkMax m_leftMotor;
  private final CANEncoder m_rightEncoder;

  private double setpoint = -5;

  /**
   * Creates a new Intake Arm PID.
   */
  public IntakeArmPID2() { 
    super(new PIDController(IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD));
    m_rightMotor = new CANSparkMax(IntakeConstants.kRightIntakeMoverMotor, MotorType.kBrushless);
    m_leftMotor = new CANSparkMax(IntakeConstants.kLeftIntakeMoverMotor, MotorType.kBrushless);
  
    m_rightEncoder = new CANEncoder(m_rightMotor);
    m_rightEncoder.setPosition(0);

    // Spark 8 - Right - Inverted 
    // Spark 9 - Left -  Not Inverted
    m_rightMotor.setInverted(true);
    //m_leftMotor.setInverted(false);
    //m_leftMotor.follow(m_rightMotor);
    m_leftMotor.follow(m_rightMotor, true);

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
    m_rightMotor.setVoltage(output);
    SmartDashboard.putNumber("R Intake Arm Output", output);
    SmartDashboard.putNumber("L Intake Arm Output", m_leftMotor.get());
    System.out.println("Im Here in output");
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
    setpoint = MathUtil.clamp(setpoint, IntakeConstants.kminEncoderValue, IntakeConstants.kmaxEncoderValue);
    super.setSetpoint(setpoint);
    this.setpoint = setpoint;
  }

  public void periodic() {
    super.periodic();
    SmartDashboard.putNumber("R Intake Arm Encoder", getMeasurement());
    SmartDashboard.putNumber("R Intake Arm Set Point", getController().getSetpoint());
  }

  
}
