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
import frc.robot.Constants.DashboardConstants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSide extends PIDSubsystem {

  private final CANSparkMax m_motor;
  private final CANEncoder m_encoder;
  private final String m_name;
  private final SimpleMotorFeedforward m_shooterFeedforward = new SimpleMotorFeedforward(ShooterConstants.KSVolts,
      ShooterConstants.KVVoltSecondsPerRotation);

  /**
   * The shooter subsystem for the robot.
   */
  public ShooterSide(int motorID, String name, boolean inverted) {
    super(new PIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD));
    m_motor = new CANSparkMax(motorID, MotorType.kBrushless);
    m_encoder = new CANEncoder(m_motor);
    m_name = name;
    // Spark 1 - Inverted - Not
    // Spark 3 - Inverted - Yes
    m_motor.setInverted(inverted);

    setSetpoint(ShooterConstants.kShooterTargetRPM);
  }

  @Override
  public void useOutput(double output, double setpoint) {
    m_motor.setVoltage(output + m_shooterFeedforward.calculate(setpoint));
  }

  @Override
  public double getMeasurement() {
    return m_encoder.getVelocity();
  }

  @Override
  public void setSetpoint(double setpoint) {
    if(getController() != null) getController().setTolerance((setpoint-ShooterConstants.kBaseRPM)*ShooterConstants.kShooterToleranceSlope);
    super.setSetpoint(setpoint);
  }

  public boolean atSetpoint() {
    return m_controller.atSetpoint();
  }

  public void stopShooter() {
    m_motor.set(0);
  }

  public void setMotorVoltage(double volts) {
    m_motor.setVoltage(volts);
  }

  public void periodic() {
    super.periodic();
    if (DashboardConstants.kShooterPIDTelemetry) {
      SmartDashboard.putNumber(m_name + " RPM", getMeasurement());
      SmartDashboard.putNumber(m_name + " Set", m_controller.getSetpoint());
      SmartDashboard.putBoolean(m_name + " OnT", atSetpoint());
    }
  }

}