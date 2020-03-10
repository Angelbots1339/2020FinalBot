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
import frc.robot.Constants.LoaderConstants;

public class LoaderPIDSubsystem extends PIDSubsystem {

  private CANSparkMax m_loader;
  private final CANEncoder m_encoder;

  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(LoaderConstants.KSVolts,
      LoaderConstants.KVVoltSecondsPerRotation);

  /**
   * Creates a new LoaderSubsystem.
   */
  public LoaderPIDSubsystem() {
    super(new PIDController(LoaderConstants.kP, LoaderConstants.kI, LoaderConstants.kD));

    m_loader = new CANSparkMax(LoaderConstants.kLoaderMotor, MotorType.kBrushless);
    m_loader.setInverted(true);

    m_encoder = new CANEncoder(m_loader);
    m_encoder.setPosition(0);
    m_encoder.setVelocityConversionFactor(0.2083);

    getController().setTolerance(LoaderConstants.kLoaderToleranceRPM);
    setSetpoint(LoaderConstants.kLoaderSetpoint);

  }

  public void reverse(double speed) {
    m_loader.set(-speed);
  }

  public void runSpeed(double speed) {
    m_loader.set(speed);
  }

  @Override
  protected void useOutput(double output, double setpoint) {
    output += m_feedforward.calculate(setpoint);
    // output = MathUtil.clamp(output, -6, 6);
    m_loader.setVoltage(output);
  }

  @Override
  protected double getMeasurement() {
    return m_encoder.getVelocity();
  }

  public boolean atSetpoint() {
    return m_controller.atSetpoint();
  }

  public void setSetpoint(double setpoint) {
    super.setSetpoint(setpoint);
  }

  public void periodic() {
    super.periodic();

    if (DashboardConstants.kLoaderTelemetry) {

      // This method will be called once per scheduler run
      SmartDashboard.putNumber("Loader Out", m_loader.get());
      SmartDashboard.putNumber("Loader RPM", m_encoder.getVelocity());
    }
  }

  public double get() {
    return m_loader.get();
  }

}
