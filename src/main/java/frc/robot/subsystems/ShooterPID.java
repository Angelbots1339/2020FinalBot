/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.HoodedShooterConstants;
import frc.robot.Constants.ShooterConstants;
//import edu.wpi.first.wpilibj.controller.PIDController;
//import edu.wpi.first.wpilibj2.command.PIDSubsystem;
//import frc.robot.Constants.ShooterConstants;

public class ShooterPID extends PIDSubsystem {
  
  private final CANSparkMax m_motor;
  private final CANEncoder m_Encoder;
  private CANSparkMax m_hoodMotor;
  private CANEncoder m_hoodEncoder;
  
  private String m_name;
  
  private final SimpleMotorFeedforward m_shooterFeedforward =
      new SimpleMotorFeedforward(ShooterConstants.KSVolts,
                                 ShooterConstants.KVVoltSecondsPerRotation);

  /**
   * The shooter subsystem for the robot.
   */
  public ShooterPID(int motorID, String name, boolean inverted) {
    super(new PIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD));
    m_motor = new CANSparkMax(motorID, MotorType.kBrushless);
    m_hoodMotor = new CANSparkMax(HoodedShooterConstants.kHoodPort, MotorType.kBrushless);
    m_Encoder = new CANEncoder(m_motor);
    m_hoodEncoder = new CANEncoder(m_hoodMotor);
    m_name = name;
    // Spark 1 - Inverted - Not 
    // Spark 3 - Inverted - Yes
    m_motor.setInverted(inverted);

    getController().setTolerance(ShooterConstants.kShooterToleranceRPS);
    setSetpoint(ShooterConstants.kShooterTargetRPS);
  }

  @Override
  public void useOutput(double output, double setpoint) {
    m_motor.setVoltage(output + m_shooterFeedforward.calculate(setpoint));
  }

  @Override
  public double getMeasurement() {
    return m_Encoder.getVelocity(); //m_shooterEncoder.getRate();
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
    SmartDashboard.putNumber(m_name + " RPM", getMeasurement());
    SmartDashboard.putNumber(m_name + " Set", m_controller.getSetpoint());
    SmartDashboard.putBoolean(m_name + " OnT", atSetpoint());
    
  }

}