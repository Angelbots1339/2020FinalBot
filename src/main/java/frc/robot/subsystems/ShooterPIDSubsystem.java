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

public class ShooterPIDSubsystem extends PIDSubsystem {
  
  private final CANSparkMax m_motorRight;
  private final CANSparkMax m_motorLeft;
  private final CANEncoder m_encoderRight;
  private final CANEncoder m_encoderLeft;
  private CANSparkMax m_hoodMotor;
  private CANEncoder m_hoodEncoder;
  
  private final SimpleMotorFeedforward m_shooterFeedforward =
      new SimpleMotorFeedforward(ShooterConstants.KSVolts,
                                 ShooterConstants.KVVoltSecondsPerRotation);

  /**
   * The shooter subsystem for the robot.
   */
  public ShooterPIDSubsystem(int motorIDRight, int motorIDLeft) {
    super(new PIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD));
    m_motorRight = new CANSparkMax(motorIDRight, MotorType.kBrushless);
    m_motorLeft = new CANSparkMax(motorIDLeft, MotorType.kBrushless);
    m_hoodMotor = new CANSparkMax(HoodedShooterConstants.kHoodPort, MotorType.kBrushless);
    m_encoderRight = new CANEncoder(m_motorRight);
    m_encoderLeft = new CANEncoder(m_motorLeft);
    m_hoodEncoder = new CANEncoder(m_hoodMotor);

    // Spark 1 - Inverted - Not 
    // Spark 3 - Inverted - Yes
    m_motorLeft.follow(m_motorRight);
    m_motorRight.setInverted(true);
    m_motorLeft.setInverted(false);

    getController().setTolerance(ShooterConstants.kShooterToleranceRPS);
    setSetpoint(ShooterConstants.kShooterTargetRPS);
  }

  @Override
  public void useOutput(double output, double setpoint) {
    m_motorRight.setVoltage(output + m_shooterFeedforward.calculate(setpoint));
  }

  @Override
  public double getMeasurement() {
    return m_encoderRight.getVelocity(); //m_shooterEncoder.getRate();
  }

  public boolean atSetpoint() {
    return m_controller.atSetpoint();
  }

  public double getSetpoint() {
    return m_controller.getSetpoint();
  }
  
  public void stopShooter() {
    m_motorRight.set(0);
  }
  
  public void periodic() {
    super.periodic();
    SmartDashboard.putNumber("Right RPM 2", m_encoderRight.getVelocity());
    SmartDashboard.putNumber("Left RPM 2", m_encoderLeft.getVelocity());
  }

}