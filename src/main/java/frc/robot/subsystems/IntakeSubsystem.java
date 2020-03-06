/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DashboardConstants;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  private CANSparkMax m_intakeMotor;

  /**
   * Creates a new IntakeSubsystem.
   */
  public IntakeSubsystem() {
    m_intakeMotor = new CANSparkMax(IntakeConstants.kIntakeMotor, MotorType.kBrushless);
  }

  public void enableIntake() {
    m_intakeMotor.set(IntakeConstants.kMaxIntakeSpeed);
  }

  public void disableIntake() {
    m_intakeMotor.set(0);
  }

  public void reverseIntake() {
    m_intakeMotor.set(-1 * IntakeConstants.kMaxIntakeSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (DashboardConstants.kIntakeTelemetry) {
      SmartDashboard.putNumber("intake", m_intakeMotor.get());
    }
  }
}
