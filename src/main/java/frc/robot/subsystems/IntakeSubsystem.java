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
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  private CANSparkMax m_rightIntakeArm;
  private CANSparkMax m_leftIntakeArm;
  private CANSparkMax m_leftIntakeMotor;
  
  /**
   * Creates a new IntakeSubsystem.
   */
  public IntakeSubsystem() {
    // Use addRequirements() here to declare subsystem dependencies.
    m_rightIntakeArm = new CANSparkMax(IntakeConstants.kRightIntakeMoverMotor, MotorType.kBrushless);
    m_leftIntakeArm = new CANSparkMax(IntakeConstants.kLeftIntakeMoverMotor, MotorType.kBrushless);
    
    m_leftIntakeMotor = new CANSparkMax(IntakeConstants.kIntakeMotor, MotorType.kBrushless);
  }

  public void enableMover(){
    m_rightIntakeArm.set(IntakeConstants.kMaxIntakeArmSpeed);
    m_leftIntakeArm.set(IntakeConstants.kMaxIntakeArmSpeed);
  }

  public void rotateIntake(double d){
    m_rightIntakeArm.set(d);
    m_leftIntakeArm.set(d);
  }

  public void disableDisableMover(){
    m_rightIntakeArm.set(0.0);
    m_leftIntakeArm.set(0.0);
  }

  public void enableIntake(){
    m_leftIntakeMotor.set(IntakeConstants.kMaxIntakeSpeed);
  }

  public void disableIntake(){
    m_leftIntakeMotor.set(0);
  }

  public void moveIntakeUp(){
    m_leftIntakeArm.set(IntakeConstants.kMaxIntakeArmSpeed);
  }
  public void moveIntakeDown(){
    m_leftIntakeArm.set(-IntakeConstants.kMaxIntakeArmSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("intake", m_leftIntakeMotor.get());
    SmartDashboard.putNumber("left move intake", m_leftIntakeArm.get());
    SmartDashboard.putNumber("right move intake", m_rightIntakeArm.get());
  }
}
