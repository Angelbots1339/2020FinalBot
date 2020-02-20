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
  private CANSparkMax m_rightMoveIntake;
  private CANSparkMax m_leftMoveIntake;
  private CANSparkMax m_leftIntakeMotor;
  
  /**
   * Creates a new IntakeSubsystem.
   */
  public IntakeSubsystem() {
    // Use addRequirements() here to declare subsystem dependencies.
    m_rightMoveIntake = new CANSparkMax(IntakeConstants.kRightIntakeMoverMotor, MotorType.kBrushless);
    m_leftMoveIntake = new CANSparkMax(IntakeConstants.kLeftIntakeMoverMotor, MotorType.kBrushless);
    
    m_leftIntakeMotor = new CANSparkMax(IntakeConstants.kIntakeMotor, MotorType.kBrushless);
  }

  public void enableMover(){
    m_rightMoveIntake.set(IntakeConstants.kMaxIntakeMoverSpeed);
    m_leftMoveIntake.set(IntakeConstants.kMaxIntakeMoverSpeed);
  }

  public void rotateIntake(double d){
    m_rightMoveIntake.set(d);
    m_leftMoveIntake.set(d);
  }

  public void disableDisableMover(){
    m_rightMoveIntake.set(0.0);
    m_leftMoveIntake.set(0.0);
  }

  public void enableIntake(){
    m_leftIntakeMotor.set(IntakeConstants.kMaxIntakeSpeed);
  }

  public void disableIntake(){
    m_leftIntakeMotor.set(0);
  }

  public void moveIntakeUp(){
    m_leftMoveIntake.set(IntakeConstants.kMaxIntakeMoverSpeed);
  }
  public void moveIntakeDown(){
    m_leftMoveIntake.set(-IntakeConstants.kMaxIntakeMoverSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("intake", m_leftIntakeMotor.get());
    SmartDashboard.putNumber("left move intake", m_leftMoveIntake.get());
    SmartDashboard.putNumber("right move intake", m_rightMoveIntake.get());
  }
}
