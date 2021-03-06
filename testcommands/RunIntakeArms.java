/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.ballmovement;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeArmPID;

/**
 * currently testing this one
 */
public class RunIntakeArms extends CommandBase {

  private final IntakeArmPID m_arm;
  private double m_setpoint;
  /**
   * Creates a setpoint for the IntakeArm to move to
   */
  public RunIntakeArms(IntakeArmPID arm, double setpoint){
    // Use addRequirements() here to declare subsystem dependencies.
    m_arm = arm;
    m_setpoint = setpoint;
    addRequirements(m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.setSetpoint(m_setpoint);
    m_arm.enable();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.disable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
