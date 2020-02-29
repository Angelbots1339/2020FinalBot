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

  private final IntakeArmPID m_rightArm;
  private final IntakeArmPID m_leftArm;

  /**
   * Creates a new RunIntakeArms.
   */
  public RunIntakeArms(IntakeArmPID right, IntakeArmPID left){
    // Use addRequirements() here to declare subsystem dependencies.
    m_rightArm = right;
    addRequirements(m_rightArm);

    m_leftArm = left;
    addRequirements(m_leftArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_leftArm.runIntakeArms();
    m_rightArm.runIntakeArms();
    m_leftArm.enable();
    m_rightArm.enable();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_leftArm.disable();
    m_rightArm.disable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
