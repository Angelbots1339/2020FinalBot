/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.ballmovement;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakingSystem;

public class ReverseEverything extends CommandBase {

  private IntakingSystem m_intaker;

  /**
   * Reverses loader, intake, and indexer
   */
  public ReverseEverything(IntakingSystem intaker) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intaker = intaker;
    addRequirements(m_intaker.getIndexer(), m_intaker.getIntake(), m_intaker.getLoader());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intaker.reverse();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intaker.disable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
