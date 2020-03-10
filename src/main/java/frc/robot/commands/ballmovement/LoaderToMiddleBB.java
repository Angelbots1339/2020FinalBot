/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.ballmovement;

import frc.robot.Constants.LoaderConstants;
import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class LoaderToMiddleBB extends CommandBase {

  // the middle beam break is right before the ball reaches the shooter, so the
  // ball is not touching the shooter

  private final IntakingSystem m_intaker;

  /**
   * loads balls from intake all the way to the middle beam break, ready to shoot
   * but not able to yet.
   */
  public LoaderToMiddleBB(IntakingSystem intaker) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_intaker = intaker;
    addRequirements(m_intaker.getIndexer(), m_intaker.getIntake(), m_intaker.getLoader());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (!m_intaker.isMiddleBeamBroken()) {
      m_intaker.getLoader().runSpeed(LoaderConstants.kInitLoaderSpeed);
    }
    m_intaker.getIndexer().enable();
    m_intaker.getIntake().enableIntake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // stops the loader when the middle beam break is broken
    if (m_intaker.isMiddleBeamBroken() && !m_intaker.isTopBeamBroken()) {
      m_intaker.getLoader().disable();
    } else if (m_intaker.isTopBeamBroken()) {
      m_intaker.getLoader().reverse(LoaderConstants.kReverseLoaderSpeed);
    }

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