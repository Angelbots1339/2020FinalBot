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

public class LoaderToMiddleBB2 extends CommandBase {

  // the middle beam break is right before the ball reaches the shooter, so the
  // ball is not touching the shooter

  private final LoaderPIDSubsystem m_loader;
  private final IntakeSubsystem m_intake;
  private final IndexerSubsystem m_indexer;
 
  public LoaderToMiddleBB2(LoaderPIDSubsystem loader, IntakeSubsystem intake, IndexerSubsystem indexer) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_loader = loader;
    addRequirements(m_loader);

    m_intake = intake;
    addRequirements(m_intake);

    m_indexer = indexer;
    addRequirements(m_indexer);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (!m_loader.isMiddleBeamBroken()) {
      m_loader.runSpeed(LoaderConstants.kInitLoaderSpeed);
    }
    m_indexer.enable();
    m_intake.enableIntake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // stops the loader when the middle beam break is broken
    if (m_loader.isMiddleBeamBroken() && !m_loader.isTopBeamBroken()) {
      m_loader.disable();
    } else if (m_loader.isTopBeamBroken()) {
      m_loader.reverse(LoaderConstants.kReverseLoaderSpeed);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_loader.disable();
    m_intake.disableIntake();
    m_indexer.disable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // checks to see if Bottom beam break is broken
    // may need a filter
    // may need to add a timeout
    // return m_loader.isTopBeamBroken();
    // stops when button stops being held
    return false;
  }
}