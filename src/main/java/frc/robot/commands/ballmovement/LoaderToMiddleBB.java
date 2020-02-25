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

  // the middle beam break is right before the ball reaches the shooter, so the ball is not touching the shooter

  private final LoaderSubsystem m_loader;
  private final IntakeSubsystem m_intake;
  private final IndexerSubsystem m_indexer;
  private int m_spikeCount = 0;
  private int m_reverseCount = 0;

  public LoaderToMiddleBB(LoaderSubsystem loader, IntakeSubsystem intake, IndexerSubsystem indexer){
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
    m_loader.enable(LoaderConstants.kInitLoaderSpeed);
    m_indexer.enable();
    m_intake.enableIntake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Load the first ball, stops the loader when the middle beam break is broken
    if(m_loader.isMiddleBeamBroken() && !m_loader.isTopBeamBroken()){
      m_loader.disable();
    } // when the speed is too fast, the top BB will trip and we need to reverse it
    else if(m_loader.isTopBeamBroken()){
      m_loader.reverse(LoaderConstants.kInitLoaderSpeed);
    }

    // if top broken, then reverse
    // if top not broken and middle broken stop
    // if the indexer stales reverse until its good - may need to add a counter to see if the current spike is happening for longer 
    /*
    THIS WAS FOR CONTINGENCY, HAS NOT BEEN USED
    if(m_indexer.isCurrentSpike()){
      m_spikeCount++;
      if (m_spikeCount >= 20) {
        m_indexer.reverse();
        m_reverseCount = 20;
        m_spikeCount = 0;
      }
    }
    else{
      if (m_reverseCount > 0) {
        m_reverseCount--;
      }
      else {
        m_indexer.enable();
      }
    }
    */
  
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
    // return m_loader.isTopBeamBroken();
    // stops when button stops being held
    return false;
  }
}
