/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.ballmovement;
import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj2.command.CommandBase;

// add an if bb is triggered only the indexer and loader runs 
public class BallToBottomBB extends CommandBase {

  //private final PowerCellMovement m_PowerCellMovement;
  private final IndexerSubsystem m_indexer;
  private final IntakeSubsystem m_intake;
  private final LoaderSubsystem m_loader;
  /**
   * Creates a new BallToStage1.
   */
  public BallToBottomBB(IndexerSubsystem indexer, IntakeSubsystem intake, LoaderSubsystem loader){
    // Use addRequirements() here to declare subsystem dependencies.
    m_indexer = indexer;
    addRequirements(indexer);
    m_intake = intake;
    addRequirements(intake);
    m_loader = loader;
    addRequirements(loader);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // checks to see if Bottom beam break is broken
    // may need a filter
    // may need to add a timeout
    return m_loader.isBottomBroken();
  
  }
}
