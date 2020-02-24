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
public class RunIntakeIndex extends CommandBase {

  private final IndexerSubsystem m_indexer;
  private final IntakeSubsystem m_intake;

  public RunIntakeIndex(IndexerSubsystem indexer, IntakeSubsystem intake){
    // Use addRequirements() here to declare subsystem dependencies.
    m_indexer = indexer;
    addRequirements(indexer);
    m_intake = intake;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_indexer.enable();
    m_intake.enableIntake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  //@Override
  /*
  public void execute() {
    if(m_indexer.isCurrentSpike()){
      m_indexer.reverse(); //TODO
    }
    
  }
  */

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_indexer.disable();
    m_intake.disableIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return false;
  
  }
}
