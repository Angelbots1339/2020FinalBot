/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.ballmovement;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LoaderSubsystem;
import frc.robot.subsystems.ShooterPID;

// brings the ball to the shooter

public class LoaderToTopBB extends CommandBase {
  // shoots the balls while the driver is holding down the button
  /**
   * Creates a new LoaderToShooter.
   */

   // uses loader, shooter, 
  private final LoaderSubsystem m_loader;
  private final IntakeSubsystem m_intake;
  private final IndexerSubsystem m_indexer;
  private final ShooterPID m_leftPID;
  private final ShooterPID m_rightPID;

  public LoaderToTopBB(LoaderSubsystem loader, IndexerSubsystem indexer, IntakeSubsystem intake, ShooterPID leftPID, ShooterPID rightPID) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_loader = loader;
    addRequirements(m_loader);

    m_intake = intake;
    addRequirements(m_intake);

    m_indexer = indexer;
    addRequirements(m_indexer);

    m_leftPID = leftPID;
    addRequirements(m_leftPID);

    m_rightPID = rightPID;
    addRequirements(m_rightPID);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // stage so PID goes first
    m_loader.enable();
    m_indexer.enable();
    m_intake.enableIntake();
    m_leftPID.enable();
    m_rightPID.enable();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // stops the shooter when top bb is broken
    if(m_loader.isTopBeamBroken()){
      m_loader.disable();
    }

    // when PID has recovered moves the loader
    if(m_leftPID.atSetpoint() && m_rightPID.atSetpoint()){
      m_loader.enable();
    }

    // if the indexer stales reverse until its good
    if(m_indexer.isCurrentSpike()){
      m_indexer.reverse();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_loader.disable();
    m_indexer.disable();
    m_intake.disableIntake();
    m_leftPID.disable();
    m_rightPID.disable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return m_loader.isTopBeamBroken();
    return false; // stops when button is released
  }
}
