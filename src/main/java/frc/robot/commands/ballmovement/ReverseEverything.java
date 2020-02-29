/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.ballmovement;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LoaderConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LoaderSubsystem;

public class ReverseEverything extends CommandBase {
  /**
   * Creates a new ReverseEverything.
   */
  private IntakeSubsystem m_intake;
  private IndexerSubsystem m_index;
  private LoaderSubsystem m_loader;

  public ReverseEverything(LoaderSubsystem loader, IntakeSubsystem intake, IndexerSubsystem indexer) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = intake;
    addRequirements(m_intake);

    m_index = indexer;
    addRequirements(m_index);

    m_loader = loader;
    addRequirements(m_loader);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.reverseIntake();
    m_index.reverse();
    m_loader.reverse(LoaderConstants.kInitLoaderSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.disableIntake();
    m_index.disable();
    m_loader.disable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
