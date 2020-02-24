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

public class ShootAllBalls extends CommandBase {

  private IntakeSubsystem m_intake;
  private IndexerSubsystem m_indexer;
  private LoaderSubsystem m_loader;
  /**
   * Creates a new ShootAllBalls.
   */
  public ShootAllBalls(IntakeSubsystem intake, IndexerSubsystem index, LoaderSubsystem loader) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = intake;
    addRequirements(m_intake);

    m_indexer = index;
    addRequirements(m_indexer);

    m_loader = loader;
    addRequirements(m_loader);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      m_intake.enableIntake();
      m_indexer.enable();
      m_loader.enable();  
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.disableIntake();
    m_indexer.disable();
    m_loader.disable();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
