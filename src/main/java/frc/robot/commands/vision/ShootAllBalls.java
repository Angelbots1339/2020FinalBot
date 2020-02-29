/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.ballmovement.RunShooter;
import frc.robot.subsystems.HoodPIDSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.LoaderSubsystem;
import frc.robot.subsystems.ShooterPID;

public class ShootAllBalls extends CommandBase {

  private IntakeSubsystem m_intake;
  private IndexerSubsystem m_indexer;
  private LoaderSubsystem m_loader;
  private ShooterPID m_leftShooter;
  private ShooterPID m_rightShooter;
  private LimelightSubsystem m_limelight;
  private HoodPIDSubsystem m_hood;
  private Command runShooter;
  private Command runHood;
  private ShootingProfiles latestProfile;

  /**
   * Creates a new ShootAllBalls.
   * 
   * @param m_targetProfile
   */
  public ShootAllBalls(IntakeSubsystem intake, IndexerSubsystem index, LoaderSubsystem loader, ShooterPID leftShooter,
      ShooterPID rightShooter, HoodPIDSubsystem hood, LimelightSubsystem limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = intake;
    addRequirements(m_intake);

    m_indexer = index;
    addRequirements(m_indexer);

    m_loader = loader;
    addRequirements(m_loader);

    m_leftShooter = leftShooter;
    m_rightShooter = rightShooter;
    m_hood = hood;
    m_limelight = limelight;
    runShooter = new RunShooter(m_leftShooter, m_rightShooter, new ShootingProfiles(0,0,0,0,0));
    runHood = new RunHood(m_hood, new ShootingProfiles(0,0,0,0,0));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    latestProfile = m_limelight.getLatestProfile();

    if (!m_limelight.isAligning()) {
      runShooter = new RunShooter(m_leftShooter, m_rightShooter, latestProfile);
      runHood = new RunHood(m_hood, latestProfile);
      runShooter.schedule();
      runHood.schedule();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_leftShooter.atSetpoint() && m_rightShooter.atSetpoint() && m_limelight.isAligned()) {
      m_intake.enableIntake();
      m_indexer.enable(latestProfile.getLoadingSpeed());
      m_loader.enable();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!m_limelight.isAligning()) {
      runShooter.cancel();
      runHood.cancel();
    }
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
