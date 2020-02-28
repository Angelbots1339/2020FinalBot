/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.util.ShootingProfiles;
import frc.robot.subsystems.HoodPIDSubsystem;

public class RunHood extends CommandBase {

  private final HoodPIDSubsystem m_hoodPID;
  private ShootingProfiles m_targetProfile;

  /**
   * Creates a new RunHood.
   */
  public RunHood(HoodPIDSubsystem hoodPID, ShootingProfiles targetProfile) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_hoodPID = hoodPID;
    addRequirements(m_hoodPID);

    m_targetProfile = targetProfile;
  }

  public RunHood(HoodPIDSubsystem hoodPID, double targetHood){
    this(hoodPID, new ShootingProfiles(-1, -1, targetHood));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_hoodPID.setSetpoint(m_targetProfile.getHoodvalue());
    m_hoodPID.enable();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_hoodPID.disable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
