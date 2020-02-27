/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.vision;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.commands.util.ShootingProfiles;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class CameraAlign extends CommandBase {
  /**
   * Creates a new Align.
   */
  private final DriveSubsystem m_driveSubsystem;
  private final LimelightSubsystem m_limeLight;
  private double m_turn;
  public double m_drive;
  private ShootingProfiles m_targetProfile;
  private double driveError;
  private boolean isFinished = false;

  public CameraAlign(DriveSubsystem driveSubsystem, LimelightSubsystem cameraSubsystem, ShootingProfiles targetProfile) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveSubsystem = driveSubsystem;
    addRequirements(m_driveSubsystem);

    m_limeLight = cameraSubsystem;
    addRequirements(m_limeLight);

    m_targetProfile = targetProfile;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive = 0;
    m_turn = 0;

    if (m_limeLight.seesTarget()) {
      driveError = m_limeLight.getDistanceToVisionTarget() - m_targetProfile.getDistance();
      m_turn += Constants.LimelightConstants.kAngleP * m_limeLight.getXTargetOffset();
      m_drive += driveError * Constants.LimelightConstants.kDriveP;
    }

    SmartDashboard.putNumber("Vision Turn", m_turn);
    SmartDashboard.putNumber("Vision Drive", m_drive);
    SmartDashboard.putNumber("Vision Drive Error", driveError);
    isFinished = m_driveSubsystem.arcadeDrive(m_drive, m_turn, Constants.LimelightConstants.kDriveTolerance);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}