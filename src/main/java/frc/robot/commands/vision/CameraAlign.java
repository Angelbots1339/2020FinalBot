/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.vision;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class CameraAlign extends CommandBase {
  private final DriveSubsystem m_driveSubsystem;
  private final LimelightSubsystem m_limeLight;
  private final DoubleSupplier m_fwdMovement;
  private double m_turn;
  public double m_drive;
  private ShootingProfile m_targetProfile;
  private double driveError;

  /**
   * Creates a new Align. Aligns and focuses the camera to aim at target
   */
  public CameraAlign(DriveSubsystem driveSubsystem, LimelightSubsystem cameraSubsystem, ShootingProfile targetProfile,
      DoubleSupplier fwdMovement) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveSubsystem = driveSubsystem;
    addRequirements(m_driveSubsystem);

    m_limeLight = cameraSubsystem;
    addRequirements(m_limeLight);

    m_targetProfile = targetProfile;
    m_fwdMovement = fwdMovement;
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
      m_turn += m_targetProfile.getAngleP() * m_limeLight.getXTargetOffset();
      if (LimelightConstants.kDistanceAlign) {
        driveError = m_limeLight.getDistanceToVisionTarget() - m_targetProfile.getDistance();
        m_drive += driveError * LimelightConstants.kDriveP;
      }
      m_drive -= m_fwdMovement.getAsDouble();
    }
    m_limeLight.setAligned(m_driveSubsystem.arcadeDrive(m_drive, m_turn, LimelightConstants.kDriveTolerance,
        LimelightConstants.kShootTolerance));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_limeLight.setAligned(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}