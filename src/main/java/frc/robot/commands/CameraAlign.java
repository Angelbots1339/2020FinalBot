/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
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
  private double m_targetDistance;

  public CameraAlign(DriveSubsystem driveSubsystem, LimelightSubsystem cameraSubsystem, double targetDistance) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveSubsystem = driveSubsystem;
    m_limeLight = cameraSubsystem;
    m_targetDistance = targetDistance;
    addRequirements(m_limeLight);
    addRequirements(m_driveSubsystem);

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
    double driveError = m_limeLight.getArea() - m_targetDistance;

    if (m_limeLight.seesTarget())
      if (Math.abs(m_limeLight.getXTargetOffset()) > Constants.LimelightConstants.kXAlignTolerance)
        m_turn = Constants.LimelightConstants.kP * m_limeLight.getXTargetOffset()
            + Math.copySign(Constants.DriveConstants.kMinPower, m_limeLight.getXTargetOffset());

      else if (Math.abs(driveError) > Constants.LimelightConstants.kDriveTolerance)
        m_drive = Math.copySign(m_limeLight.getArea() * Constants.LimelightConstants.kDriveP + Constants.DriveConstants.kMinPower,
            -driveError);

    m_driveSubsystem.tankDrive(m_drive + m_turn, m_drive - m_turn);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // m_driveSubsystem.tankDriveVolts(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}