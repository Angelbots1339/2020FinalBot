/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.stream.Collectors;

import edu.wpi.first.wpilibj.Filesystem;
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
  private static ArrayList<ShootingProfiles> data = getData();

  public CameraAlign(DriveSubsystem driveSubsystem, LimelightSubsystem cameraSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveSubsystem = driveSubsystem;
    m_limeLight = cameraSubsystem;
    addRequirements(m_limeLight);
    addRequirements(m_driveSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double dist = m_limeLight.getDistanceToVisionTarget();
    m_targetProfile = data.stream()
        .collect(Collectors
            .minBy((a, b) -> (int) Math.signum(Math.abs(a.getDistance() - dist) - Math.abs(b.getDistance() - dist))))
        .get();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive = 0;
    m_turn = 0;

    if (m_limeLight.seesTarget()) {
      double driveError = m_limeLight.getDistanceToVisionTarget() - m_targetProfile.getDistance();
      m_turn += Constants.LimelightConstants.kAngleP * m_limeLight.getXTargetOffset();
      m_drive += -driveError * Constants.LimelightConstants.kDriveP;
    }

    m_driveSubsystem.tankDrive(m_drive + m_turn, m_drive - m_turn, Constants.LimelightConstants.kDriveTolerance);
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

  public static ArrayList<ShootingProfiles> getData() {
    var ret = new ArrayList<ShootingProfiles>();
    try {
      var br = new BufferedReader(
          new FileReader(Filesystem.getDeployDirectory().getCanonicalPath() + File.separator + "shooterProfiles.data"));
      String line;
      while ((line = br.readLine()) != null)
        ret.add(new ShootingProfiles(getProperty(line, "m"), getProperty(line, "rpm"), getProperty(line, "clicks")));
      br.close();
    } catch (FileNotFoundException e) {
      e.printStackTrace();
    } catch (IOException e) {
      e.printStackTrace();
    }
    return ret;
  }

  private static double getProperty(String str, String key) {
    return Double.parseDouble(str.replaceAll(".*?(\\d+)" + key + ".*", "$1"));
  }
}