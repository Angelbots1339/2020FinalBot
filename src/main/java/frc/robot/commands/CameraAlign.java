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
import java.util.stream.Collectors;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.commands.ballmovement.RunShooter;
import frc.robot.commands.ballmovement.ShootAllBalls;
import frc.robot.commands.util.ShootingProfiles;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.LoaderSubsystem;
import frc.robot.subsystems.ShooterPID;

public class CameraAlign extends CommandBase {
  /**
   * Creates a new Align.
   */
  private final DriveSubsystem m_driveSubsystem;
  private final LimelightSubsystem m_limeLight;
  private final ShooterPID m_leftPID;
  private double m_turn;
  public double m_drive;
  private ShootingProfiles m_targetProfile;
  private static ArrayList<ShootingProfiles> data = getData();
  private ShooterPID m_rightPID;
  private double driveError;
  private final IntakeSubsystem m_intake;
  private final IndexerSubsystem m_indexer;
  private final LoaderSubsystem m_loader;

  public CameraAlign(DriveSubsystem driveSubsystem, LimelightSubsystem cameraSubsystem, ShooterPID leftPID,
      ShooterPID rightPID, IndexerSubsystem index, IntakeSubsystem intake, LoaderSubsystem loader) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveSubsystem = driveSubsystem;
    addRequirements(m_driveSubsystem);

    m_limeLight = cameraSubsystem;
    addRequirements(m_limeLight);

    m_rightPID = rightPID;
    m_leftPID = leftPID;
    m_intake = intake;
    m_indexer = index;
    m_loader = loader;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double currentDist = m_limeLight.getDistanceToVisionTarget();
    /**
     * compares the profiles and current distance to find the correct profile(text
     * files) to use
     */
    m_targetProfile = data.stream().collect(Collectors.minBy(
        (a, b) -> (int) Math.signum(Math.abs(a.getDistance() - currentDist) - 
        Math.abs(b.getDistance() - currentDist))))
        .get();
    new RunShooter(m_leftPID, m_rightPID, m_targetProfile.getShooterSpeed()).schedule();; // spins up before it gets to the target
                                                                              // profile
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive = 0;
    m_turn = 0;

    if (m_limeLight.seesTarget()) {
      driveError = m_limeLight.getDistanceToVisionTarget() - m_targetProfile.getDistance();
      m_turn += Constants.LimelightConstants.kAngleP * m_limeLight.getXTargetOffset();
      m_drive += -driveError * Constants.LimelightConstants.kDriveP;
    }

    m_driveSubsystem.arcadeDrive(m_drive, m_turn, Constants.LimelightConstants.kDriveTolerance);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(!interrupted){
      new ShootAllBalls(m_intake, m_indexer, m_loader);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(driveError) > LimelightConstants.kDistanceTolerance
        && Math.abs(m_limeLight.getXTargetOffset()) > LimelightConstants.kAngleTolerance;
  }

  public static ArrayList<ShootingProfiles> getData() {
    var profilesArr = new ArrayList<ShootingProfiles>();
    try {
      var br = new BufferedReader( // type of reader to read text file
          new FileReader(Filesystem.getDeployDirectory().getCanonicalPath() + File.separator + "shooterProfiles.data")); // getting path to text file
      String line;
      while ((line = br.readLine()) != null)
        profilesArr.add(new ShootingProfiles(getProperty(line, "m"), getProperty(line, "rpm"), getProperty(line, "clicks")));
      br.close(); // stops the reader
    } 
    // if file is not found, stack trace is printed
      catch (FileNotFoundException e) {
      e.printStackTrace(); // list of methods that happened up to error
    } 
    // fail to read file 
      catch (IOException e) {
      e.printStackTrace();
    }
    return profilesArr;
  }

  /**
   * uses regxr to look through the text file
   * 
   * @param str Line of text in text file
   * @param key Leters behind numbers in str
   * @return the number before the key
   */
  private static double getProperty(String str, String key) {
    return Double.parseDouble(str.replaceAll(".*?(\\d+)" + key + ".*", "$1"));
  }
}