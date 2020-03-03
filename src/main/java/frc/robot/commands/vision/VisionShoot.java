/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.vision;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.stream.Collectors;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.ballmovement.RunShooter;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HoodPIDSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.LoaderPIDSubsystem;
import frc.robot.subsystems.ShooterPID;

public class VisionShoot extends CommandBase {

  private static final ArrayList<ShootingProfiles> data = getData();
  private final IntakeSubsystem m_intake;
  private final IndexerSubsystem m_indexer;
  private final LoaderPIDSubsystem m_loader;
  private final ShooterPID m_leftShooter;
  private final ShooterPID m_rightShooter;
  private final LimelightSubsystem m_limelight;
  private final HoodPIDSubsystem m_hood;
  private final DriveSubsystem m_drive;
  private final Command runShooter;
  private final Command runHood;
  private final Command cameraAlign;
  private final ShootingProfiles latestProfile;
  private final BooleanSupplier m_isAligning;
  private final BooleanSupplier m_isShooting;
  private double m_startTime, m_currentTime;
  private final double m_timeout;

  /**
   * Creates a new ShootAllBalls.
   * 
   * @param m_targetProfile
   */
  public VisionShoot(IntakeSubsystem intake, IndexerSubsystem index, LoaderPIDSubsystem loader, ShooterPID leftShooter,
      ShooterPID rightShooter, HoodPIDSubsystem hood, LimelightSubsystem limelight, DriveSubsystem drive,
      BooleanSupplier isAligning, BooleanSupplier isShooting, DoubleSupplier fwdMovement, double timeout) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = intake;
    addRequirements(m_intake);

    m_indexer = index;
    addRequirements(m_indexer);

    m_loader = loader;
    addRequirements(m_loader);

    m_leftShooter = leftShooter;
    m_rightShooter = rightShooter;
    addRequirements(m_leftShooter, m_rightShooter);
    m_hood = hood;
    addRequirements(m_hood);
    m_limelight = limelight;
    addRequirements(m_limelight);
    m_drive = drive;
    m_isAligning = isAligning;
    m_isShooting = isShooting;

    latestProfile = new ShootingProfiles(0, 0, 0, 0, 0, 0);
    runShooter = new RunShooter(m_leftShooter, m_rightShooter, latestProfile);
    runHood = new RunHood(m_hood, latestProfile);
    cameraAlign = new CameraAlign(m_drive, m_limelight, latestProfile, fwdMovement);
    m_timeout = timeout;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    updateProfile();
    runShooter.initialize();
    runHood.initialize();
    m_startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    updateProfile();
    if (m_isAligning.getAsBoolean() && !m_limelight.isAligning()) {
      cameraAlign.initialize();
      m_limelight.setAligning(true);
    }
    if(m_isAligning.getAsBoolean()){
      cameraAlign.execute();
    }
    runShooter.execute();
    runHood.execute();
    if (m_isShooting.getAsBoolean()) {
      if (((m_leftShooter.atSetpoint() && m_rightShooter.atSetpoint())
          || m_limelight.getDistanceToVisionTarget() < ShooterConstants.kRapidShotThreshold) && m_hood.atSetpoint()
          && (m_limelight.isAligned() || !m_isAligning.getAsBoolean())) {
        m_intake.enableIntake();
        m_indexer.enable(latestProfile.getIndexerSpeed());
        m_loader.setSetpoint(latestProfile.getLoaderSpeed());
        m_loader.enable();
      } else {
        if (m_loader.isMiddleBeamBroken()) {
          m_intake.disableIntake();
          m_indexer.disable();
          m_loader.disable();
        }
      }
    } else {
      m_intake.disableIntake();
      m_indexer.disable();
      m_loader.disable();
    }
    if (!m_isAligning.getAsBoolean() && m_limelight.isAligning()) {
      cameraAlign.end(false);
      m_limelight.setAligning(false);
    }
  }

  private void updateProfile() {
    if (m_limelight.seesTarget()) {
      double currentDist = m_limelight.getDistanceToVisionTarget();
      m_limelight
          .setActiveProfile(data.stream()
              .collect(Collectors.minBy((a,
                  b) -> (int) Math
                      .signum(Math.abs(a.getDistance() - currentDist) - Math.abs(b.getDistance() - currentDist))))
              .get());
    }
    latestProfile.set(m_limelight.getLatestProfile());
  }

  public static ArrayList<ShootingProfiles> getData() {
    var profilesArr = new ArrayList<ShootingProfiles>();
    try {
      var br = new BufferedReader( // type of reader to read text file
          new FileReader(Filesystem.getDeployDirectory().getCanonicalPath() + File.separator + "shooterProfiles.data"));
      String line;
      while ((line = br.readLine()) != null) {
        if (!line.startsWith("//"))
          profilesArr.add(new ShootingProfiles(line)); // hr = hoodRotations
      }
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

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    runShooter.end(false);
    runHood.end(false);
    cameraAlign.end(false);
    m_limelight.setAligning(false);
    m_intake.disableIntake();
    m_indexer.disable();
    m_loader.disable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    m_currentTime = Timer.getFPGATimestamp();
    if(m_currentTime - m_startTime >= m_timeout){
      m_drive.arcadeDrive(0, 0);
      return true;
    }
    return false;
  }
}
