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
import java.util.stream.Collectors;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.ballmovement.RunShooter;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HoodPIDSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.LoaderSubsystem;
import frc.robot.subsystems.ShooterPID;

public class VisionShoot extends CommandBase {

  private static final ArrayList<ShootingProfiles> data = getData();
  private final IntakeSubsystem m_intake;
  private final IndexerSubsystem m_indexer;
  private final LoaderSubsystem m_loader;
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

  /**
   * Creates a new ShootAllBalls.
   * 
   * @param m_targetProfile
   */
  public VisionShoot(IntakeSubsystem intake, IndexerSubsystem index, LoaderSubsystem loader, ShooterPID leftShooter,
      ShooterPID rightShooter, HoodPIDSubsystem hood, LimelightSubsystem limelight, DriveSubsystem drive,
      BooleanSupplier isAligning, BooleanSupplier isShooting) {
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
    m_drive = drive;
    m_isAligning = isAligning;
    m_isShooting = isShooting;

    latestProfile = new ShootingProfiles(0, 0, 0, 0, 0);
    runShooter = new RunShooter(m_leftShooter, m_rightShooter, latestProfile);
    runHood = new RunHood(m_hood, latestProfile);
    cameraAlign = new CameraAlign(m_drive, m_limelight, latestProfile);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    runShooter.schedule();
    runHood.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("hi");
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
    if (m_isAligning.getAsBoolean() && !m_limelight.isAligning()) {
      cameraAlign.schedule();
      m_limelight.setAligning(true);
    }
    if (m_isShooting.getAsBoolean()) {
      if (m_leftShooter.atSetpoint() && m_rightShooter.atSetpoint() && m_hood.atSetpoint()
          && (m_limelight.isAligned() || !m_isAligning.getAsBoolean())) {
        // m_intake.enableIntake();
        m_indexer.enable(latestProfile.getLoadingSpeed());
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
      cameraAlign.cancel();
      m_limelight.setAligning(false);
    }
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
    runShooter.cancel();
    runHood.cancel();
    cameraAlign.cancel();
    m_limelight.setAligning(false);
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
