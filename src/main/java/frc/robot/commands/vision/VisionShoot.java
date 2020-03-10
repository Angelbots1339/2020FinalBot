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
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.ballmovement.RunShooter;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HoodPIDSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.LoaderPIDSubsystem;
import frc.robot.subsystems.Shooter;

public class VisionShoot extends CommandBase {

  private static final ArrayList<ShootingProfile> data = getData();
  private final IntakeSubsystem m_intake;
  private final IndexerSubsystem m_indexer;
  private final LoaderPIDSubsystem m_loader;
  private final Shooter m_shooter;
  private final LimelightSubsystem m_limelight;
  private final HoodPIDSubsystem m_hood;
  private final DriveSubsystem m_drive;
  private final RunShooter runShooter;
  private final RunHood runHood;
  private final CameraAlign cameraAlign;
  private final ShootingProfile latestProfile;
  private final BooleanSupplier m_isAligning;
  private final BooleanSupplier m_isShooting;
  private double m_startTime, m_currentTime;
  private final double m_timeout;
  private final boolean m_useVision;
  private final double m_setDistance;

  /**
   * Shoots balls when aligned with the right perameters. If aligned and hood is
   * at correct set point it shoots.
   * 
   * @param m_targetProfile
   */
  public VisionShoot(IntakeSubsystem intake, IndexerSubsystem index, LoaderPIDSubsystem loader, Shooter shooter,
      HoodPIDSubsystem hood, LimelightSubsystem limelight, DriveSubsystem drive, BooleanSupplier isAligning,
      BooleanSupplier isShooting, DoubleSupplier fwdMovement, double timeout, boolean useVision, double setDistance) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = intake;
    addRequirements(m_intake);

    m_indexer = index;
    addRequirements(m_indexer);

    m_loader = loader;
    addRequirements(m_loader);

    m_shooter = shooter;
    addRequirements(m_shooter.getLeft(), m_shooter.getRight());
    m_hood = hood;
    addRequirements(m_hood);
    m_limelight = limelight;
    addRequirements(m_limelight);
    m_drive = drive;
    m_isAligning = isAligning;
    m_isShooting = isShooting;

    latestProfile = new ShootingProfile(0, 0, 0, 0, 0, 0);
    runShooter = new RunShooter(m_shooter, latestProfile);
    runHood = new RunHood(m_hood, latestProfile);
    cameraAlign = new CameraAlign(m_drive, m_limelight, latestProfile, fwdMovement);
    m_timeout = timeout;
    m_useVision = useVision;
    m_setDistance = setDistance;
  }

  public VisionShoot(IntakeSubsystem intake, IndexerSubsystem index, LoaderPIDSubsystem loader, Shooter shooter,
      HoodPIDSubsystem hood, LimelightSubsystem limelight, DriveSubsystem drive, BooleanSupplier isAligning,
      BooleanSupplier isShooting, DoubleSupplier fwdMovement, double timeout) {
    this(intake, index, loader, shooter, hood, limelight, drive, isAligning, isShooting, fwdMovement, timeout, true,
        -1);
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
    if (m_isAligning.getAsBoolean()) {
      cameraAlign.execute();
    }
    runShooter.execute();
    runHood.execute();
    if (m_isShooting.getAsBoolean()) {
      if (m_shooter.atSetpoint() && m_hood.atSetpoint() && (m_limelight.isAligned() || !m_isAligning.getAsBoolean())) {
        m_intake.enableIntake();
        m_indexer.enable(latestProfile.getIndexerSpeed());
        m_loader.setSetpoint(latestProfile.getLoaderSpeed());
        m_loader.enable();
      } else if (m_limelight.getDistanceToVisionTarget() > ShooterConstants.kRapidShotThreshold
          && m_loader.isMiddleBeamBroken()) {
        disableIntaking();
      }
    } else {
      disableIntaking();
    }
    if (!m_isAligning.getAsBoolean() && m_limelight.isAligning()) {
      cameraAlign.end(false);
      m_limelight.setAligning(false);
    }
  }

  private void updateProfile() {
    if (m_limelight.seesTarget() || !m_useVision) {
      double currentDist = m_useVision ? m_limelight.getDistanceToVisionTarget() : m_setDistance;
      m_limelight
          .setActiveProfile(data.stream()
              .collect(Collectors.minBy((a,
                  b) -> (int) Math
                      .signum(Math.abs(a.getDistance() - currentDist) - Math.abs(b.getDistance() - currentDist))))
              .get());
    }
    latestProfile.set(m_limelight.getLatestProfile());
  }

  public static ArrayList<ShootingProfile> getData() {
    var profilesArr = new ArrayList<ShootingProfile>();
    try {
      var br = new BufferedReader( // type of reader to read text file
          new FileReader(Filesystem.getDeployDirectory().getCanonicalPath() + File.separator + "shooterProfiles.data"));
      String line;
      while ((line = br.readLine()) != null) {
        if (!line.startsWith("//") || line.isBlank())
          profilesArr.add(new ShootingProfile(line));
      }
      br.close(); // stops the reader
    } catch (FileNotFoundException e) {
      e.printStackTrace();
    } catch (IOException e) {
      e.printStackTrace();
    }
    return profilesArr;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    runShooter.end(interrupted);
    runHood.end(interrupted);
    cameraAlign.end(interrupted);
    m_limelight.setAligning(false);
    disableIntaking();
  }

  public void disableIntaking() {
    m_intake.disableIntake();
    m_indexer.disable();
    m_loader.disable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    m_currentTime = Timer.getFPGATimestamp();
    return m_currentTime - m_startTime >= m_timeout;
  }
}
