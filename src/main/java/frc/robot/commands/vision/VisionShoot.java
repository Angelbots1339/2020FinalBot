/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.vision;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.ballmovement.RunShooter;
import frc.robot.commands.utils.DriveControl;
import frc.robot.commands.utils.VisionControl;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HoodPIDSubsystem;
import frc.robot.subsystems.IntakingSystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Shooter;

public class VisionShoot extends CommandBase {

  private static final List<ShootingProfile> data = getData();

  private final IntakingSystem m_intaker;
  private final Shooter m_shooter;
  private final LimelightSubsystem m_limelight;
  private final HoodPIDSubsystem m_hood;
  private final DriveSubsystem m_drive;

  private final RunShooter runShooter;
  private final RunHood runHood;
  private final CameraAlign cameraAlign;

  private final ShootingProfile latestProfile;

  private final VisionControl m_visionControl;
  private boolean m_useVision = true;
  private double m_setDistance = -1;

  /**
   * Shoots balls when aligned with the right perameters. If aligned and hood is
   * at correct set point it shoots.
   * 
   * @param m_targetProfile
   */
  public VisionShoot(IntakingSystem intaker, Shooter shooter, HoodPIDSubsystem hood, LimelightSubsystem limelight,
      DriveSubsystem drive, VisionControl visionControl, DriveControl driveControl) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intaker = intaker;
    addRequirements(m_intaker.getRequirements());

    m_shooter = shooter;
    addRequirements(m_shooter.getRequirements());
    m_hood = hood;
    addRequirements(m_hood);
    m_limelight = limelight;
    addRequirements(m_limelight);
    m_drive = drive;
    m_visionControl = visionControl;

    latestProfile = new ShootingProfile();
    runShooter = new RunShooter(m_shooter, latestProfile);
    runHood = new RunHood(m_hood, latestProfile);
    cameraAlign = new CameraAlign(m_drive, m_limelight, latestProfile, driveControl);
  }

  public VisionShoot(IntakingSystem intaker, Shooter shooter, HoodPIDSubsystem hood, LimelightSubsystem limelight,
      DriveSubsystem drive, DriveControl driveControl, double setDistance) {
    this(intaker, shooter, hood, limelight, drive, VisionControl.shootOnly, driveControl);
    m_useVision = false;
    m_setDistance = setDistance;
  }

  public VisionShoot(IntakingSystem intaker, Shooter shooter, HoodPIDSubsystem hood, LimelightSubsystem limelight,
      DriveSubsystem drive, boolean isAligning, boolean isShooting) {
    this(intaker, shooter, hood, limelight, drive, new VisionControl(isAligning, isShooting), DriveControl.empty);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    updateProfile();
    runShooter.initialize();
    runHood.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    updateProfile();

    if (m_visionControl.isAligning()) {
      if (!m_limelight.isAligning()) {
        cameraAlign.initialize();
        m_limelight.setAligning(true);
      }
      cameraAlign.execute();
    }
    if (!m_visionControl.isAligning() && m_limelight.isAligning()) {
      cameraAlign.end(false);
      m_limelight.setAligning(false);
    }

    runShooter.execute();
    runHood.execute();

    if (m_visionControl.isShooting()) {
      if (atSetpoint()) {
        m_intaker.enable(latestProfile.getIndexerSpeed(), latestProfile.getLoaderSpeed());
      } else if (m_limelight.getDistanceToVisionTarget() > ShooterConstants.kRapidShotThreshold
          && m_intaker.isMiddleBeamBroken()) {
        m_intaker.disable();
      }
    } else {
      m_intaker.disable();
    }
  }

  private boolean atSetpoint() {
    return m_shooter.atSetpoint() && m_hood.atSetpoint() && (m_limelight.isAligned() || !m_visionControl.isAligning());
  }

  private void updateProfile() {
    if (m_limelight.seesTarget() || !m_useVision) {
      double currentDist = m_useVision ? m_limelight.getDistanceToVisionTarget() : m_setDistance;
      m_limelight.setActiveProfile(data.stream()
          .min((a,
              b) -> (int) Math
                  .signum(Math.abs(a.getDistance() - currentDist) - Math.abs(b.getDistance() - currentDist)))
          .orElseThrow());
    }
    latestProfile.set(m_limelight.getLatestProfile());
  }

  public static List<ShootingProfile> getData() {
    var profilesArr = new ArrayList<ShootingProfile>();
    try (var br = new BufferedReader( // type of reader to read text file
        new FileReader(Filesystem.getDeployDirectory().getCanonicalPath() + File.separator + "shooterProfiles.data"))) {
      String line;
      while ((line = br.readLine()) != null) {
        if (!line.startsWith("//") || line.isBlank()) {
          profilesArr.add(new ShootingProfile(line));
        }
      }
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
    if (m_limelight.isAligning()) {
      cameraAlign.end(interrupted);
      m_limelight.setAligning(false);
    }
    m_intaker.disable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
