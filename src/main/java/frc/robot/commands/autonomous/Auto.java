/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.ballmovement.ToggleIntakeArms;
import frc.robot.commands.vision.VisionShoot;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HoodPIDSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeArmPID;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.LoaderPIDSubsystem;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html

public class Auto extends SequentialCommandGroup {
  /**
   * Runs VisionShoot, then runs parallel command to drive robot in reverse and
   * lower intake arm
   */
  public Auto(IntakeArmPID arm, IntakeSubsystem intake, IndexerSubsystem index, LoaderPIDSubsystem loader,
      Shooter shooter, HoodPIDSubsystem hood, LimelightSubsystem limelight,
      DriveSubsystem drive) {
    addCommands(
        new VisionShoot(intake, index, loader, shooter, hood, limelight, drive, () -> false,
            () -> true, () -> 0, AutoConstants.kVisionTime),
        new ParallelCommandGroup(new ToggleIntakeArms(arm), new Reverse(drive)));
  }
}
