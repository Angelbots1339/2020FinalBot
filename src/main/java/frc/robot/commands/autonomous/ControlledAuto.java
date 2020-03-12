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
import frc.robot.subsystems.IntakeArmPID;
import frc.robot.subsystems.IntakingSystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html

public class ControlledAuto extends SequentialCommandGroup {
    /**
     * Runs VisionShoot, then runs parallel command to drive robot in reverse and
     * lower intake arm
     */
    public ControlledAuto(IntakeArmPID arm, IntakingSystem intaker, Shooter shooter, HoodPIDSubsystem hood,
            LimelightSubsystem limelight, DriveSubsystem drive) {
        addCommands(
                new Timeout(new VisionShoot(intaker, shooter, hood, limelight, drive, false, true), AutoConstants.kVisionTime),
                new ParallelCommandGroup(new ToggleIntakeArms(arm),
                        new Timeout(new PIDDrive(drive, 0, -1, drive::getRotation, drive::getForwardMeters), 2)));
    }
}
