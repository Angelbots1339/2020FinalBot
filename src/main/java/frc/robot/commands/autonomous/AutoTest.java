/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AngleConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.utils.DriveControl;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoTest extends SequentialCommandGroup {
  /**
   * Creates a new AutoTest.
   */
  public AutoTest(DriveSubsystem drive) {
    super(new Timeout(new PIDDrive(drive, AngleConstants.kQuarterTurn, 0,
        new DriveControl(drive::getRotation, drive::getForwardMeters)), AutoConstants.kPIDReverseTime));
  }
}