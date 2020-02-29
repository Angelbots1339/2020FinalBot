/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.ballmovement.RunIntakeArms;
import frc.robot.subsystems.IntakeArmPID;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html

public class Auto extends ParallelCommandGroup {
  /**
   * Creates a new Auto.
   */
  private IntakeArmPID m_rightArm;
  private IntakeArmPID m_leftArm;

  public Auto(IntakeArmPID rightArm, IntakeArmPID leftArm) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());super();
    m_rightArm = rightArm;
    m_leftArm = leftArm;

    addCommands(new RunIntakeArms(m_rightArm, m_leftArm));
  }
}
