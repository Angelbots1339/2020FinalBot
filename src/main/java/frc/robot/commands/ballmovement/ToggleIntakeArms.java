/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.ballmovement;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeArmPID;

public class ToggleIntakeArms extends CommandBase {

  private final IntakeArmPID m_arm;

  /**
   * toggles the intake arms
   */
  public ToggleIntakeArms(IntakeArmPID arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_arm = arm;
    addRequirements(m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.toggleSetpoint();
    m_arm.enable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_arm.atSetpoint();
  }

}
