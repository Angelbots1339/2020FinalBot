/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.ballmovement;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeArmPID2;
/**
 * currently testing this one
 */

public class RunIntakeArms2 extends CommandBase {

  private final IntakeArmPID2 m_rightArm;
  private final double m_setpoint;
  

  /**
   * Creates a new RunIntakeArms.
   */
  public RunIntakeArms2(IntakeArmPID2 motor, double setpoint){
    // Use addRequirements() here to declare subsystem dependencies.
    m_rightArm = motor;
    addRequirements(m_rightArm);

    m_setpoint = setpoint;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_rightArm.setSetpoint(m_setpoint);
    m_rightArm.enable();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_rightArm.disable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
