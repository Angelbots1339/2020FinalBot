/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.ballmovement;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterPID;

public class RunShooterVolt extends CommandBase {
  private final ShooterPID m_leftPID;
  private final ShooterPID m_rightPID;
  private double m_volt;
  /**
   * Creates a new RunShooter.
   */
  public RunShooterVolt(ShooterPID leftPID, ShooterPID rightPID, double volts) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_leftPID = leftPID;
    addRequirements(m_leftPID);

    m_rightPID = rightPID;
    addRequirements(m_rightPID);

    m_volt = volts;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_leftPID.setMotorVoltage(m_volt);
    m_rightPID.setMotorVoltage(m_volt);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_leftPID.disable();
    m_rightPID.disable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
