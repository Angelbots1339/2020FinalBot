/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.ballmovement;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.vision.ShootingProfile;
import frc.robot.subsystems.Shooter;

public class RunShooter extends CommandBase {

  private final Shooter m_shooter;
  private ShootingProfile m_targetProfile;

  /**
   * Creats a set speed for the left and right motors for the shooter to fire.
   */
  public RunShooter(Shooter shooter, ShootingProfile targetProfile) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    addRequirements(m_shooter.getLeft());
    addRequirements(m_shooter.getRight());

    m_targetProfile = targetProfile;
  }

  /**
   * Creates a new RunShooter.
   */
  public RunShooter(Shooter shooter, double targetSpeed) {
    this(shooter, new ShootingProfile(-1, targetSpeed, -1, 0, 0, 0));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.setSetpoint(m_targetProfile.getShooterSpeed());
    m_shooter.enable();
    m_shooter.calculate();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.disable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
