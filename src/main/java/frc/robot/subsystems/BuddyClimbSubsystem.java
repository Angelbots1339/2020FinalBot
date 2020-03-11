/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BuddyClimbConstants;
import frc.robot.Constants.DashboardConstants;

public class BuddyClimbSubsystem extends SubsystemBase {
  /**
   * Creates a new ServoSubsystem.
   */

  private Servo leftServo;
  private Servo rightServo;
  private boolean engageToggle = true;
  ClimberSubsystem m_climber;

  public BuddyClimbSubsystem(ClimberSubsystem climber) {
    leftServo = new Servo(BuddyClimbConstants.kLeftServo);
    rightServo = new Servo(BuddyClimbConstants.kRightServo);
    m_climber = climber;
  }

  public double getLeftAngle() {
    return leftServo.get();
  }

  public double getRightAngle() {
    return rightServo.get();
  }

  public void engage() {
    rightServo.setAngle(BuddyClimbConstants.kRightEngagePos);
    leftServo.setAngle(BuddyClimbConstants.kLeftEngagePos);
  }

  public void disengage() {
    if (m_climber.isEnabled()) {
      rightServo.setAngle(BuddyClimbConstants.kRightDisengagePos);
      leftServo.setAngle(BuddyClimbConstants.kLeftDisengagePos);
    }
  }

  // Right Servo is correct
  public void toggle() {
    if (getEngageToggled())
      disengage();
    else
      engage();

    toggleEngage();
  }

  public void toggleEngage() {
    engageToggle = !engageToggle;
  }

  public boolean getEngageToggled() {
    return engageToggle;
  }

  @Override
  public void periodic() {

    if (DashboardConstants.kBuddyClimbTelemetry) {
      SmartDashboard.putNumber("L Servo Pos", leftServo.get());
      SmartDashboard.putNumber("R Servo Pos", rightServo.get());
    }
  }
}
