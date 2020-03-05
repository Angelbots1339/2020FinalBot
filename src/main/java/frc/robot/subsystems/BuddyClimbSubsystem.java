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
import frc.robot.Constants.ClimberConstants;
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
    leftServo = new Servo(ClimberConstants.kLeftServo);
    rightServo = new Servo(ClimberConstants.kRightServo);
    m_climber = climber;
  }

  public void setBothAngle(double degrees) {
    leftServo.setAngle(degrees);
    rightServo.setAngle(degrees);
  }

  public double getLeftAngle() {
    return leftServo.get();
  }

  public double getRightAngle() {
    return rightServo.get();
  }

  public void engage() {
    rightServo.setAngle(140);
    leftServo.setAngle(10.0);
  }

  public void disengage() {
    if (m_climber.isEnabled()) {
      rightServo.setAngle(40);
      leftServo.setAngle(110.0);
    }
  }

  // Right Servo is correct
  public void toggle() {
    if (getEngageToggled()) { // if false (engaged) set to unlocked position
      rightServo.setAngle(40);
      leftServo.setAngle(100);
      toggleEngage();
    } else { // else sets to locked
      rightServo.setAngle(100);
      leftServo.setAngle(40.0);
      toggleEngage();
    }
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
