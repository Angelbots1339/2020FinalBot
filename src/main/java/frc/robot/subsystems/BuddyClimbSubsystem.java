/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
 // uses a servo power module
 //https://docs.wpilib.org/en/latest/docs/software/actuators/servos.html
import frc.robot.Constants.BuddyClimbConstants;

public class BuddyClimbSubsystem extends SubsystemBase {

  private Servo m_servo;
  /**
   * Creates a new BuddyClimbSubsystem.
   */
  public BuddyClimbSubsystem() {
    m_servo = new Servo(BuddyClimbConstants.kServoPort);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void DropClimber(){
    // TODO check the angle 
    m_servo.set(BuddyClimbConstants.kServoPort);
  }
}
