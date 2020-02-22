/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ServoTest extends SubsystemBase {

  private Servo leftServo;
  private Servo rightServo;
  /**
   * Creates a new ServoTest.
   */
  public ServoTest() {
    leftServo = new Servo(ClimberConstants.kLeftServo);
    rightServo = new Servo(ClimberConstants.kRightServo);
  }

  public void setBothAngle(double degrees) {
    leftServo.setAngle(degrees);
    rightServo.setAngle(degrees);
  }

  public double getLeftAngle(){
    return leftServo.getAngle();
  }

  public double getRightAngle(){
    return rightServo.getAngle();
  }




  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.putNumber("Left angle:", getLeftAngle());
    //SmartDashboard.putNumber("Right angle:", getRightAngle());
  }
}