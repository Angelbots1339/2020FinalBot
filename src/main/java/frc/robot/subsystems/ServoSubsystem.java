/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;


public class ServoSubsystem extends SubsystemBase {
  /**
   * Creates a new ServoSubsystem.
   */

  private Servo leftServo;
  private Servo rightServo;
  private boolean engageToggle;

  public ServoSubsystem() {
    leftServo = new Servo(ClimberConstants.kLeftServo);
    rightServo = new Servo(ClimberConstants.kRightServo);
  }

  public void setBothAngle(double degrees) {
    leftServo.setAngle(degrees);
    rightServo.setAngle(degrees);
   // rightServo.set(0.0);//need to find setpoints
   // leftServo.set(0.0);
  }

  public double getLeftAngle(){
    return leftServo.get();
  }

  public double getRightAngle(){
    return rightServo.get();
  }

  public void engage() {
    if(getEngageToggled()) { // if false (engaged) set to unlocked position
      rightServo.setAngle(0.0);//TODO this ain't final someone check this
      leftServo.setAngle(0.0);//TODO this ain't final someone check this
      toggleEngage();
    } else { // else sets to locked
      rightServo.setAngle(90.0);//TODO this ain't final someone check this
      leftServo.setAngle(90.0);//TODO this ain't final someone check this
      toggleEngage();
    }
  }

  public void toggleEngage() {
    engageToggle = !engageToggle;
  }

  public boolean getEngageToggled() {
    return engageToggle;
  }




  /*public void crankDatSouljaBoy() {
    if(refreshCount < 50) {
      rightServo.set(1.0);
      leftServo.set(1.0);
    } else {
      refreshCount++;
      rightServo.set(0.0);
      leftServo.set(0.0);
    }
  }
*/

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //crankDatSouljaBoy();

    //SmartDashboard.putNumber("LeftPos", leftServo.get());
    //SmartDashboard.putNumber("RightPos", rightServo.get());
  }
}
