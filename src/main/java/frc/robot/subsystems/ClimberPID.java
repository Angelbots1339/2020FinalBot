/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.ClimberConstants;

// TODO needs lots of work !!!

public class ClimberPID extends PIDSubsystem {
  /**
   * Controls PID using distance
   */

  public ClimberPID(PIDController controller){
    super(controller);
  }

   // sets up the motors
   private CANSparkMax m_leftClimberMotor = new CANSparkMax(ClimberConstants.kLeftClimberMotor, MotorType.kBrushless);
   private CANSparkMax m_rightClimberMotor = new CANSparkMax(ClimberConstants.kRightClimberMotor, MotorType.kBrushless);
   
   // sets up the encoders
   private Encoder m_leftClimbEncoder = new Encoder(ClimberConstants.kLeftEncoder[0], ClimberConstants.kLeftEncoder[1], ClimberConstants.kLeftEncoderReversed);
   private Encoder m_rightClimbEncoder = new Encoder(ClimberConstants.kRightEncoder[0], ClimberConstants.kRightEncoder[1], ClimberConstants.krightEncoderReversed);

   // sets up the feet forward
   private SimpleMotorFeedforward m_climberFeedForward = new SimpleMotorFeedforward(ClimberConstants.kSVolts, ClimberConstants.kSVoltsPerSecondsPerRotation);

   // sets up the PID Controller
   private PIDController pidUp = new PIDController(ClimberConstants.kUpP, ClimberConstants.kUpI, ClimberConstants.kUpD);
   private PIDController pidDown = new PIDController(ClimberConstants.kDownP, ClimberConstants.kDownI, ClimberConstants.kDownD);
   ClimberPID(){
    super(
        // The PIDController used by the subsystem
        new PIDController(ClimberConstants.kUpP, ClimberConstants.kUpI, ClimberConstants.kUpD));
        getController().setTolerance(ClimberConstants.kClimberToleranceRPS);
        m_leftClimbEncoder.setDistancePerPulse(ClimberConstants.kEncoderDistancePerPulse);
        m_rightClimbEncoder.setDistancePerPulse(ClimberConstants.kEncoderDistancePerPulse);
        setSetpoint(ClimberConstants.kClimberTargetRPS);
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
    m_leftClimberMotor.setVoltage(output + m_climberFeedForward.calculate(setpoint));
    m_rightClimberMotor.setVoltage(output + m_climberFeedForward.calculate(setpoint));
  }

  public double getLeftMeasurement() {
    // Return the process variable measurement here
    return m_leftClimbEncoder.getRate();
  }

  public double getRightMeasurment(){
    return m_rightClimbEncoder.getRate();
  }

  public boolean atSetpoint(){
    return m_controller.atSetpoint();
  }

  public void runClimber(){
    m_leftClimberMotor.set(ClimberConstants.kClimberSpeed);
    m_rightClimberMotor.set(ClimberConstants.kClimberSpeed);
  }

  @Override // created two more methods that get the left and the right
  protected double getMeasurement() {
    // TODO Auto-generated method stub
    return 0;
  }
}
