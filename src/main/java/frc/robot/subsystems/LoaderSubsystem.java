/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LoaderConstants;
import frc.robot.Constants.SensorConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LoaderSubsystem extends SubsystemBase {

  private PWMSparkMax m_loader;
  private DigitalInput m_bottomEmitter;
  private DigitalInput m_bottomReceiver;
  private DigitalInput m_middleEmitter;
  private DigitalInput m_middleReceiver;
  private DigitalInput m_topEmitter;
  private DigitalInput m_topReceiver;

  /**
   * Creates a new LoaderSubsystem.
   */
  public LoaderSubsystem() {
    m_loader = new PWMSparkMax(LoaderConstants.kLoaderMotor);

    m_topEmitter = new DigitalInput(SensorConstants.topEmitter);
    m_topReceiver = new DigitalInput(SensorConstants.topReciever);

    m_middleEmitter = new DigitalInput(SensorConstants.middleEmitter);
    m_middleReceiver = new DigitalInput(SensorConstants.middleReciever);

    m_topEmitter = new DigitalInput(SensorConstants.topEmitter);
    m_topReceiver = new DigitalInput(SensorConstants.topReciever);
  }

  public void enable(){
    m_loader.set(LoaderConstants.kMaxLoaderSpeed);
  }

  public void disable(){
    m_loader.set(0);
  }

  public void periodic() {
    SmartDashboard.putBoolean("Top Emitter", m_topEmitter.get());
    SmartDashboard.putBoolean("Top Reciever", m_topReceiver.get());
    SmartDashboard.putBoolean("Middle Emitter", m_middleEmitter.get());
    SmartDashboard.putBoolean("Middle Reciever", m_middleReceiver.get());
    SmartDashboard.putBoolean("Bottom Emitter", m_bottomEmitter.get());
    SmartDashboard.putBoolean("Bottom Reciever", m_bottomReceiver.get());
  

    // This method will be called once per scheduler run
  }

  public boolean isTopBeamBroken(){
    return!(m_topReceiver.get());
  }

  public boolean isMiddleBeamBroken(){
    return!(m_middleReceiver).get();
  }

  public boolean isBottomBeamBroken(){
    return!(m_bottomReceiver).get();
  }
}
