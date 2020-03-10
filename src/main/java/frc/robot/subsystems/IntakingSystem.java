package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.LoaderConstants;
import frc.robot.Constants.SensorConstants;

/**
 * IntakingSystem
 */
public class IntakingSystem {
    private final IndexerSubsystem m_indexer;
    private final IntakeSubsystem m_intake;
    private final LoaderPIDSubsystem m_loader;

    private DigitalInput m_shooterEmitter;
    private DigitalInput m_shooterReceiver;
    private DigitalInput m_bottomRightEmitter;
    private DigitalInput m_bottomRightReceiver;

    private DigitalInput m_middleEmitter;
    private DigitalInput m_middleReceiver;
    private DigitalInput m_topEmitter;
    private DigitalInput m_topReceiver;

    private int m_count = 0;

    public IntakingSystem(IndexerSubsystem indexer, IntakeSubsystem intake, LoaderPIDSubsystem loader, Shooter shooter) {
        this.m_indexer = indexer;
        this.m_intake = intake;
        this.m_loader = loader;

        m_topEmitter = new DigitalInput(SensorConstants.topEmitter);
        m_topReceiver = new DigitalInput(SensorConstants.topReciever);

        m_middleEmitter = new DigitalInput(SensorConstants.middleEmitter);
        m_middleReceiver = new DigitalInput(SensorConstants.middleReciever);

        m_shooterEmitter = new DigitalInput(SensorConstants.ShooterEmitter);
        m_shooterReceiver = new DigitalInput(SensorConstants.ShooterReciever);
        m_bottomRightEmitter = new DigitalInput(SensorConstants.bottomEmitter);
        m_bottomRightReceiver = new DigitalInput(SensorConstants.bottomReciever);

        Trigger bottomTrigger = new Trigger(() -> isBottomBroken());
        bottomTrigger.and(new Trigger(() -> loader.get() >= 0)).whenActive(() -> m_count++);

        Trigger releaseBalls = bottomTrigger.and(new Trigger(() -> loader.get() < 0));
        releaseBalls.whenActive(() -> m_count--);
        // releaseBalls.and(new Trigger(() -> getCount() >= 5)).whenInactive(() ->
        // m_count--);

        Trigger topTrigger = new Trigger(() -> isTopBeamBroken());
        topTrigger.and(new Trigger(() -> shooter.atSetpoint())).whenActive(() -> m_count--);
        // releaseBalls.and(new Trigger(() -> getCount() >= 5)).whenInactive(() ->
        // m_count--);
    }

    public IndexerSubsystem getIndexer() {
        return m_indexer;
    }

    public IntakeSubsystem getIntake() {
        return m_intake;
    }

    public LoaderPIDSubsystem getLoader() {
        return m_loader;
    }

    public int getCount() {
        return m_count;
    }

    public boolean isShooterBeamBroken() {
        return !(m_shooterReceiver.get());
    }

    public boolean isTopBeamBroken() {
        return !m_topReceiver.get();
    }

    public boolean isMiddleBeamBroken() {
        return !m_middleReceiver.get();
    }

    public boolean isBottomBroken() {
        return !m_bottomRightReceiver.get();
    }

    public void enable(double indexerSpeed, double loaderSpeed) {
        m_intake.enableIntake();
        m_indexer.enable(indexerSpeed);
        m_loader.setSetpoint(loaderSpeed);
        m_loader.enable();
    }

    public void disable() {
        m_intake.disableIntake();
        m_indexer.disable();
        m_loader.disable();
    }

    public void telemetry() {
        SmartDashboard.putBoolean("Top Emitter", m_topEmitter.get());
        SmartDashboard.putBoolean("Top Reciever", m_topReceiver.get());
        SmartDashboard.putBoolean("Middle Emitter", m_middleEmitter.get());
        SmartDashboard.putBoolean("Middle Reciever", m_middleReceiver.get());

        SmartDashboard.putBoolean("Bottom Emitter", m_bottomRightEmitter.get());
        SmartDashboard.putBoolean("Bottom Reciever", m_bottomRightReceiver.get());
        SmartDashboard.putBoolean("Shooter Emitter", m_shooterEmitter.get());
        SmartDashboard.putBoolean("Shooter Reciever", m_shooterReceiver.get());

        SmartDashboard.putNumber("# of balls", getCount());
    }

    public void reverse() {
        m_intake.reverseIntake();
        m_indexer.reverse();
        m_loader.reverse(LoaderConstants.kFullSpeed);
    }

    public void disableIntake() {
      m_intake.disableIntake();
    }
  
    public void reverseIntake() {
      m_intake.reverseIntake();
    }
}