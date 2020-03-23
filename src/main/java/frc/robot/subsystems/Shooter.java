package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.ShooterConstants;

/**
 * Shooter
 */
public class Shooter {
    ShooterSide m_leftShooter;
    ShooterSide m_rightShooter;

    public Shooter() {
        super();
        
        m_leftShooter = new ShooterSide(ShooterConstants.kLeftShooter, "Left Shooter", false);
        m_rightShooter = new ShooterSide(ShooterConstants.kRightShooter, "Right Shooter", true);
    }

    public ShooterSide getLeft() {
        return m_leftShooter;
    }

    public ShooterSide getRight() {
        return m_rightShooter;
    }

    public void setSetpoint(double shooterSpeed) {
        m_leftShooter.setSetpoint(shooterSpeed);
        m_rightShooter.setSetpoint(shooterSpeed);
    }

    public void enable() {
        m_leftShooter.enable();
        m_rightShooter.enable();
    }

    public void calculate() {
        m_leftShooter.getController().calculate(m_leftShooter.getMeasurement());
        m_rightShooter.getController().calculate(m_rightShooter.getMeasurement());
    }

    public void disable() {
        m_leftShooter.disable();
        m_rightShooter.disable();
    }

    public boolean atSetpoint() {
        return m_leftShooter.atSetpoint() && m_rightShooter.atSetpoint();
    }

    public Subsystem[] getRequirements() {
        return new Subsystem[]{m_leftShooter, m_rightShooter};
    }
}