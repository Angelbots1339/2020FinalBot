/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

//TODO
public class PIDDrive extends CommandBase {
    private final DriveSubsystem m_drive;
    private double m_startTime, m_currentTime, m_timeout;
    PIDController m_turnController, m_driveController;
    DoubleSupplier m_turnSupplier, m_driveSupplier;

    /**
     * Drives backwards and ends after 2 seconds
     */
    /**
     * 
     * @param drive
     * @param turnSetpoint
     * @param driveSetpoint
     * @param turnSupplier returns the current value of turn 
     * @param driveSupplier returns current drive value
     * @param timeout
     */
    
    public PIDDrive(DriveSubsystem drive, double turnSetpoint, double driveSetpoint, DoubleSupplier turnSupplier,
            DoubleSupplier driveSupplier, double timeout) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_drive = drive;
        addRequirements(m_drive);
        m_driveController = new PIDController(DriveConstants.kPDrive, DriveConstants.kIDrive, DriveConstants.kDDrive);
        m_turnController = new PIDController(DriveConstants.kPTurn, DriveConstants.kITurn, DriveConstants.kDTurn);
        m_turnController.setSetpoint(turnSetpoint);
        m_driveController.setSetpoint(driveSetpoint);
        m_turnSupplier = turnSupplier;
        m_driveSupplier = driveSupplier;
        m_timeout = timeout;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_drive.zeroHeading();
        m_startTime = Timer.getFPGATimestamp();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_drive.arcadeDrive(m_driveController.calculate(m_driveSupplier.getAsDouble()),
                m_turnController.calculate(m_turnSupplier.getAsDouble()));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_drive.arcadeDrive(0, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        m_currentTime = Timer.getFPGATimestamp();
        return m_currentTime - m_startTime >= m_timeout
                || (m_driveController.atSetpoint() && m_turnController.atSetpoint());
    }
}
