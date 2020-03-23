/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.utils.DriveControl;
import frc.robot.subsystems.DriveSubsystem;

//TODO
public class PIDDrive extends CommandBase {
    private final DriveSubsystem m_drive;
    PIDController m_turnController;
    PIDController m_driveController;
    DriveControl m_currentDrivePosition;

    /**
     * Drives backwards and ends after 2 seconds
     */
    /**
     * 
     * @param drive
     * @param turnSetpoint
     * @param driveSetpoint
     * @param turnSupplier  returns the current value of turn
     * @param driveSupplier returns current drive value
     * @param timeout
     */

    public PIDDrive(DriveSubsystem drive, double turnSetpoint, double driveSetpoint,
            DriveControl currentDrivePosition) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_drive = drive;
        addRequirements(m_drive);
        m_driveController = new PIDController(DriveConstants.kPDrive, DriveConstants.kIDrive, DriveConstants.kDDrive);
        m_turnController = new PIDController(DriveConstants.kPTurn, DriveConstants.kITurn, DriveConstants.kDTurn);
        m_turnController.setSetpoint(turnSetpoint);
        m_driveController.setSetpoint(driveSetpoint);
        m_currentDrivePosition = currentDrivePosition;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_drive.zeroHeading();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_drive.arcadeDrive(m_driveController.calculate(m_currentDrivePosition.getDrive()),
                m_turnController.calculate(m_currentDrivePosition.getTurn()));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_drive.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_driveController.atSetpoint() && m_turnController.atSetpoint();
    }
}
