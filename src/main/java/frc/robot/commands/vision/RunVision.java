package frc.robot.commands.vision;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.regex.Pattern;
import java.util.stream.Collectors;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ballmovement.HoodPID;
import frc.robot.commands.ballmovement.RunShooter;
import frc.robot.commands.ballmovement.ShootAllBalls;
import frc.robot.commands.util.ShootingProfiles;
import frc.robot.subsystems.AdjustableHoodSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HoodPIDSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.LoaderSubsystem;
import frc.robot.subsystems.ShooterPID;
import frc.robot.subsystems.LimelightSubsystem.LedMode;

/**
 * RunVision
 */
public class RunVision extends ParallelCommandGroup {
    private static ArrayList<ShootingProfiles> data = getData();
    LimelightSubsystem m_limeLight;
    DriveSubsystem m_drive;
    ShooterPID m_leftShooter;
    ShooterPID m_rightShooter;
    IntakeSubsystem m_intake;
    IndexerSubsystem m_indexer;
    LoaderSubsystem m_loader;
    HoodPIDSubsystem m_hood;
    ShootingProfiles m_targetProfile = new ShootingProfiles(-1, -1, -1);

    // add something that sets a default hood and speed once it no longer sees it

    public RunVision(LimelightSubsystem limeLight, DriveSubsystem drive, ShooterPID leftShooter,
            ShooterPID rightShooter, IntakeSubsystem intake, IndexerSubsystem indexer,
             LoaderSubsystem loader, HoodPIDSubsystem hood) {
        m_limeLight = limeLight;
        m_drive = drive;
        m_leftShooter = leftShooter;
        m_rightShooter = rightShooter;
        m_intake = intake;
        m_indexer = indexer;
        m_loader = loader;
        m_hood = hood;

        addCommands(new RunShooter(m_leftShooter, m_rightShooter, m_targetProfile),
                    new RunHood(m_hood, m_targetProfile),
                    new SequentialCommandGroup(new CameraAlign(m_drive, m_limeLight, m_targetProfile),
                        new ShootAllBalls(m_intake, m_indexer, m_loader, m_leftShooter, m_rightShooter)));
    }

    @Override
    public void initialize() {
        //m_limeLight.setLed(LedMode.PIPELINE);
        double currentDist = m_limeLight.getDistanceToVisionTarget();
        SmartDashboard.putNumber("current Dist", currentDist);
        /**
         * compares the profiles and current distance to find the correct profile(text
         * files) to use
         */
        m_targetProfile
                .set(data.stream()
                        .collect(Collectors.minBy((a, b) -> (int) Math.signum(
                                Math.abs(a.getDistance() - currentDist) - Math.abs(b.getDistance() - currentDist))))
                        .get());
        super.initialize();
    }

    public static ArrayList<ShootingProfiles> getData() {
        var profilesArr = new ArrayList<ShootingProfiles>();
        try {
            var br = new BufferedReader( // type of reader to read text file
                    new FileReader(Filesystem.getDeployDirectory().getCanonicalPath() + File.separator
                            + "shooterProfiles.data")); // getting
                                                        // path
                                                        // to
                                                        // text
                                                        // file
            String line;
            while ((line = br.readLine()) != null)
                profilesArr.add(new ShootingProfiles(getProperty(line, "m"), getProperty(line, "rpm"),
                        getProperty(line, "hr"))); // hr = hoodRotations
            br.close(); // stops the reader
        }
        // if file is not found, stack trace is printed
        catch (FileNotFoundException e) {
            e.printStackTrace(); // list of methods that happened up to error
        }
        // fail to read file
        catch (IOException e) {
            e.printStackTrace();
        }
        return profilesArr;
    }

    @Override
    public void end(boolean interrupted) {
        //m_limeLight.setLed(LedMode.OFF);
        super.end(interrupted);        
    }


    /**
     * uses regxr to look through the text file
     * 
     * @param str Line of text in text file
     * @param key Leters behind numbers in str
     * @return the number before the key
     */
    private static double getProperty(String str, String key) {
        return Double.parseDouble(str.replaceAll(".*?(\\d+(\\.\\d*)?)" + Pattern.quote(key) + ".*", "$1"));
    }

}