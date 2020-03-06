package frc.robot.commands.vision;

import java.util.regex.Pattern;

public class ShootingProfile {
    private double distance;
    private double shooterSpeed;
    private double hoodvalue;
    private double angleP;
    private double indexerSpeed;
    private double loaderSpeed;

    public ShootingProfile(double distance, double shooterSpeed, double hoodvalue, double angleP, double indexerSpeed,
            double loaderSpeed) {
        this.distance = distance;
        this.shooterSpeed = shooterSpeed;
        this.hoodvalue = hoodvalue;
        this.angleP = angleP;
        this.indexerSpeed = indexerSpeed;
        this.loaderSpeed = loaderSpeed;
    }

    /**
     * pulls data from shooting profiles and sets speed
     */

    public ShootingProfile(String info) {
        this(getProperty(info, "m"), getProperty(info, "rpm"), getProperty(info, "hr"), getProperty(info, "aP"),
                getProperty(info, "IS"), getProperty(info, "LS"));
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

    public double getDistance() {
        return distance;
    }

    public double getShooterSpeed() {
        return shooterSpeed;
    }

    public double getHoodvalue() {
        return hoodvalue;
    }

    public double getAngleP() {
        return angleP;
    }

    public double getIndexerSpeed() {
        return indexerSpeed;
    }

    public void set(ShootingProfile shootingProfiles) {
        distance = shootingProfiles.getDistance();
        shooterSpeed = shootingProfiles.getShooterSpeed();
        hoodvalue = shootingProfiles.getHoodvalue();
        angleP = shootingProfiles.getAngleP();
        indexerSpeed = shootingProfiles.getIndexerSpeed();
        loaderSpeed = shootingProfiles.getLoaderSpeed();
    }

    public double getLoaderSpeed() {
        return loaderSpeed;
    }

    @Override
    public String toString() {
        return "ShootingProfiles [angleP=" + angleP + ", distance=" + distance + ", hoodvalue=" + hoodvalue
                + ", indexerSpeed=" + indexerSpeed + ", loaderSpeed=" + loaderSpeed + ", shooterSpeed=" + shooterSpeed
                + "]";
    }

}