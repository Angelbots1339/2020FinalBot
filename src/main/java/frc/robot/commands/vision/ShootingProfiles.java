package frc.robot.commands.vision;

import java.util.regex.Pattern;

public class ShootingProfiles {
    private double distance;
    private double shooterSpeed;
    private double hoodvalue;

    public ShootingProfiles(double distance, double shooterSpeed, double hoodvalue) {
        this.distance = distance;
        this.shooterSpeed = shooterSpeed;
        this.hoodvalue = hoodvalue;
    }

    public ShootingProfiles(String info) {
        this(getProperty(info, "m"), getProperty(info, "rpm"), getProperty(info, "hr"));
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

    public void set(ShootingProfiles shootingProfiles) {
        distance = shootingProfiles.getDistance();
        shooterSpeed = shootingProfiles.getShooterSpeed();
        hoodvalue = shootingProfiles.getHoodvalue();
    }

}