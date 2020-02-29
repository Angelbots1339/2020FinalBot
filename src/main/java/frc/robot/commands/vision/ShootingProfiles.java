package frc.robot.commands.vision;

import java.util.regex.Pattern;

public class ShootingProfiles {
    private double distance;
    private double shooterSpeed;
    private double hoodvalue;
    private double angleP;
    private double loadingSpeed;

    public ShootingProfiles(double distance, double shooterSpeed, double hoodvalue, double angleP,
            double loadingSpeed) {
        this.distance = distance;
        this.shooterSpeed = shooterSpeed;
        this.hoodvalue = hoodvalue;
        this.angleP = angleP;
        this.loadingSpeed = loadingSpeed;
    }

    public ShootingProfiles(String info) {
        this(getProperty(info, "m"), getProperty(info, "rpm"), getProperty(info, "hr"), getProperty(info, "aP"),
                getProperty(info, "LS"));
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

    public double getLoadingSpeed() {
        return loadingSpeed;
    }

    public void set(ShootingProfiles shootingProfiles) {
        distance = shootingProfiles.getDistance();
        shooterSpeed = shootingProfiles.getShooterSpeed();
        hoodvalue = shootingProfiles.getHoodvalue();
        angleP = shootingProfiles.getAngleP();
        loadingSpeed = shootingProfiles.getLoadingSpeed();
    }

    @Override
    public String toString() {
        return "ShootingProfiles [angleP=" + angleP + ", distance=" + distance + ", hoodvalue=" + hoodvalue
                + ", loadingSpeed=" + loadingSpeed + ", shooterSpeed=" + shooterSpeed + "]";
    }

}