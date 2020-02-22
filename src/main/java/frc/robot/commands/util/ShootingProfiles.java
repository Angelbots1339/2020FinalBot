package frc.robot.commands.util;

public class ShootingProfiles {
    private final double distance;
    private final double shooterSpeed;
    private final double hoodvalue;

    public ShootingProfiles(double distance, double shooterSpeed, double hoodvalue) {
        this.distance = distance;
        this.shooterSpeed = shooterSpeed;
        this.hoodvalue = hoodvalue;
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

}