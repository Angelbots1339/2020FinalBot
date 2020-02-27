package frc.robot.commands.util;

public class ShootingProfiles {
    private double distance;
    private double shooterSpeed;
    private double hoodvalue;

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

	public void set(ShootingProfiles shootingProfiles) {
        distance = shootingProfiles.getDistance();
        shooterSpeed = shootingProfiles.getShooterSpeed();
        hoodvalue = shootingProfiles.getHoodvalue();
	}

}