package frc.robot.commands.utils;

import java.util.function.DoubleSupplier;

/**
 * DriveControl
 */
public class DriveControl {
    private final DoubleSupplier driveSupplier;
    private final DoubleSupplier turnSupplier;
    public static final DriveControl empty = new DriveControl(() -> 0, () -> 0);

    public DriveControl(DoubleSupplier driveSupplier, DoubleSupplier turnSupplier) {
        this.driveSupplier = driveSupplier;
        this.turnSupplier = turnSupplier;
    }
    
    public double getDrive() {
        return driveSupplier.getAsDouble();
    }

    public double getTurn() {
        return turnSupplier.getAsDouble();
    }
}