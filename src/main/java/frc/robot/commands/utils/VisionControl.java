package frc.robot.commands.utils;

import java.util.function.BooleanSupplier;

/**
 * VisionControl
 */
public class VisionControl {
    private final BooleanSupplier alignSupplier;
    private final BooleanSupplier shootSupplier;
    public static final VisionControl shootOnly = new VisionControl(false, true);

    public VisionControl(BooleanSupplier alignSupplier, BooleanSupplier shootSupplier) {
        this.alignSupplier = alignSupplier;
        this.shootSupplier = shootSupplier;
    }

    public VisionControl(boolean isAligning, BooleanSupplier shootSupplier) {
        this(() -> isAligning, shootSupplier);
    }

    public VisionControl(BooleanSupplier driveSupplier, boolean isShooting) {
        this(driveSupplier, () -> isShooting);
    }

    public VisionControl(boolean isAligning, boolean isShooting) {
        this(() -> isAligning, () -> isShooting);
    }
    
    public boolean isAligning() {
        return alignSupplier.getAsBoolean();
    }

    public boolean isShooting() {
        return shootSupplier.getAsBoolean();
    }
}