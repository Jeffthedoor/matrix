package frc.robot.lib.swervelib.ctre;

public class ThriftyConfig {
    private final int id;
    private final double offset;

    public ThriftyConfig(int id, double offset) {
        this.id = id;
        this.offset = offset;
    }

    public int getId() {
        return id;
    }

    public double getOffset() {
        return offset;
    }
}
