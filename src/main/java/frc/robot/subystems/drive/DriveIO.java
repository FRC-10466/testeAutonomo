package frc.robot.subystems.drive;

public interface DriveIO {

    class DriveIOInputs {
        public double positionLeft = 0;
        public double positonRight = 0;
        
        public double voltsRight = 0;
        public double voltsLeft = 0;

        public double velocityRight = 0;
        public double velocityLeft = 0;
    }

    default void updateInputs(DriveIOInputs inputs) {}

    default void setVolts(double voltsLeft,double voltsRight) {}

    default void periodic() {}
}
