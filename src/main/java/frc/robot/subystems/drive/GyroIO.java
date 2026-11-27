package frc.robot.subystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;

public interface GyroIO {

    class GyroIOInputs {
        public boolean conectado = false;
        public Rotation2d angulo = Rotation2d.kZero;
        public double velocityGrausPorSeg = 0.0;
    }

    default void updateInputs(GyroIOInputs inputs) {}

    default void zerar() {}
}
