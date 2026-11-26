package frc.robot.subystem.traction;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import edu.wpi.first.math.geometry.Rotation2d;

public class Gyroscopio {

    Pigeon2 pigeon = new Pigeon2(22);

    public Gyroscopio() {
        Pigeon2Configuration config = new Pigeon2Configuration();
        config.MountPose.MountPoseYaw = -90.61016845703125;
        config.MountPose.MountPoseRoll = -0.3616531789302826;
        config.MountPose.MountPosePitch = -0.9369150996208191;
        pigeon.getConfigurator().apply(config);
        pigeon.setYaw(0);
    }

    public Rotation2d getAngulo() {
        return Rotation2d.fromDegrees(pigeon.getYaw().getValueAsDouble());
    }

    public double getVelocityGyro() {
        return pigeon.getAngularVelocityZWorld().getValueAsDouble();
    }

    public void zerar() {
        pigeon.setYaw(0);
    }
}
