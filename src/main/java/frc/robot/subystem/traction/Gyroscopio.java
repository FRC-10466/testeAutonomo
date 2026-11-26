package frc.robot.subystem.traction;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import edu.wpi.first.math.geometry.Rotation2d;

public class Gyroscopio {

    Pigeon2 pigeon = new Pigeon2(DriveConstants.ID_PIGEON);

    public Gyroscopio() {
        Pigeon2Configuration config = new Pigeon2Configuration();
        config.MountPose.MountPoseYaw = DriveConstants.MOUNTPOSE_YAW;
        config.MountPose.MountPoseRoll = DriveConstants.MOUNTPOSE_ROLL;
        config.MountPose.MountPosePitch = DriveConstants.MOUNTPOSEPITCH;
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
