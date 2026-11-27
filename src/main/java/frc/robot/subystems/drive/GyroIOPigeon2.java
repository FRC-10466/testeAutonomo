package frc.robot.subystems.drive;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;

public class GyroIOPigeon2 implements GyroIO{

    private final Pigeon2 pigeon = new Pigeon2(DriveConstants.ID_PIGEON);

    public GyroIOPigeon2() {
        Pigeon2Configuration config = new Pigeon2Configuration();
        config.MountPose.MountPoseYaw = DriveConstants.MOUNTPOSE_YAW;
        config.MountPose.MountPosePitch = DriveConstants.MOUNTPOSEPITCH;
        config.MountPose.MountPoseRoll = DriveConstants.MOUNTPOSE_ROLL;
        pigeon.getConfigurator().apply(config);
        pigeon.setYaw(0);
    }

    public Rotation2d getAngulo() {
        return Rotation2d.fromDegrees(pigeon.getYaw().getValueAsDouble());
    }

    public double getVelocityGyro() {
        return pigeon.getAngularVelocityZWorld().getValueAsDouble();
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        var yaw = pigeon.getYaw();
        var velocidade = pigeon.getAngularVelocityZWorld();

        inputs.conectado = yaw.getStatus() == StatusCode.OK;
        inputs.angulo = Rotation2d.fromDegrees(yaw.getValueAsDouble());
        inputs.velocityGrausPorSeg = velocidade.getValueAsDouble();
    }

    @Override
    public void zerar() {
        pigeon.setYaw(0);
    }
}
