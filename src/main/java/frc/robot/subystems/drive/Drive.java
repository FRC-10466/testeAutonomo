package frc.robot.subystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subystems.drive.DriveIO.DriveIOInputs;
import frc.robot.subystems.drive.GyroIO.GyroIOInputs;

public class Drive extends SubsystemBase{

    private final DriveIO drive;
    private final GyroIO gyro;

    private final DriveIOInputs inputs = new DriveIO.DriveIOInputs();
    private final GyroIOInputs gyroinputs = new GyroIO.GyroIOInputs();

    private final StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault().getStructTopic("Odometria/Pose", Pose2d.struct).publish();
    private final DifferentialDriveOdometry odometria;
    private Field2d campo;

    private double potenciaMaxima = DriveConstants.VELOCIDADE_TURBO;

    public Drive(DriveIO drive, GyroIO gyro) {
        this.drive = drive;
        this.gyro = gyro;

        drive.updateInputs(inputs);
        gyro.updateInputs(gyroinputs);
        
        odometria = new DifferentialDriveOdometry(getAngulo(), inputs.positionLeft, inputs.positonRight);
        SmartDashboard.putData("Campo", campo);
    }

    @Override
    public void periodic() {
        drive.updateInputs(inputs);
        gyro.updateInputs(gyroinputs);

        odometria.update(getAngulo(), inputs.positionLeft, inputs.positionLeft);
        posePublisher.set(getPose());
        campo.setRobotPose(getPose());

        SmartDashboard.putNumber("encoder direito", inputs.positonRight);
        SmartDashboard.putNumber("encoder esquedo", inputs.positionLeft);
        SmartDashboard.putNumber("Angulo", getAngulo().getDegrees());
        SmartDashboard.putNumber("Graus/seg", getVelocityGyro());
        SmartDashboard.putNumber("Pose X",getPose().getX());
        SmartDashboard.putNumber("Pose Y", getPose().getY());
    }
    
    public void setTurbo(boolean ligado) {
        potenciaMaxima = ligado ? DriveConstants.VELOCIDADE_TURBO : DriveConstants.VELOCIDADE_NORMAL;
    }

    public Command commandTurbo() {
        return Commands.runEnd(
            () -> setTurbo(true),
            () -> setTurbo(false) );
    }

    public void setVolts(double left, double Right) {
        drive.setVolts(left, Right);
    }

    public void arcade(double straight,double rotation) {
        var lados = DifferentialDrive.arcadeDriveIK(straight,rotation,true);
        drive.setVolts(lados.left * potenciaMaxima * DriveConstants.COMPENSATION_VOLTAGE, lados.right * potenciaMaxima * DriveConstants.COMPENSATION_VOLTAGE);
    }

    public double mediaAtual() {
        return (inputs.positionLeft + inputs.positonRight) / 2;
    }

    public void parar(){
        drive.setVolts(0, 0);
    }

    public Rotation2d getAngulo() {
        return gyroinputs.angulo;
    }

    public double getVelocityGyro() {
        return gyroinputs.velocityGrausPorSeg;
    }

    public void zerarAngulo() {
        gyro.zerar();
    }

    public Pose2d getPose() {
        return odometria.getPoseMeters();
    }

    public void resetarPose(Pose2d pose) {
        odometria.resetPosition(getAngulo(),inputs.positionLeft, inputs.positonRight,pose);
    }

    public double velocidadeMedia() {
        return (inputs.velocityLeft + inputs.velocityRight) / 2.0;
    }

}