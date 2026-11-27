package frc.robot.subystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.PersistMode;

public class Drive extends SubsystemBase{

    private final StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault().getStructTopic("Odometria/Pose", Pose2d.struct).publish();
    private final GyroIO gyroscopio = new GyroIO();
    private final Field2d campo = new Field2d();
    private DifferentialDriveOdometry odometria;

    private final SparkMax Rightfront = new SparkMax(DriveConstants.ID_RIGHT_FRONT,MotorType.kBrushed);
    public final SparkMax Rightback = new SparkMax(DriveConstants.ID_RIGHT_BACK,MotorType.kBrushed);
    
    private final SparkMax Leftfront = new SparkMax(DriveConstants.ID_LEFT_FRONT, MotorType.kBrushed);
    public final SparkMax Leftback = new SparkMax(DriveConstants.ID_LEFT_BACK, MotorType.kBrushed);

    public final DifferentialDrive drive = new DifferentialDrive(Leftfront, Rightfront);

    RelativeEncoder encoderRightFront = Rightfront.getEncoder();
    RelativeEncoder encoderLeftFront = Leftfront.getEncoder();
    
    public Drive() {
        SparkMaxConfig RightConfig = new SparkMaxConfig ();
        SparkMaxConfig LeftConfig = new SparkMaxConfig();

        SparkMaxConfig RightBackConfig = new SparkMaxConfig ();
        RightBackConfig.follow(Rightfront);
        RightBackConfig.idleMode(IdleMode.kBrake).voltageCompensation(DriveConstants.COMPENSATION_VOLTAGE);
        Rightback.configure(RightBackConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig LeftBackConfig = new SparkMaxConfig ();
        LeftBackConfig.follow(Leftfront);
        LeftBackConfig.idleMode(IdleMode.kBrake).voltageCompensation(DriveConstants.COMPENSATION_VOLTAGE);
        Leftback.configure(LeftBackConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        RightConfig.idleMode(IdleMode.kBrake).voltageCompensation(DriveConstants.COMPENSATION_VOLTAGE);
        LeftConfig.idleMode(IdleMode.kBrake).voltageCompensation(DriveConstants.COMPENSATION_VOLTAGE);

        LeftConfig.inverted(true);
        RightConfig.encoder.inverted(true);

        LeftConfig.encoder.countsPerRevolution(8192).velocityConversionFactor(DriveConstants.CONVERSAO_METROS_POR_SEG).positionConversionFactor(DriveConstants.CONVERSAO_METROS);
        RightConfig.encoder.countsPerRevolution(8192).velocityConversionFactor(DriveConstants.CONVERSAO_METROS_POR_SEG).positionConversionFactor(DriveConstants.CONVERSAO_METROS);

        Rightfront.configure(RightConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        Leftfront.configure(LeftConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        odometria = new DifferentialDriveOdometry(getAngulo(), encoderLeftFront.getPosition(), encoderRightFront.getPosition());
        SmartDashboard.putData("Campo", campo);
    }

    public void setTurbo(boolean ligado) {
        drive.setMaxOutput(ligado ? DriveConstants.VELOCIDADE_TURBO : DriveConstants.VELOCIDADE_NORMAL);
    }

    public Command commandTurbo() {
        return Commands.runEnd(
            () -> setTurbo(true),
            () -> setTurbo(false) );
    }

    public void resetarEncoder() {
        encoderLeftFront.setPosition(0);
        encoderRightFront.setPosition(0);
    }

    public double mediaAtual() {
        return (encoderLeftFront.getPosition() + encoderRightFront.getPosition()) / 2;
    }

    public void StopTraction(){
        drive.stopMotor();
    }

    public Rotation2d getAngulo() {
        return gyroscopio.getAngulo();
    }

    public double getVelocityGyro() {
        return gyroscopio.getVelocityGyro();
    }

    public void zerarAngulo() {
        gyroscopio.zerar();
    }

    public Pose2d getPose() {
        return odometria.getPoseMeters();
    }

    public void resetarPose(Pose2d pose) {
        odometria.resetPosition(getAngulo(),encoderLeftFront.getPosition(),encoderRightFront.getPosition(),pose);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("encoder direito", encoderRightFront.getPosition());
        SmartDashboard.putNumber("encoder esquedo", encoderLeftFront.getPosition());
        SmartDashboard.putNumber("Angulo", gyroscopio.getAngulo().getDegrees());
        SmartDashboard.putNumber("Graus/seg", gyroscopio.getVelocityGyro());

        posePublisher.set(getPose());
        odometria.update(getAngulo(), encoderLeftFront.getPosition(), encoderRightFront.getPosition());
        campo.setRobotPose(getPose());
        SmartDashboard.putNumber("Pose X",getPose().getX());
        SmartDashboard.putNumber("Pose Y", getPose().getY());
    }

    public void setVolts(double esquerda, double direita) {
        Leftfront.setVoltage(esquerda);
        Rightfront.setVoltage(direita);
        drive.feed();
    }

    public double velocidadeMedia() {
        return (encoderLeftFront.getVelocity() + encoderRightFront.getVelocity()) / 2.0;
    }

}