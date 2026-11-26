package frc.robot.subystem.traction;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.auto.Gyroscopio;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.PersistMode;

public class Drive extends SubsystemBase{

    Gyroscopio gyroscopio = new Gyroscopio();
    DriveConstants constants = new DriveConstants();

    private final SparkMax Rightfront = new SparkMax(constants.ID_RIGHT_FRONT,MotorType.kBrushed);
    public final SparkMax Rightback = new SparkMax(constants.ID_RIGHT_BACK,MotorType.kBrushed);
    
    private final SparkMax Leftfront = new SparkMax(constants.ID_LEFT_FRONT, MotorType.kBrushed);
    public final SparkMax Leftback = new SparkMax(constants.ID_LEFT_BACK, MotorType.kBrushed);

    public final DifferentialDrive drive = new DifferentialDrive(Leftfront, Rightfront);

    RelativeEncoder encoderRightFront = Rightfront.getEncoder();
    RelativeEncoder encoderLeftFront = Leftfront.getEncoder();
    
    SparkClosedLoopController pidM_Right = Rightfront.getClosedLoopController();
    SparkClosedLoopController pidM_Left = Leftfront.getClosedLoopController();

    public Drive() {
        SparkMaxConfig RightConfig = new SparkMaxConfig ();
        SparkMaxConfig LeftConfig = new SparkMaxConfig();

        SparkMaxConfig RightBackConfig = new SparkMaxConfig ();
        RightBackConfig.follow(Rightfront);
        RightBackConfig.idleMode(IdleMode.kBrake).voltageCompensation(12);
        Rightback.configure(RightBackConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig LeftBackConfig = new SparkMaxConfig ();
        LeftBackConfig.follow(Leftfront);
        LeftBackConfig.idleMode(IdleMode.kBrake).voltageCompensation(12);
        Leftback.configure(LeftBackConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        RightConfig.idleMode(IdleMode.kBrake).voltageCompensation(12);
        LeftConfig.idleMode(IdleMode.kBrake).voltageCompensation(12);

        RightConfig.closedLoop.pid(constants.P, constants.I, constants.D).feedForward.kV(constants.KV).kS(constants.KS);
        LeftConfig.closedLoop.pid(constants.P,constants.I, constants.D).feedForward.kV(constants.KV).kS(constants.KS);
        LeftConfig.closedLoop.maxMotion.cruiseVelocity(0.5).maxAcceleration(0.5);
        RightConfig.closedLoop.maxMotion.cruiseVelocity(0.5).maxAcceleration(0.5);
        LeftConfig.closedLoop.outputRange(-0.4, 0.4);
        RightConfig.closedLoop.outputRange(-0.4, 0.4);

        LeftConfig.inverted(true);
        RightConfig.encoder.inverted(true);

        LeftConfig.closedLoop.allowedClosedLoopError(0.02, ClosedLoopSlot.kSlot0);
        RightConfig.closedLoop.allowedClosedLoopError(0.02, ClosedLoopSlot.kSlot0);

        LeftConfig.encoder.countsPerRevolution(8192).velocityConversionFactor(constants.CONVERSAO_METROS_POR_SEG).positionConversionFactor(constants.CONVERSAO_METROS);
        RightConfig.encoder.countsPerRevolution(8192).velocityConversionFactor(constants.CONVERSAO_METROS_POR_SEG).positionConversionFactor(constants.CONVERSAO_METROS);

        Rightfront.configure(RightConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        Leftfront.configure(LeftConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        drive.setMaxOutput(constants.VELOCIDADE_NORMAL);
    }

    public void setTurbo(boolean ligado) {
        drive.setMaxOutput(ligado ? constants.VELOCIDADE_TURBO : constants.VELOCIDADE_NORMAL);
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

    public void andarDireto(double alvoDir, double alvoEsq) {
        pidM_Right.setSetpoint(alvoDir, ControlType.kMAXMotionPositionControl);
        pidM_Left.setSetpoint(alvoEsq, ControlType.kMAXMotionPositionControl);   
        drive.feed();
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

    @Override
    public void periodic() {
        SmartDashboard.putNumber("encoder direito", encoderRightFront.getPosition());
        SmartDashboard.putNumber("encoder esquedo", encoderLeftFront.getPosition());
        SmartDashboard.putNumber("Angulo", gyroscopio.getAngulo().getDegrees());
        SmartDashboard.putNumber("Graus/seg", gyroscopio.getVelocityGyro());
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