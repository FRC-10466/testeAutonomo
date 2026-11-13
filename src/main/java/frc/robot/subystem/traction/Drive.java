package frc.robot.subystem.traction;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.ResetMode;
import com.revrobotics.RelativeEncoder;

import com.revrobotics.PersistMode;

public class Drive extends SubsystemBase{

    private final SparkMax Rightfront = new SparkMax(4,MotorType.kBrushed);
    private final SparkMax Leftfront = new SparkMax(2, MotorType.kBrushed);

    public final SparkMax Rightback = new SparkMax(3,MotorType.kBrushed);
    public final SparkMax Leftback = new SparkMax(1, MotorType.kBrushed);

    public final DifferentialDrive drive = new DifferentialDrive(Leftfront, Rightfront);

    RelativeEncoder encoderRightFront = Rightfront.getEncoder();
    RelativeEncoder encoderLeftFront = Leftfront.getEncoder();
    
    SparkClosedLoopController pidM_Right = Rightfront.getClosedLoopController();
    SparkClosedLoopController pidM_Left = Leftfront.getClosedLoopController();
    
    private static final double VELOCIDADE_NORMAL = 0.7;
    private static final double VELOCIDADE_TURBO = 0.9;

    public Drive() {
        SparkMaxConfig RightConfig = new SparkMaxConfig ();
        SparkMaxConfig LeftConfig = new SparkMaxConfig();

        SparkMaxConfig RightBackConfig = new SparkMaxConfig ();
        RightBackConfig.follow(Rightfront);
        RightBackConfig.idleMode(IdleMode.kBrake).voltageCompensation(12);
        Rightback.configure(RightBackConfig,ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        SparkMaxConfig LeftBackConfig = new SparkMaxConfig ();
        LeftBackConfig.follow(Leftfront);
        LeftBackConfig.idleMode(IdleMode.kBrake).voltageCompensation(12);
        Leftback.configure(LeftBackConfig,ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        RightConfig.inverted(true);
        RightConfig.encoder.inverted(true);

        RightConfig.idleMode(IdleMode.kBrake).voltageCompensation(12);
        LeftConfig.idleMode(IdleMode.kBrake).voltageCompensation(12);

        RightConfig.closedLoop.pid(1.5, 0, 0.05).feedForward.kS(0.2);
        LeftConfig.closedLoop.pid(1.5,0, 0.05).feedForward.kS(0.2);
        LeftConfig.closedLoop.outputRange(0.4, -0.4);
        RightConfig.closedLoop.outputRange(0.4, -0.4);

        LeftConfig.closedLoop.allowedClosedLoopError(0.02, ClosedLoopSlot.kSlot0);
        RightConfig.closedLoop.allowedClosedLoopError(0.02, ClosedLoopSlot.kSlot0);

        LeftConfig.closedLoop.maxMotion.cruiseVelocity(0.5).maxAcceleration(0.5);
        RightConfig.closedLoop.maxMotion.cruiseVelocity(0.5).maxAcceleration(0.5);

        LeftConfig.encoder.countsPerRevolution(8192).velocityConversionFactor(Math.PI * 0.1524 / 60).positionConversionFactor(Math.PI * 0.1524);
        RightConfig.encoder.countsPerRevolution(8192).velocityConversionFactor(Math.PI * 0.1524 / 60).positionConversionFactor(Math.PI * 0.1524);

        Rightfront.configure(RightConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        Leftfront.configure(LeftConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        drive.setMaxOutput(VELOCIDADE_NORMAL);
    }

    public void setTurbo(boolean ligado) {
        drive.setMaxOutput(ligado ? VELOCIDADE_TURBO : VELOCIDADE_NORMAL);
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

    public void andarDireto(double metros) {
        pidM_Right.setSetpoint(metros, ControlType.kMAXMotionPositionControl);
        pidM_Left.setSetpoint(metros, ControlType.kMAXMotionPositionControl);   

        drive.feed();
    }

    public double mediaAtual() {
        double media = encoderLeftFront.getPosition() + encoderRightFront.getPosition();
        return media / 2;
    }

    public void StopTraction(){
        drive.stopMotor();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("encoder direito", encoderRightFront.getPosition());
        SmartDashboard.putNumber("encoder esquedo", encoderLeftFront.getPosition());
    }

}