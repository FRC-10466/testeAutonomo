package frc.robot.subystem.traction;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
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

    PIDController pidM_Right = new PIDController(0, 0, 0);
    PIDController pidM_Left = new PIDController(0, 0, 0);
    
    private static final double VELOCIDADE_NORMAL = 0.7;
    private static final double VELOCIDADE_TURBO = 0.9;

    public Drive() {
        SparkMaxConfig RightConfig = new SparkMaxConfig ();
        SparkMaxConfig LeftConfig = new SparkMaxConfig();

        SparkMaxConfig RightBackConfig = new SparkMaxConfig ();
        RightBackConfig.follow(Rightfront);
        Rightback.configure(RightBackConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig LeftBackConfig = new SparkMaxConfig ();
        LeftBackConfig.follow(Leftfront);
        Leftback.configure(LeftBackConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        RightConfig.inverted(true);

        SmartDashboard.putData("PID MOTOR E", pidM_Left);
        SmartDashboard.putData("PID MOTOR D", pidM_Right);

        LeftConfig.encoder.countsPerRevolution(8192).positionConversionFactor(Math.PI * 0.14);
        RightConfig.encoder.countsPerRevolution(8192).positionConversionFactor(Math.PI * 0.14);

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

    public void andarDireto(double metros) {
        pidM_Left.setSetpoint(metros);
        pidM_Right.setSetpoint(metros);
        
        pidM_Left.setTolerance(0.05);
        pidM_Right.setTolerance(0.05);

        double potenciaMotor_L = pidM_Left.calculate(encoderLeftFront.getPosition());
        double potenciaMotor_R = pidM_Right.calculate(encoderRightFront.getPosition());

        drive.tankDrive(potenciaMotor_L, potenciaMotor_R);
    }

    public double mediaAtual() {
        double media = encoderLeftFront.getPosition() + encoderRightFront.getPosition();
        return media / 2;
    }

    public void StopTraction(){
        drive.stopMotor();
    }

}