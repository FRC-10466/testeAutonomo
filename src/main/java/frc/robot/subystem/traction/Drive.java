package frc.robot.subystem.traction;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.ResetMode;

import com.revrobotics.PersistMode;

public class Drive extends SubsystemBase{

    private final SparkMax Rightfront = new SparkMax(4,MotorType.kBrushed);
    private final SparkMax Leftfront = new SparkMax(2, MotorType.kBrushed);

    public final SparkMax Rightback = new SparkMax(3,MotorType.kBrushed);
    public final SparkMax Leftback = new SparkMax(1, MotorType.kBrushed);

    public final DifferentialDrive drive;

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

        Rightfront.configure(RightConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        Leftfront.configure(LeftConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        drive = new DifferentialDrive(Leftfront, Rightfront);
        
        drive.setMaxOutput(VELOCIDADE_NORMAL);
    }

    public void setTurbo(boolean ligado) {
        drive.setMaxOutput(ligado ? VELOCIDADE_TURBO : VELOCIDADE_NORMAL);
    }

    public Command commandTurbo() {
        return runEnd(
            () -> setTurbo(true),
            () -> setTurbo(false) );
    }

    public void andarDireto(double velocidade) {
        Leftfront.set(velocidade);
        Rightfront.set(velocidade);
        drive.feed();
    }

    public void StopTraction(){
        drive.stopMotor();
    }

}