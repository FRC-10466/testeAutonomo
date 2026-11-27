package frc.robot.subystems.drive;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class DriveIOSparkMax implements DriveIO{

    private final SparkMax Rightfront = new SparkMax(DriveConstants.ID_RIGHT_FRONT,MotorType.kBrushed);
    public final SparkMax Rightback = new SparkMax(DriveConstants.ID_RIGHT_BACK,MotorType.kBrushed);
    
    private final SparkMax Leftfront = new SparkMax(DriveConstants.ID_LEFT_FRONT, MotorType.kBrushed);
    public final SparkMax Leftback = new SparkMax(DriveConstants.ID_LEFT_BACK, MotorType.kBrushed);

    public final DifferentialDrive drive = new DifferentialDrive(Leftfront, Rightfront);

    RelativeEncoder encoderRightFront = Rightfront.getEncoder();
    RelativeEncoder encoderLeftFront = Leftfront.getEncoder();
    
    public DriveIOSparkMax() {
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
    }

}




