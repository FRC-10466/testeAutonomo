package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.drive.AndarReto;
import frc.robot.commands.drive.CaracterizarFF;
import frc.robot.commands.drive.GirarAte;
import frc.robot.subystems.drive.Drive;
import frc.robot.subystems.drive.DriveIOSparkMax;
import frc.robot.subystems.drive.GyroIOPigeon2;


public class RobotContainer {
  
  Drive tracao;
  CommandXboxController controle = new CommandXboxController(0);

  public RobotContainer() {
    if (RobotBase.isReal()) {
      tracao = new Drive(new DriveIOSparkMax(), new GyroIOPigeon2());
    } // else {
      //DriveIOSim driveSim = new DriveIOSim();
      //tracao = new Drive(driveSim, new GyroIOSim(driveSim))
    //}
    configureBindings();
  }

  private void configureBindings() {
    tracao.setDefaultCommand(tracao.run(() -> {
          double speed = MathUtil.applyDeadband(-controle.getLeftY() * 0.8, 0.1);
          double rotacao = MathUtil.applyDeadband(-controle.getRightX() * 0.6, 0.1);
          tracao.arcade(speed, rotacao);
        }));
    controle.a().toggleOnTrue(tracao.commandTurbo());
    controle.povUp().whileTrue(CaracterizarFF.medir(tracao));
  }

  public Command getAutonomousCommand() {
    return Commands.sequence(Commands.runOnce(() -> tracao.resetarPose(new Pose2d(3.5,3,Rotation2d.kZero))),
      new GirarAte(tracao, 90).withTimeout(5),
      new AndarReto(tracao, 1));
  }
}
