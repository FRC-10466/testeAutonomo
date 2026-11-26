package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subystem.traction.Drive;
import frc.robot.auto.AndarReto;
import frc.robot.auto.GirarAte;
import frc.robot.subystem.traction.CaracterizarFF;

public class RobotContainer {

  Drive tracao = new Drive();
  CommandXboxController controle = new CommandXboxController(0);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    tracao.setDefaultCommand(tracao.run(() -> {
          double speed = MathUtil.applyDeadband(-controle.getLeftY(), 0.1);
          double rotacao = MathUtil.applyDeadband(-controle.getRightX() * 0.6, 0.1);
          tracao.drive.arcadeDrive(speed, rotacao);
        }));
    controle.a().toggleOnTrue(tracao.commandTurbo());
    controle.povUp().whileTrue(CaracterizarFF.medir(tracao));
  }

  public Command getAutonomousCommand() {
    return Commands.sequence(
      new GirarAte(tracao, 90),
      new AndarReto(tracao, 1));
  }
}
