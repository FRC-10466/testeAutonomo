package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subystem.tracao.tracao;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

  tracao tracao = new tracao();
  CommandXboxController controle = new CommandXboxController(0);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    controle.a().onTrue(tracao.dizerOla());
  }

  public Command getAutonomousCommand() {
    return Commands.sequence(
      tracao.dizerOla(),
      tracao.tracaoPor(2),
      Commands.print("acabou"));
  }
}
