package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subystem.traction.Drive;
import frc.robot.subystem.traction.RobotStraight;

public class RobotContainer {

  Drive tracao = new Drive();
  CommandXboxController controle = new CommandXboxController(0);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    tracao.setDefaultCommand(tracao.run(() -> {
          double speed = MathUtil.applyDeadband(-controle.getLeftY(), 0.1);
          double rotacao = MathUtil.applyDeadband(-controle.getRightX(), 0.1);
          double rotation = Math.copySign(rotacao * rotacao, rotacao);
          tracao.drive.arcadeDrive(speed, rotation);
        }));
    controle.a().toggleOnTrue(tracao.commandTurbo());
  }

  public Command getAutonomousCommand() {
    return Commands.sequence( new RobotStraight(tracao, 1));
  }
}
