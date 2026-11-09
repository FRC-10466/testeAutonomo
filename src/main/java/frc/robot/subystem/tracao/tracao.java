package frc.robot.subystem.tracao;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class tracao extends SubsystemBase{

 public Command dizerOla() {
    return runOnce(() -> System.out.println("Começo"));
 }

 public Command tracaoPor(int segundos) {
    return startEnd(
        () -> System.out.println("tracou iniciou"),
        () -> System.out.println("tracao acabou"))
        .withTimeout(segundos);
 }
}
