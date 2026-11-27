package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subystems.drive.Drive;

import java.util.ArrayList;
import java.util.List;

/**
 * Mede kS e kV da tração. Adaptado de DriveCommands.feedforwardCharacterization, do template de
 * tank do AdvantageKit (FRC 6328, licença BSD).
 *
 * <p>Como usar: segure o botão; a tensão sobe devagar. Solte ANTES da parede. Os valores aparecem
 * no console do Driver Station e no dashboard.
 */
public final class CaracterizarFF {
  /** Quanto a tensão sobe por segundo. A 6328 usa 0.1; numa sala pequena, 0.3 a 0.5. */
  private static final double TAXA_VOLTS_POR_SEGUNDO = 0.3;

  private CaracterizarFF() {}

  public static Command medir(Drive tracao) {
    List<Double> velocidades = new ArrayList<>();
    List<Double> tensoes = new ArrayList<>();
    Timer timer = new Timer();

    return Commands.sequence(
            // 1) Zera tudo
            Commands.runOnce(
                () -> {
                  velocidades.clear();
                  tensoes.clear();
                  timer.restart();
                }),

            // 2) Sobe a tensão devagar e guarda (velocidade, tensão) a cada 20 ms
            Commands.run(
                    () -> {
                      double volts = timer.get() * TAXA_VOLTS_POR_SEGUNDO;
                      tracao.setVolts(volts, volts);
                      velocidades.add(tracao.velocidadeMedia());
                      tensoes.add(volts);
                    },
                    tracao)

                // 3) Quando parar: para o robô, calcula a reta e mostra kS e kV
                .finallyDo(
                    () -> {
                      tracao.setVolts(0, 0);
                      int n = velocidades.size();
                      double somaX = 0, somaY = 0, somaXY = 0, somaX2 = 0;
                      for (int i = 0; i < n; i++) {
                        double x = velocidades.get(i);
                        double y = tensoes.get(i);
                        somaX += x;
                        somaY += y;
                        somaXY += x * y;
                        somaX2 += x * x;
                      }
                      double divisor = n * somaX2 - somaX * somaX;
                      if (n < 10 || divisor == 0) {
                        System.out.println("CaracterizarFF: poucos dados, rode por mais tempo.");
                        return;
                      }
                      double kS = (somaY * somaX2 - somaX * somaXY) / divisor;
                      double kV = (n * somaXY - somaX * somaY) / divisor;

                      System.out.println("********** Resultado do feedforward **********");
                      System.out.printf("  kS: %.5f V%n", kS);
                      System.out.printf("  kV: %.6f V por unidade de velocidade%n", kV);
                      System.out.println("  amostras: " + n);
                      SmartDashboard.putNumber("FF/kS", kS);
                      SmartDashboard.putNumber("FF/kV", kV);
                    }))
        .withName("CaracterizarFF");
  }
}