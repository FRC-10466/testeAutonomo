package frc.robot.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subystem.traction.Drive;

public class AndarReto extends Command {
  private final Drive tracao;
  private final double metros;
  private double inicio;
  private double anguloAlvo;

  private final TrapezoidProfile perfil = new TrapezoidProfile(new TrapezoidProfile.Constraints(1, 1));
  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.82, 3.39);
  private final PIDController pidDistance = new PIDController(8, 0, 0);
  private final PIDController pidHeading = new PIDController(0.1, 0,0);

  private TrapezoidProfile.State posicaoAtual;
  private TrapezoidProfile.State objetivo;
  
  public AndarReto(Drive tracao, double metros) {
    this.tracao = tracao;
    this.metros = metros;
    addRequirements(tracao);
  }

  @Override
  public void initialize() {
    inicio = tracao.mediaAtual();
    anguloAlvo = tracao.getAngulo().getDegrees();
    posicaoAtual = new TrapezoidProfile.State(0,0);
    objetivo = new TrapezoidProfile.State(metros,0);
    pidDistance.reset();
    pidHeading.reset();
  }

  @Override
  public void execute() {
    posicaoAtual = perfil.calculate(0.02, posicaoAtual, objetivo);
    double distancia = tracao.mediaAtual() - inicio;
    double voltsAndar = feedforward.calculate(posicaoAtual.velocity) + pidDistance.calculate(distancia, posicaoAtual.position);
    double voltsCorrigir = pidHeading.calculate(tracao.getAngulo().getDegrees(), anguloAlvo);

    tracao.setVolts(voltsAndar - voltsCorrigir, voltsAndar + voltsCorrigir);
  }
  
  @Override
  public boolean isFinished() {
    double distancia = tracao.mediaAtual() - inicio;
    boolean acabo = posicaoAtual.position == objetivo.position && posicaoAtual.velocity == 0;
    return acabo && Math.abs(metros - distancia) < 0.02;
  }
 
  @Override
  public void end(boolean interrupted) {
    tracao.setVolts(0, 0);
  }

}
