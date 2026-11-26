package frc.robot.subystem.traction;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

public class GirarAte extends Command {
 
  private final Drive tracao;
  private final double graus;
  private final PIDController pid = new PIDController(0.15, 0, 0.01);
  private static final double KS_GIRO = 0.82;
  private double alvo;

  public GirarAte(Drive tracao, double graus) {
    this.tracao = tracao;
    this.graus = graus;
    pid.setTolerance(2);
    addRequirements(tracao);
  }

  @Override
  public void initialize() {
    alvo =  tracao.getAngulo().getDegrees()+ graus;
    pid.reset();
  }

  @Override
  public void execute() {
    double angulo = tracao.getAngulo().getDegrees();
    double erro = alvo - angulo;
    double volts = pid.calculate(angulo,alvo);
    if (Math.abs(erro) > 2) {
      volts =+ KS_GIRO * Math.signum(erro);
    }
    volts = MathUtil.clamp(volts, -6, 6);
    tracao.setVolts(-volts, volts);
  }

  
  @Override
  public boolean isFinished() {
    return pid.atSetpoint() && Math.abs(tracao.getVelocityGyro()) < 5;
  }

  @Override
  public void end(boolean interrupted) {
    tracao.setVolts(0, 0);
  }
}
