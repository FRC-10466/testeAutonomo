package frc.robot.subystem.traction;

import edu.wpi.first.wpilibj2.command.Command;

public class RobotStraight extends Command{

    private final Drive tracao;
    private final double metros;   
    private double inicio;

    public RobotStraight(Drive tracao, double metros) {
        this.tracao = tracao;
        this.metros = metros;
        addRequirements(tracao);
    }

    @Override
    public void initialize() {
        inicio = tracao.mediaAtual();
    }

    @Override
    public void execute() {
        tracao.andarDireto(Math.signum(metros) * 0.3);        
    }

    @Override
    public boolean isFinished() {
        return Math.abs(tracao.mediaAtual() - inicio) >= metros;
    }

    @Override
    public void end(boolean interrupted) {
        tracao.StopTraction();
    }
}
