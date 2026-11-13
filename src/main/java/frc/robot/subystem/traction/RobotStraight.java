package frc.robot.subystem.traction;

import edu.wpi.first.wpilibj2.command.Command;

public class RobotStraight extends Command{

    private final Drive tracao;
    private final double metros;   
    private double alvoEsq;
    private double alvoDir;

    public RobotStraight(Drive tracao, double metros) {
        this.tracao = tracao;
        this.metros = metros;
        addRequirements(tracao);
    }

    @Override
    public void initialize() {
        alvoEsq = tracao.encoderLeftFront.getPosition() + metros;
        alvoDir = tracao.encoderRightFront.getPosition() + metros;
    }

    @Override
    public void execute() {
        tracao.andarDireto(alvoDir, alvoEsq);        
    }

    // @Override
    // public boolean isFinished() {
    //     return Math.abs(tracao.mediaAtual() - inicio) >= metros;
    // }

    @Override
    public void end(boolean interrupted) {
        tracao.StopTraction();
    }
}
