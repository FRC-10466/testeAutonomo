package frc.robot.subystem.traction;

import edu.wpi.first.wpilibj2.command.Command;

public class RobotStraight extends Command{

    private final Drive tracao;
    private final double metros;   


    public RobotStraight(Drive tracao, double metros) {
        this.tracao = tracao;
        this.metros = metros;
        addRequirements(tracao);
    }

    @Override
    public void initialize() {
        tracao.mediaAtual();
    }

    @Override
    public void execute() {        
    }

    @Override
    public boolean isFinished() {
        
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}
