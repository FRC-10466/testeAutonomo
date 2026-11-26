package frc.robot.subystem.traction;

public class DriveConstants {

    public final int ID_RIGHT_FRONT = 7;
    public final int ID_RIGHT_BACK = 5;
    
    public final int ID_LEFT_FRONT = 6;
    public final int ID_LEFT_BACK = 8;

    //-----------------------------------------------------------------------------

    public final double VELOCIDADE_NORMAL = 0.7;
    public final double VELOCIDADE_TURBO = 0.9;

    //------------------------------------------------------------------------------

    public final double P = 1.5;
    public final double I = 0;
    public final double D = 0.05;

    //----------------------------------------------------------------------------

    public final double KV = 3.39;
    public final double KS = 0.82;

    //-------------------------------------------------------------------------------

    public final double CONVERSAO_METROS_POR_SEG = Math.PI * 0.1524 / 60;
    public final double CONVERSAO_METROS = Math.PI * 0.1524;
}
