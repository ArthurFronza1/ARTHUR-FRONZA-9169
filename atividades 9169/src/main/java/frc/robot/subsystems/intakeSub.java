package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class intakeSub extends SubsystemBase {

    public enum posicaoalgae {
        FORA,
        CENTRO_INTAKE,
        MANIPULADOR,
        ARREMESSADA
    }

    private static final double ANGULO_FECHADO = Units.degreesToRadians(0.0);
    private static final double ANGULO_ABERTO = Units.degreesToRadians(-100.0);
    private static final double VELOCIDADE_ANGULAR = Units.degreesToRadians(180.0);
    private static final double TOLERANCIA = Units.degreesToRadians(2.0);
    private static final double PERIOD = 0.02;

    private static final Translation3d PIVO = new Translation3d(0.30, 0.0, 0.25);

    private posicaoalgae posicaoAlgae = posicaoalgae.FORA;

    private boolean aberto = false;

    private double anguloAtual = ANGULO_FECHADO;
    private double anguloAlvo = ANGULO_FECHADO;

    private BooleanSupplier coletor = () -> false;

    private final StructArrayPublisher<Pose3d> componentePub = NetworkTableInstance.getDefault()
            .getStructArrayTopic("Intake/Pose3d", Pose3d.struct)
            .publish();

    public intakeSub() {
    }

    public void setColetor(BooleanSupplier coletor) {
        this.coletor = coletor;
    }

    public void definirPosicaoalgae(posicaoalgae posicao) {
        posicaoAlgae = posicao;
    }

    public posicaoalgae getPosicaoAlgae() {
        return posicaoAlgae;
    }

    public boolean temAlgaeNoCentro() {
        return posicaoAlgae == posicaoalgae.CENTRO_INTAKE;
    }

    public boolean temAlgaeNoManipulador() {
        return posicaoAlgae == posicaoalgae.MANIPULADOR;
    }

    public void abrir() {
        aberto = true;
        anguloAlvo = ANGULO_ABERTO;
    }

    public void fechar() {
        aberto = false;
        anguloAlvo = ANGULO_FECHADO;
    }

    public boolean estaAberto() {
        return aberto;
    }

    public boolean noAlvo() {
        return Math.abs(anguloAlvo - anguloAtual) <= TOLERANCIA;
    }

    public Command alternarIntake() {
        return Commands.runOnce(() -> {
            if (aberto) {
                fechar();
            } else {
                abrir();
            }
        }, this);
    }

    public Command coletarAlgae() {
        return Commands.sequence(

                // Abre o Intake
                Commands.runOnce(this::abrir, this),

                // Aguarda o movimento de abertura
                Commands.waitUntil(this::noAlvo).withTimeout(2.0),

                Commands.waitSeconds(0.5),

                // Tenta coletar uma Alga
                Commands.runOnce(() -> {
                    if (posicaoAlgae == posicaoalgae.FORA
                            || posicaoAlgae == posicaoalgae.ARREMESSADA) {

                        if (coletor.getAsBoolean()) {
                            definirPosicaoalgae(posicaoalgae.CENTRO_INTAKE);
                        }
                    }
                }, this),

                // Fecha o Intake
                Commands.runOnce(this::fechar, this),

                // Aguarda o movimento de fechamento
                Commands.waitUntil(this::noAlvo).withTimeout(2.0));
    }

    public Command transferirAlgae() {
        return Commands.sequence(
                Commands.waitSeconds(1.0),

                Commands.runOnce(
                        () -> definirPosicaoalgae(posicaoalgae.MANIPULADOR),
                        this))
                .onlyIf(this::temAlgaeNoCentro);
    }

    @SuppressWarnings("unused")
    @Override
    public void periodic() {
        double passo = VELOCIDADE_ANGULAR * PERIOD;

        anguloAtual += MathUtil.clamp(
                anguloAlvo - anguloAtual,
                -passo,
                passo);

        Pose3d pose3d = new Pose3d(
                PIVO,
                new Rotation3d(0.0, anguloAtual, 0.0));

        componentePub.set(new Pose3d[] { new Pose3d() });

        SmartDashboard.putString(
                "Intake/PosicaoAlgae",
                posicaoAlgae.toString());

        SmartDashboard.putBoolean("Intake/Aberto", aberto);

        SmartDashboard.putNumber(
                "Intake/AnguloGraus",
                Units.radiansToDegrees(anguloAtual));

        SmartDashboard.putBoolean(
                "Intake/AlgaeNoCentro",
                temAlgaeNoCentro());

        SmartDashboard.putBoolean(
                "Intake/AlgaeNoManipulador",
                temAlgaeNoManipulador());
    }
}