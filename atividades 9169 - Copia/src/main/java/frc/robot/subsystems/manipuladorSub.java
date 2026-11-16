
package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.Constants;

public class manipuladorSub
        extends edu.wpi.first.wpilibj2.command.SubsystemBase {

    private double anguloAtual =
            Constants.Manipulador.ANGULO_INICIAL;

    private double setpoint =
            Constants.Manipulador.ANGULO_INICIAL;

    private final Mechanism2d mecanismo =
            new Mechanism2d(1.0, 1.0);

    private final MechanismRoot2d base =
            mecanismo.getRoot("Base", 0.5, 0.1);

    private final MechanismLigament2d braco =
            base.append(
                    new MechanismLigament2d(
                            "Braco",
                            0.5,
                            anguloAtual,
                            6,
                            new Color8Bit(Color.kGreen)));

    public manipuladorSub() {

        SmartDashboard.putData(
                "Manipulador/Mecanismo",
                mecanismo);
    }

    public void definirAngulo(double angulo) {

        setpoint = MathUtil.clamp(
                angulo,
                Constants.Manipulador.ANGULO_MIN,
                Constants.Manipulador.ANGULO_MAX);
    }

    public double getAngulo() {
        return anguloAtual;
    }

    public double getSetpoint() {
        return setpoint;
    }

    public boolean estaNoSetpoint() {

        return Math.abs(setpoint - anguloAtual)
                <= Constants.Manipulador.TOLERANCIA;
    }

    public boolean estaNoZero() {

        return Math.abs(anguloAtual)
                <= Constants.Manipulador.TOLERANCIA;
    }

    public Command irParaAngulo(double angulo) {

        return Commands.run(
                () -> definirAngulo(angulo),
                this)
                .until(this::estaNoSetpoint);
    }

    // Comando para arremessar a Alga
    public Command arremessarAlgae1(intakeSub intake) {

        return Commands.sequence(

                // Move o manipulador para a posição de lançamento
                irParaAngulo(
                        Constants.Manipulador.ANGULO_MAX),

                // Atualiza o estado da Alga
                Commands.runOnce(
                        () -> intake.definirPosicaoalgae(
                                intakeSub.posicaoalgae.ARREMESSADA),
                        intake),

                Commands.waitSeconds(0.5),

                // Retorna à posição inicial
                irParaAngulo(
                        Constants.Manipulador.ANGULO_INICIAL)

        ).onlyIf(intake::temAlgaeNoManipulador);
    }

    @Override
    public void periodic() {

        double diferenca = setpoint - anguloAtual;

        double variacaoMaxima =
                Constants.Manipulador.VELOCIDADE_MAX
                        * Constants.Simulacao.PERIOD;

        double variacao = MathUtil.clamp(
                diferenca,
                -variacaoMaxima,
                variacaoMaxima);

        anguloAtual += variacao;

        braco.setAngle(anguloAtual);

        SmartDashboard.putNumber(
                "Manipulador/Angulo",
                anguloAtual);

        SmartDashboard.putNumber(
                "Manipulador/Setpoint",
                setpoint);

        SmartDashboard.putBoolean(
                "Manipulador/NoZero",
                estaNoZero());
    }
}
