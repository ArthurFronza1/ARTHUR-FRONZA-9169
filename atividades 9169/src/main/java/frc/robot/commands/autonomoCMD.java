package frc.robot.commands;

import java.util.Set;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;

import frc.robot.subsystems.elevadorSub;
import frc.robot.subsystems.intakeSub;
import frc.robot.subsystems.manipuladorSub;

public final class autonomoCMD {

        private autonomoCMD() {
        }

        public static void registrar1(
                        intakeSub intake,
                        elevadorSub elevador,
                        manipuladorSub manipulador) {

                // Requisitos dos subsistemas
                Set<Subsystem> requisitos = Set.of(intake, elevador, manipulador);

                // Coleta uma Alga
                NamedCommands.registerCommand(
                                "ColetarAlga",
                                Commands.defer(
                                                () -> coletar(intake),
                                                requisitos));

                // Pontuação na NET
                NamedCommands.registerCommand(
                                "PontuarNet",
                                Commands.defer(
                                                () -> arremessar(intake, manipulador),
                                                Set.of(intake, manipulador)));

                // Pontuação no PROCESSOR
                NamedCommands.registerCommand(
                                "PontuarProcessor",
                                Commands.defer(
                                                () -> arremessar(intake, manipulador),
                                                Set.of(intake, manipulador)));
        }

        private static Command coletar(intakeSub intake) {

                return Commands.sequence(

                                Commands.runOnce(
                                                () -> intake.definirPosicaoalgae(
                                                                intakeSub.posicaoalgae.FORA),
                                                intake),

                                intake.coletarAlgae(),

                                intake.transferirAlgae()

                ).withTimeout(5.0);
        }

        private static Command arremessar(
                        intakeSub intake,
                        manipuladorSub manipulador) {

                return manipulador
                                .arremessarAlgae1(intake)
                                .withTimeout(10.0);
        }

        // Compatibilidade com chamadas que utilizam registrar()
        public static void registrar(
                        intakeSub intake,
                        elevadorSub elevador,
                        manipuladorSub manipulador) {

                registrar1(intake, elevador, manipulador);
        }
}