package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.subsystems.SimElevator;
import frc.robot.subsystems.SimHand;
import frc.robot.subsystems.SimIntake;

public class Autos {

        // =========================================================
        // AUTONOMO QUADRADO
        // =========================================================

        public static Command quadrado(TankSubsystem tank) {

                return Commands.sequence(
                                tank.driveToPose(2.0, 2.0, 0),
                                tank.driveToPose(4.0, 2.0, 90),
                                tank.driveToPose(4.0, 5.0, 180),
                                tank.driveToPose(2.0, 5.0, -90));
        }

        // =========================================================
        // AUTONOMO RETO
        // =========================================================

        public static Command reto(TankSubsystem tank) {

                return Commands.sequence(
                                tank.driveToPose(2.0, 2.0, 0),
                                tank.driveToPose(5.0, 2.0, 0));
        }

        // =========================================================
        // AUTONOMO ZIGZAG
        // =========================================================

        public static Command autoZigZag(
                        TankSubsystem tank,
                        SimIntake intake,
                        SimElevator elevator,
                        SimHand hand) {

                return Commands.sequence(
                                tank.driveToPose(2.0, 2.0, 0),
                                intake.CMDintakeOn(),
                                tank.driveToPose(4.0, 3.0, 45),
                                tank.driveToPose(6.0, 2.0, -45));
        }

        // =========================================================
        // DEPOSITAR GAME PIECE
        // =========================================================

        public static Command depositGamePiece(
                        TankSubsystem tank,
                        SimHand hand) {

                return Commands.sequence(

                                Commands.runOnce(() -> System.out.println(">>> INICIANDO DEPOSITO")),

                                hand.CMDsetVelocity(1),

                                tank.depositFuelCommand(),

                                Commands.waitSeconds(1),

                                hand.CMDsetVelocity(0),

                                Commands.runOnce(() -> System.out.println(">>> DEPOSITO FINALIZADO")));
        }

        // =========================================================
        // TRANSFERIR PARA HAND
        // =========================================================

        public static Command transferirParaHand(
                        TankSubsystem tank,
                        SimElevator elevator,
                        SimHand hand) {

                return Commands.sequence(

                                Commands.runOnce(() -> System.out.println(">>> INICIANDO TRANSFERENCIA")),

                                // Eleva o elevador
                                elevator.CMDsetPosition(400, 3),

                                Commands.waitUntil(
                                                () -> SimElevator.getRealPosition() > 390).withTimeout(3),

                                Commands.runOnce(() -> System.out.println(
                                                ">>> POSICAO ELEVADOR: "
                                                                + SimElevator.getRealPosition())),

                                // Posiciona a Hand
                                hand.CMDsetAngle(0, 3),

                                Commands.waitUntil(
                                                () -> Math.abs(SimHand.getRealAngle()) < 5).withTimeout(3),

                                Commands.runOnce(() -> System.out.println(
                                                ">>> ANGULO HAND: "
                                                                + SimHand.getRealAngle())),

                                hand.CMDsetAngle(-120, 3),

                                // Ativa o mecanismo da Hand
                                hand.CMDsetVelocity(-1),

                                Commands.waitUntil(
                                                tank::isHandCollected).withTimeout(3),

                                hand.CMDsetVelocity(0),

                                Commands.runOnce(() -> {

                                        if (tank.isHandCollected()) {
                                                System.out.println(
                                                                ">>> ALGAE TRANSFERIDA PARA HAND");
                                        } else {
                                                System.out.println(
                                                                ">>> ERRO: HAND NAO RECEBEU A ALGAE");
                                        }
                                }));
        }  public static Command resetangleCommand(
                        TankSubsystem tank,
                        SimElevator elevator,
                        SimHand hand) {

                return Commands.sequence(

                                Commands.runOnce(() -> System.out.println(">>> INICIANDO TRANSFERENCIA")),

                                // Eleva o elevador
                                elevator.CMDsetPosition(0, 3),

                                Commands.waitUntil(
                                                () -> SimElevator.getRealPosition() > 390).withTimeout(3),

                                Commands.runOnce(() -> System.out.println(
                                                ">>> POSICAO ELEVADOR: "
                                                                + SimElevator.getRealPosition())),

                                // Posiciona a Hand
                                hand.CMDsetAngle(0, 3));

                                
                                };
        

        // =========================================================
        // COLETAR ALGAE
        // =========================================================

        public static Command coletarAlgae(
                        TankSubsystem tank,
                        SimIntake intake,
                        SimElevator elevator,
                        SimHand hand) {

                return Commands.sequence(

                                Commands.runOnce(() -> System.out.println(">>> INICIANDO COLETA")),

                                // Liga o Intake
                                intake.CMDintakeOn(),

                                // Espera a Algae entrar no robô
                                Commands.waitUntil(
                                                tank::isFuelInsideRobot).withTimeout(3),

                                Commands.runOnce(() -> {

                                        if (tank.isFuelInsideRobot()) {
                                                System.out.println(
                                                                ">>> ALGAE DETECTADA NO ROBÔ");
                                        } else {
                                                System.out.println(
                                                                ">>> ERRO: ALGAE NAO FOI COLETADA");
                                        }
                                }),

                                // Só transfere se o robô tiver coletado a Algae
                                Commands.either(

                                                transferirParaHand(tank, elevator, hand),

                                                Commands.runOnce(() -> System.out.println(
                                                                ">>> TRANSFERENCIA CANCELADA: SEM ALGAE")),

                                                tank::isFuelInsideRobot),

                                // Desliga o Intake
                                intake.CMDintakeOff(),

                                Commands.runOnce(() -> System.out.println(">>> ETAPA DE COLETA FINALIZADA")));
        }

        // =========================================================
        // SEGUIR PATHPLANNER
        // (VERSAO COM DIAGNOSTICO: imprime tempo total e pontos do Path)
        // =========================================================
        public static Command seguirPath(String nome) {

                try {

                        PathPlannerPath path = PathPlannerPath.fromPathFile(nome);

                        // ---- DIAGNOSTICO (temporario) ----
                        try {
                                com.pathplanner.lib.config.RobotConfig cfgDebug = com.pathplanner.lib.config.RobotConfig
                                                .fromGUISettings();

                                System.out.println(">>> DEBUG " + nome + " | holonomico: " + cfgDebug.isHolonomic);
                                System.out.println(">>> DEBUG " + nome + " | massa: " + cfgDebug.massKG);
                                System.out.println(">>> DEBUG " + nome + " | vel max modulo (m/s): "
                                                + cfgDebug.moduleConfig.maxDriveVelocityMPS);

                                var trajDebug = path.getIdealTrajectory(cfgDebug);
                                System.out.println(">>> DEBUG " + nome + " | tempo total (s): "
                                                + trajDebug.map(t -> t.getTotalTimeSeconds()).orElse(-1.0));

                                var pts = path.getAllPathPoints();
                                System.out.println(">>> DEBUG " + nome + " | pontos: " + pts.size());
                                System.out.println(">>> DEBUG " + nome + " | inicio: " + pts.get(0).position);
                                System.out.println(">>> DEBUG " + nome + " | fim: "
                                                + pts.get(pts.size() - 1).position);
                        } catch (Exception e) {
                                e.printStackTrace();
                        }
                        // ---- FIM DIAGNOSTICO ----

                        Command pathCommand = AutoBuilder.followPath(path);

                        return Commands.sequence(

                                        Commands.runOnce(() -> System.out.println(
                                                        ">>> INICIO DO PATH: " + nome
                                                                        + " | TEMPO: " + Timer.getFPGATimestamp())),

                                        pathCommand.finallyDo(interrupted -> System.out.println(
                                                        ">>> FIM DO PATH: " + nome
                                                                        + " | TEMPO: " + Timer.getFPGATimestamp()
                                                                        + " | Interrompido: " + interrupted)));

                } catch (Exception e) {

                        throw new IllegalStateException(
                                        "Erro ao carregar o Path: " + nome, e);
                }
        }

        // =========================================================
        // AUTONOMO PRINCIPAL - UMA ALGAE
        // =========================================================

        public static Command autoUmaAlgae(
                        TankSubsystem tank,
                        SimIntake intake,
                        SimElevator elevator,
                        SimHand hand) {

                return Commands.sequence(

                                Commands.runOnce(() -> System.out.println(
                                                "================================")),

                                Commands.runOnce(() -> System.out.println(
                                                "===== AUTONOMO UMA ALGAE =====")),

                                // 1. Reinicia a pose do robô
                                tank.CMDresetPose(
                                                new Pose2d(7.2, 1.0, new Rotation2d())),

                                Commands.runOnce(() -> System.out.println(
                                                ">>> POSE INICIAL: " + tank.getPose())),

                                // 2. Vai até a Algae
                                seguirPath("1ofc"),

                                Commands.runOnce(() -> System.out.println(
                                                ">>> POSE DEPOIS DO PATH 1: "
                                                                + tank.getPose())),

                                // 3. Coleta e transfere para a Hand
                                coletarAlgae(tank, intake, elevator, hand),

                                // 4. Verifica se a Hand recebeu a Algae
                                Commands.either(

                                                Commands.sequence(

                                                                Commands.runOnce(() -> System.out.println(
                                                                                ">>> HAND CONFIRMADA. CONTINUANDO AUTONOMO")),

                                                                // 5. Vai até a Net
                                                                seguirPath("2ofc"),

                                                                // 6. Deposita a Algae
                                                                depositGamePiece(tank, hand),

                                                                // 7. Retorna ao início
                                                                seguirPath("3 final"),

                                                                Commands.runOnce(() -> System.out.println(
                                                                                "===== AUTONOMO FINALIZADO ====="))),

                                                Commands.sequence(

                                                                Commands.runOnce(() -> System.out.println(
                                                                                ">>> AUTONOMO INTERROMPIDO: HAND SEM ALGAE"))),

                                                tank::isHandCollected));
        }

        public static Command coletarAlgaeEmParalelo(TankSubsystem tank, SimIntake mSimIntake) {
                // TODO Auto-generated method stub
                throw new UnsupportedOperationException("Unimplemented method 'coletarAlgaeEmParalelo'");
        }
}
