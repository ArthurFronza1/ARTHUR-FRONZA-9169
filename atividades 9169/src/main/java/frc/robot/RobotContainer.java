
package frc.robot;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.subsystems.SimElevator;
import frc.robot.subsystems.SimHand;
import frc.robot.subsystems.SimIntake;

public class RobotContainer {

        private final TankSubsystem tank = new TankSubsystem();

        private final CommandXboxController driver = new CommandXboxController(0);

        private final XboxController driverget = new XboxController(0);

        private final SimIntake mSimIntake = new SimIntake(20, 1.5, 3);

        private final SimElevator mSimElevator = new SimElevator(400, 1.5);

        private final SimHand mSimHand = new SimHand(-180, 180, 1.5, 3);

        private final SendableChooser<Command> autoChooser = new SendableChooser<>();

        public RobotContainer() {

                // =========================================
                // AUTONÔMICO
                // =========================================

                NamedCommands.registerCommand("coleta", Autos.coletarAlgae(tank, mSimIntake, mSimElevator, mSimHand));
                NamedCommands.registerCommand("Cospe Alga",Autos.depositGamePiece(tank, mSimHand));
                NamedCommands.registerCommand("transfere alga",Autos.transferirParaHand(tank, mSimElevator,mSimHand));
                NamedCommands.registerCommand("resetar hand",Autos.resetangleCommand(tank, mSimElevator,mSimHand));
                autoChooser.addOption(
                                "Uma Algae",
                                Autos.autoUmaAlgae(
                                                tank,
                                                mSimIntake,
                                                mSimElevator,
                                                mSimHand));
                autoChooser.addOption(
                                "Quadrado",
                                Autos.quadrado(tank));

                autoChooser.addOption(
                                "Reto",
                                Autos.reto(tank));

                autoChooser.addOption(
                                "ZigZag",
                                Autos.autoZigZag(
                                                tank,
                                                mSimIntake,
                                                mSimElevator,
                                                mSimHand));

                autoChooser.addOption("Rota teste", new PathPlannerAuto("New Auto"));

                SmartDashboard.putData(
                                "Auto Selector",
                                autoChooser);

                // =========================================
                // CONTROLES
                // =========================================

                configureBindings();

                // =========================================
                // DRIVE DEFAULT
                // =========================================

                tank.setDefaultCommand(
                                tank.run(() -> {

                                        double leftX = driver.getLeftX();
                                        double leftY = driver.getLeftY();

                                        double joystickMagnitude = Math.hypot(leftX, leftY);

                                        if (joystickMagnitude > 0.08) {

                                                tank.driveWithAngleControl(
                                                                getLeftJoystickAngle(),
                                                                driver.getRawAxis(3));

                                        } else {

                                                double forward = driver.getRawAxis(3);

                                                double backward = driver.getRawAxis(2);

                                                double rotation = driver.getRightX();

                                                tank.drive(
                                                                forward,
                                                                -backward,
                                                                -rotation);
                                        }
                                }));
        }

        // =========================================
        // CONFIGURAÇÃO DOS BOTÕES
        // =========================================

        private void configureBindings() {

                // Reset manual da pose
                driver.start().onTrue(
                                tank.runOnce(() -> tank.resetPose(
                                                new Pose2d(
                                                                2.0,
                                                                4.0,
                                                                new Rotation2d()))));

                // Elevador
                driver.povUp().onTrue(
                                mSimElevator.CMDsetPosition(400, 10));

                driver.povDown().onTrue(
                                mSimElevator.CMDsetPosition(0, 3));

                // Ângulo do Hand
                driver.povLeft().onTrue(
                                Commands.runOnce(
                                                () -> mSimHand.setAngle(-45, 3)));

                driver.povRight().onTrue(
                                Commands.runOnce(
                                                () -> mSimHand.setAngle(45, 3)));

                driver.a().onTrue(
                                Commands.runOnce(
                                                () -> mSimHand.setAngle(135, 3)));

                driver.b().onTrue(
                                Commands.runOnce(
                                                () -> mSimHand.setAngle(-135, 3)));

                driver.back().onTrue(
                                Commands.runOnce(
                                                () -> mSimHand.setAngle(0, 3)));

                // Coleta manual
                coletar_Ground();

                // Transferência automática no teleoperado
                autoHand();

                // Depositar
                driver.y().onTrue(deposit());
        }

        // =========================================
        // INTAKE MANUAL                        
        // =========================================

        private void coletar_Ground() {

                driver.leftBumper().whileTrue(
                                mSimIntake.CMDintakeOn()
                                                .onlyIf(() -> !tank.isFuelInsideRobot()
                                                                && SimElevator.getRealPosition() >= 200));

                driver.leftBumper().whileFalse(
                                mSimIntake.CMDintakeOff());
        }

        // =========================================
        // TRANSFERÊNCIA AUTOMÁTICA
        // =========================================

        private void autoHand() {

                BooleanSupplier autoHandOk = () -> !DriverStation.isAutonomous()
                                && tank.isFuelInsideRobot()
                                && !tank.isHandCollected();

                onCommandCondition(
                                autoHandOk,
                                Autos.transferirParaHand(
                                                tank,
                                                mSimElevator,
                                                mSimHand));
        }

        // =========================================
        // COMANDO AUTÔNOMO SELECIONADO
        // =========================================

        public Command getAutonomousCommand() {

                System.out.println(">>> AUTONOMO UMA ALGAE SELECIONADO!");

                return autoChooser.getSelected();
                /*Autos.autoUmaAlgae(
                                tank,
                                mSimIntake,
                                mSimElevator,
                                mSimHand);*/
        }
        // =========================================
        // ÂNGULO DO JOYSTICK
        // =========================================

        public double getLeftJoystickAngle() {

                double x = -driverget.getLeftX();
                double y = -driverget.getLeftY();

                if (Math.hypot(x, y) < 0.1) {
                        return 0.0;
                }

                return Math.toDegrees(
                                Math.atan2(x, y));
        }

        // =========================================
        // CONDIÇÕES DE COMANDOS
        // =========================================

        private void onCommandCondition(
                        BooleanSupplier condition,
                        Command command) {

                new Trigger(condition).onTrue(command);
        }

        private void whileCommandCondition(
                        BooleanSupplier condition,
                        Command command) {

                new Trigger(condition).whileTrue(command);
        }

        // =========================================
        // DEPOSITAR MANUALMENTE
        // =========================================

        private Command deposit() {

                return Commands.sequence(

                                Commands.runOnce(
                                                () -> mSimHand.setVelocity(1)),

                                tank.depositFuelCommand(),

                                Commands.waitSeconds(1),

                                Commands.runOnce(
                                                () -> mSimHand.setVelocity(0)));
        }
}
