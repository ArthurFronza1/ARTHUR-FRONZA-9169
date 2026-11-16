
package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class drivetrainSub extends SubsystemBase {

    // Limites da simulação
    private static final double VELOCIDADE_MAX = 2.0;

    private static final double VELOCIDADE_GIRO_MAX = 3.0;

    private static final double VELOCIDADE_GIRO_TELEOP = 0.3;

    private static final double PERIOD = 0.02;

    private static final double DEADBAND = 0.08;

    // Pose de destino do Desafio 2
    public static final Pose2d POSE_DESTINO =
            new Pose2d(
                    7.8,
                    4.0,
                    Rotation2d.fromDegrees(60.0));

    // Restrições do PathPlanner
    private static final PathConstraints LIMITES =
            new PathConstraints(
                    1.8,
                    2.0,
                    Units.degreesToRadians(180.0),
                    Units.degreesToRadians(360.0));

    // Pose inicial
    private Pose2d pose =
            new Pose2d(2.0, 2.0, new Rotation2d());

    // Velocidades atuais
    private double velocidadeFrente = 0.0;

    private double velocidadeGiro = 0.0;

    private final Field2d field = new Field2d();

    // Publicação da pose no NetworkTables
    private final StructPublisher<Pose2d> posePub =
            NetworkTableInstance.getDefault()
                    .getStructTopic(
                            "Drivetrain/Pose",
                            Pose2d.struct)
                    .publish();

    public drivetrainSub() {

        field.setRobotPose(pose);

        SmartDashboard.putData(
                "Drivetrain/Field",
                field);

        PathPlannerLogging.setLogActivePathCallback(
                poses -> field.getObject("path").setPoses(poses));

        PathPlannerLogging.setLogTargetPoseCallback(
                alvo -> field.getObject("alvo").setPose(alvo));

        configurarAutoBuilder();
    }

    // Configuração do PathPlanner
    private void configurarAutoBuilder() {

        try {

            RobotConfig config =
                    RobotConfig.fromGUISettings();

            AutoBuilder.configure(
                    this::getPose,
                    this::resetPose,
                    this::getRobotRelativeSpeeds,

                    (speeds, feedforwards) ->
                            driveRobotRelative(speeds),

                    new PPLTVController(PERIOD),

                    config,

                    () -> false,

                    this);

        } catch (Exception e) {

            throw new IllegalStateException(
                    "Erro ao carregar a configuração do robô no PathPlanner.",
                    e);
        }
    }

    // Controle manual
    public void drive(double frente, double giro) {

        velocidadeFrente =
                MathUtil.applyDeadband(
                        frente,
                        DEADBAND) * VELOCIDADE_MAX;

        velocidadeGiro =
                MathUtil.applyDeadband(
                        giro,
                        DEADBAND) * VELOCIDADE_GIRO_TELEOP;
    }

    // Retorna a pose atual
    public Pose2d getPose() {
        return pose;
    }

    // Reinicia a pose
    public void resetPose(Pose2d novaPose) {

        pose = novaPose;

        field.setRobotPose(pose);

        posePub.set(pose);
    }

    // Velocidades relativas ao robô
    public ChassisSpeeds getRobotRelativeSpeeds() {

        return new ChassisSpeeds(
                velocidadeFrente,
                0.0,
                velocidadeGiro);
    }

    // Recebe velocidades do PathPlanner
    public void driveRobotRelative(ChassisSpeeds speeds) {

        velocidadeFrente = MathUtil.clamp(
                speeds.vxMetersPerSecond,
                -VELOCIDADE_MAX,
                VELOCIDADE_MAX);

        velocidadeGiro = MathUtil.clamp(
                speeds.omegaRadiansPerSecond,
                -VELOCIDADE_GIRO_MAX,
                VELOCIDADE_GIRO_MAX);
    }

    // Navegação até uma pose
    public Command irParaPose(Pose2d destino) {

        return AutoBuilder.pathfindToPose(
                destino,
                LIMITES,
                0.0)
                .andThen(
                        girarParaAngulo(destino.getRotation()));
    }

    // Controle de orientação
    public Command girarParaAngulo(Rotation2d alvo) {

        @SuppressWarnings("resource")
        PIDController pid =
                new PIDController(4.0, 0.0, 0.0);

        pid.enableContinuousInput(-Math.PI, Math.PI);

        pid.setTolerance(Math.toRadians(1.5));

        return run(() -> {

            double saida = pid.calculate(
                    pose.getRotation().getRadians(),
                    alvo.getRadians());

            driveRobotRelative(
                    new ChassisSpeeds(
                            0.0,
                            0.0,
                            saida));

        })
        .beforeStarting(pid::reset)
        .until(pid::atSetpoint)
        .finallyDo(interrompido ->
                driveRobotRelative(new ChassisSpeeds()));
    }

    // Atualização da simulação
    @Override
    public void periodic() {

        pose = pose.exp(
                new Twist2d(
                        velocidadeFrente * PERIOD,
                        0.0,
                        velocidadeGiro * PERIOD));

        field.setRobotPose(pose);

        posePub.set(pose);

        SmartDashboard.putNumber(
                "Drivetrain/X",
                pose.getX());

        SmartDashboard.putNumber(
                "Drivetrain/Y",
                pose.getY());

        SmartDashboard.putNumber(
                "Drivetrain/Angulo",
                pose.getRotation().getDegrees());
    }
}
