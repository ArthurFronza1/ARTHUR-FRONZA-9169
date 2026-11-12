package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.TankdriveSub;

public class DriveDistanceCMD extends Command {

    private final TankdriveSub drive;
    private final double distanciaDesejada;

    private final PIDController pid =
            new PIDController(0.5, 0.0, 0.0);

    private final Timer timer = new Timer();

    public DriveDistanceCMD(
            TankdriveSub drive,
            double distanciaMetros) {

        this.drive = drive;
        this.distanciaDesejada = distanciaMetros;

        addRequirements(drive);

        pid.setTolerance(0.03);
    }

    @Override
    public void initialize() {
        drive.stopMotor();
        drive.resetEncoders();
        pid.reset();
        timer.restart();
    }

    @Override
    public void execute() {

        double distanciaAtual = drive.getDistanceMeters();

        double output = pid.calculate(
                distanciaAtual,
                distanciaDesejada);

        output = MathUtil.clamp(output, -0.5, 0.5);

        drive.driveDistance(output);
    }

    @Override
    public boolean isFinished() {
        return pid.atSetpoint()
                || timer.hasElapsed(5.0);
    }

    @Override
    public void end(boolean interrupted) {
        drive.stopMotor();
        timer.stop();
    }
}