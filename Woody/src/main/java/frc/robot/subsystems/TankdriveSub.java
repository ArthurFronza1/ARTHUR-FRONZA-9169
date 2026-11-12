package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.OperatorConstants;

public class TankdriveSub extends SubsystemBase {

    public final SparkMax SparkRlider;
    public final SparkMax SparkRfollow;
    public final SparkMax SparkLlider;
    public final SparkMax SparkLfollow;

    private final DifferentialDrive drive;

    private final RelativeEncoder encoderDireito;
    private final RelativeEncoder encoderEsquerdo;

    // Diametro da roda: 6 polegadas convertidas para metros
    private static final double DIAMETRO_RODA = 0.1524;
    // valor da alteração 8:45
    private static final double RELACAO_TRANSMISSAO = 5.625;

    private static final double METROS_POR_ROTACAO = Math.PI * DIAMETRO_RODA / RELACAO_TRANSMISSAO;

    public TankdriveSub() {

        SparkRlider = new SparkMax(
                OperatorConstants.IDSparkRlider,
                MotorType.kBrushless);

        SparkRfollow = new SparkMax(
                OperatorConstants.IDSparkRfollow,
                MotorType.kBrushless);

        SparkLlider = new SparkMax(
                OperatorConstants.IDSparkLlider,
                MotorType.kBrushless);

        SparkLfollow = new SparkMax(
                OperatorConstants.IDSparkLfollow,
                MotorType.kBrushless);

        drive = new DifferentialDrive(
                SparkLlider,
                SparkRlider);

        encoderDireito = SparkRlider.getEncoder();
        encoderEsquerdo = SparkLlider.getEncoder();
    }

    // Controle manual existente
    public void arcadeDrive(double speed, double rotation) {
        drive.arcadeDrive(speed, rotation);
    }

    // Controle de distância: não aplica o quadrado dos valores
    public void driveDistance(double output) {
        drive.arcadeDrive(output, 0, false);
    }

    public void stopMotor() {
        drive.stopMotor();
    }

    public void resetEncoders() {
        encoderDireito.setPosition(0);
        encoderEsquerdo.setPosition(0);
    }

    public double getDistanceMeters() {

        double esquerda = encoderEsquerdo.getPosition() * METROS_POR_ROTACAO;

        // Sinal a confirmar durante o teste
        double direita = -encoderDireito.getPosition() * METROS_POR_ROTACAO;

        return (esquerda + direita) / 2.0;
    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber(
                "Drive/EncoderEsquerdo",
                encoderEsquerdo.getPosition());

        SmartDashboard.putNumber(
                "Drive/EncoderDireito",
                encoderDireito.getPosition());

        SmartDashboard.putNumber(
                "Drive/DistanciaMetros",
                getDistanceMeters());
    }
}