package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;

import frc.robot.subsystems.GamePieceConstants;
import frc.robot.subsystems.intakeSub;

public class GamePieceSim {

    private final Supplier<Pose2d> getRobotPose;
    private final intakeSub intake;

    private final List<Translation2d> gamePieces = new ArrayList<>();

    private final StructArrayPublisher<Pose3d> gamePiecesPublisher =
            NetworkTableInstance.getDefault()
                    .getStructArrayTopic("GamePieces/Poses", Pose3d.struct)
                    .publish();

    public GamePieceSim(Supplier<Pose2d> getRobotPose, intakeSub intake) {
        this.getRobotPose = getRobotPose;
        this.intake = intake;

        reset();
    }

    public void reset() {
        gamePieces.clear();

        gamePieces.add(GamePieceConstants.GAME_PIECE_1);
        gamePieces.add(GamePieceConstants.GAME_PIECE_2);
        gamePieces.add(GamePieceConstants.GAME_PIECE_3);

        publicarGamePieces();
    }

    public boolean tentarColetar() {

        if (intake.temAlgaeNoCentro() || intake.temAlgaeNoManipulador()) {
            return false;
        }

        Pose2d poseRobo = getRobotPose.get();

        int indiceMaisProximo = -1;
        double menorDistancia = Double.MAX_VALUE;

        for (int i = 0; i < gamePieces.size(); i++) {

            double distancia = poseRobo.getTranslation()
                    .getDistance(gamePieces.get(i));

            if (distancia < menorDistancia) {
                menorDistancia = distancia;
                indiceMaisProximo = i;
            }
        }

        if (indiceMaisProximo >= 0
                && menorDistancia <= GamePieceConstants.DISTANCIA_COLETA) {

            gamePieces.remove(indiceMaisProximo);

            publicarGamePieces();

            return true;
        }

        return false;
    }

    private void publicarGamePieces() {

        Pose3d[] poses = new Pose3d[gamePieces.size()];

        for (int i = 0; i < gamePieces.size(); i++) {

            Translation2d posicao = gamePieces.get(i);

            poses[i] = new Pose3d(
                    new Translation3d(
                            posicao.getX(),
                            posicao.getY(),
                            0.0),
                    new Rotation3d());
        }

        gamePiecesPublisher.set(poses);
    }
}