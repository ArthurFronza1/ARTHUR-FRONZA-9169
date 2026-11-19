
package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;

public final class GamePieceConstants {

        // Posições das três Algas no campo Reefscape (metros)
        public static final Translation2d GAME_PIECE_1 = new Translation2d(1.223, 2.200);

        public static final Translation2d GAME_PIECE_2 = new Translation2d(1.223, 4.023);

        public static final Translation2d GAME_PIECE_3 = new Translation2d(1.223, 5.858);

        // Distância máxima para o Intake conseguir coletar
        public static final double DISTANCIA_COLETA = 0.55;

        private GamePieceConstants() {
        }
}
