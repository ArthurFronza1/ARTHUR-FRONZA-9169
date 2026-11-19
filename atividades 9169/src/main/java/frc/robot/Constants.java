
package frc.robot;

public final class Constants {

    private Constants() {
    }

    public static final class OperatorConstants {

        public static final int kDriverControllerPort = 0;

        private OperatorConstants() {
        }
    }

    public static final class Elevador {

        public static final double ALTURA_COLETA = 0.0;
        public static final double ALTURA_LANCAMENTO = 0.5;

        public static final double TOLERANCIA = 0.01;

        public static final double VELOCIDADE_MAX = 0.3;

        private Elevador() {
        }
    }

    public static final class Manipulador {

        public static final double ANGULO_INICIAL = 70.0;

        public static final double ANGULO_MIN = 0.0;
        public static final double ANGULO_MAX = 180.0;

        public static final double ANGULO_LANCAMENTO = 45.0;

        public static final double VELOCIDADE_MAX = 90.0;

        public static final double TOLERANCIA = 1.0;

        private Manipulador() {
        }
    }

    public static final class Simulacao {

        public static final double PERIOD = 0.02;

        private Simulacao() {
        }
    }
}
