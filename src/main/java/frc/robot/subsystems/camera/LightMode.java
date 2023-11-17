package frc.robot.subsystems.camera;

// https://github.com/NAHSRobotics-Team5667/2020-FRC/blob/master/src/main/java/frc/robot/utils/LimeLight.java
public enum LightMode {
    DEFAULT(0),
    OFF(1),
    BLINK(2),
    ON(3);

    private final int ledMode;

    private LightMode(int ledMode) {
        this.ledMode = ledMode;
    }

    public int getLedMode() {
        return ledMode;
    }

    public enum CamMode {
        FRONT(0),
        BACK(1);
    
        private final int mode;
    
        private CamMode(int mode) {
            this.mode = mode;
        }
    
        public int getMode() {
            return mode;
        }
    }

    public enum SnapMode {
        DISABLED(0),
        ENABLED(1);
    
        private final int mode;
    
        private SnapMode(int mode) {
            this.mode = mode;
        }
    
        public int getMode() {
            return mode;
        }
    }

    public enum StreamMode {
        STANDARD(0),
        MAIN(1),
        SECONDARY(2);
    
        private final int mode;
    
        private StreamMode(int mode) {
            this.mode = mode;
        }
    
        public int getMode() {
            return mode;
        }
    }

}
