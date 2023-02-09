package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public final class Constants {
    public static class Turret {
        public static final double MANUAL_ROTATE_SCALE = 0.0001;
        public static final double GEAR_RATIO = 37.2; // Possibly subject to change

        public static final double SOFT_MIN_ANGLE = -GEAR_RATIO * 110 / 360; // Not sure of what the angle is
        public static final double SOFT_MAX_ANGLE = GEAR_RATIO * 110 / 360;
        public static final double HARD_MIN_ANGLE = -GEAR_RATIO * 130 / 360;
        public static final double HARD_MAX_ANGLE = GEAR_RATIO * 130 / 360;

        public static final double P = 0;
        public static final double I = 0;
        public static final double D = 0;
        public static final double MAX_VELOCITY = 1; // Rotations per second of turret (not motor)
        public static final double MAX_ACCELERATION = 1; // Rotations per second per second of turret
        public static final double BOUNDARY_SCALE = 2/3; // Shrink voltage when near boundary
    }

    public static class Vision {
        public static final Transform3d CAMERA_TO_ROBOT =
            new Transform3d();
        public static final Transform3d CAMERA_TO_TURRET = 
            new Transform3d(); // turret pose is angle 0
        public static final Transform3d TAG_TO_CUBE =
            new Transform3d();
        // Note this may be different depending on which april tag. Will be implemented in future
        public static final Transform3d TAG_TO_LEFT_CONE_NODE =
            new Transform3d(new Translation3d(0, 0, 0), new Rotation3d()); 
        public static final Transform3d TAG_TO_RIGHT_CONE_NODE =
            new Transform3d(new Translation3d(0, 0, 0), new Rotation3d());
    }
}
