package first.robot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.List;

import org.wpilib.framework.RobotBase; // RobotBase throws an error for some reason ill have to look into it
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.util.Units;

import com.ctre.phoenix6.CANBus;

import first.robot.subsystems.endEffector.EEConstants.RollerStates;
import first.robot.subsystems.endEffector.EEConstants.WristStates;
import first.robot.subsystems.launcher.Launcher;
import first.robot.subsystems.launcher.LauncherConstants.LauncherStates;
import first.robot.subsystems.telescope.TelescopeConstants.TelescopeStates;

public class Constants {

    public enum Mode {
        REAL,
        SIM,
        REPLAY
    }

    public static final Mode simMode = Mode.SIM;

    // public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
    public static final Mode currentMode = simMode;

    public static final CANBus SUPERSTRUCTURE_CAN_BUS = CANBus.systemcore(4);
    public static final double LOOP_FREQ_HZ = 50;
    public static final double LOOP_PERIOD_SEC = 1 / LOOP_FREQ_HZ;

    public static enum SuperstructureStates {
        HOME(TelescopeStates.HOME, WristStates.STOWED, RollerStates.IDLE),
        INTAKING(TelescopeStates.DOWN, WristStates.DEPLOYED, RollerStates.FAST_FWD),
        OUTTAKING(TelescopeStates.DOWN, WristStates.DEPLOYED, RollerStates.FAST_REV),
        L1_FRONT(TelescopeStates.L1_FRONT, WristStates.L1_FRONT),
        L1_BACK(TelescopeStates.L1_BACK, WristStates.L1_BACK),
        L2_FRONT(TelescopeStates.L2_FRONT, WristStates.L2_FRONT),
        L2_BACK(TelescopeStates.L2_BACK, WristStates.L2_BACK),
        CLASSIFIER_FRONT(TelescopeStates.CLASSIFIER_FRONT, WristStates.CLASSIFIER_FRONT),
        CLASSIFIER_BACK(TelescopeStates.CLASSIFIER_BACK, WristStates.CLASSIFIER_BACK),
        LAUNCHER(TelescopeStates.LAUNCHER, WristStates.STOWED, RollerStates.IDLE, true),
        CLIMB_RAISED(TelescopeStates.CLIMB_RAISED, WristStates.DEPLOYED),
        CLUMB(TelescopeStates.CLUMB, WristStates.DEPLOYED);

        public final TelescopeStates telescopeState;
        public final WristStates wristState;
        public final RollerStates rollerState;
        public final boolean usesLauncher;

        private SuperstructureStates(TelescopeStates telescopeState, WristStates wristState, RollerStates rollerState, boolean usesLauncher) {
            this.telescopeState = telescopeState;
            this.wristState = wristState;
            this.rollerState = rollerState;
            this.usesLauncher = usesLauncher;
        }
        private SuperstructureStates(TelescopeStates telescopeState, WristStates wristState, RollerStates rollerState) {
            this(telescopeState, wristState, rollerState, false);
        }
        private SuperstructureStates(TelescopeStates telescopeState, WristStates wristState) {
            this(telescopeState, wristState, RollerStates.IDLE, false);
        }
    }


    /** Constants for the field. :scream: */
    public static class FieldConstants {

        private static final Translation2d flipX(Translation2d translation) {
            return new Translation2d(-translation.getX(), translation.getY());
        }

        private static final Pose2d flipX(Pose2d pose) {
            return new Pose2d(-pose.getX(), pose.getY(), pose.getRotation().unaryMinus());
        }

        public static enum LShaft_ClockwiseColorOrder {
            PURPLE, YELLOW, GREEN, ORANGE
        }

        // OVERALL CONSTANTS
        public static final double FIELD_LENGTH = Units.inchesToMeters(648);
        public static final double FIELD_WIDTH = Units.inchesToMeters(324);
        public static final Transform2d LSHAFT_WALL_DISTANCE = new Transform2d(new Translation2d(Units.inchesToMeters(10), Rotation2d.kZero), Rotation2d.kZero);
        public static final Transform2d USHAFT_WALL_DISTANCE = new Transform2d(new Translation2d(Units.inchesToMeters(31.25), Rotation2d.kZero), Rotation2d.kZero);
        
        public static final double MINE_OUTER_WIDTH = Units.inchesToMeters(50.75);
        public static final double MINE_PILLAR_LENGTH = Units.inchesToMeters(12.0);

        public static final Translation2d CENTER = new Translation2d(FIELD_LENGTH/2., FIELD_WIDTH/2.);
        // +x = right, -x = left
        // +y = up, -y = down

        /** Constants for the blue side of the field. */
        public static class BlueFieldConstants {
            // CAVE
            public static final Translation2d CAVE_CENTER = CENTER.plus(new Translation2d(Units.inchesToMeters(165.747218), 0.));
            public static final Pose2d[] LOWER_SHAFTS = new Pose2d[8];
            static {
                for (int i = 0; i < LOWER_SHAFTS.length; i++) {
                    Rotation2d angle = new Rotation2d(Units.degreesToRadians(45 * i + 22.5));
                    LOWER_SHAFTS[i] = new Pose2d(CAVE_CENTER.plus(new Translation2d(Units.inchesToMeters(32.50), angle)), angle);
                }
            }
            public static final Pose2d[] UPPER_SHAFTS = new Pose2d[4];
            static {
                for (int i = 0; i < UPPER_SHAFTS.length; i++) {
                    Rotation2d angle = new Rotation2d(Units.degreesToRadians(90 * i + 45));
                    UPPER_SHAFTS[i] = new Pose2d(CAVE_CENTER.plus(new Translation2d(Units.inchesToMeters(12.176912), angle)), angle);
                }
            }

            // CLASSIFIER
            public static final Translation2d CLASSIFIER_SOURCE_CORNER = CENTER.plus(new Translation2d(Units.inchesToMeters(324.797850), -Units.inchesToMeters(25.777159)));
            public static final Translation2d CLASSIFIER_MINE_CORNER = CLASSIFIER_SOURCE_CORNER.minus(new Translation2d(0, Units.inchesToMeters(62)));
            public static final Translation2d CLASSIFIER_CENTER = CLASSIFIER_SOURCE_CORNER.plus(CLASSIFIER_SOURCE_CORNER.minus(CLASSIFIER_MINE_CORNER).div(2));
            public static final Translation2d CLASSIFIER_AIM_TARGET = CAVE_CENTER.minus(new Translation2d(2, 0));

            // STATION
            public static final Translation2d SOURCE_SOURCE_CORNER = CENTER.plus(new Translation2d(-Units.inchesToMeters(255.446748), Units.inchesToMeters(159.593244)));
            public static final Translation2d SOURCE_MINE_CORNER = CENTER.plus(new Translation2d(-Units.inchesToMeters(324.526445), Units.inchesToMeters(119.742799)));
            public static final Translation2d SOURCE_CENTER = SOURCE_SOURCE_CORNER.plus(SOURCE_SOURCE_CORNER.minus(SOURCE_MINE_CORNER).div(2));

            // MINE
            public static final Translation2d MINE_CENTER_CORNER = CENTER.plus(new Translation2d(Units.inchesToMeters(125.785719), Units.inchesToMeters(107.027159)));
            public static final Translation2d MINE_CLASSIFIER_CORNER = CENTER.plus(new Translation2d(Units.inchesToMeters(268.535719), Units.inchesToMeters(107.027159)));
            public static final Translation2d MINE_CENTER = MINE_CENTER_CORNER.plus(new Translation2d(MINE_CENTER_CORNER.minus(MINE_CLASSIFIER_CORNER).div(2).getX(), MINE_PILLAR_LENGTH + (MINE_OUTER_WIDTH / 2.)));
        }

        /** Constants for the red side of the field.
         * <p> NOTE: this essentially takes the blue measurements and flips them across the Y axis.
         */
        public static class RedFieldConstants {
            // CAVE
            public static final Translation2d CAVE_CENTER = flipX(BlueFieldConstants.CAVE_CENTER);
            public static final Pose2d[] LOWER_SHAFTS = new Pose2d[BlueFieldConstants.LOWER_SHAFTS.length];
            static {
                for(int i = 0; i < LOWER_SHAFTS.length; i++) {
                    LOWER_SHAFTS[i] = flipX(BlueFieldConstants.LOWER_SHAFTS[LOWER_SHAFTS.length-1-i]); // doing so to preserve the clockwise color order
                }
            }
            public static final Pose2d[] UPPER_SHAFTS = new Pose2d[BlueFieldConstants.UPPER_SHAFTS.length];
            static {
                for(int i = 0; i < UPPER_SHAFTS.length; i++) {
                    UPPER_SHAFTS[i] = flipX(BlueFieldConstants.UPPER_SHAFTS[UPPER_SHAFTS.length-1-i]);
                }
            }

            // CLASSIFIER
            public static final Translation2d CLASSIFIER_SOURCE_CORNER = flipX(BlueFieldConstants.CLASSIFIER_SOURCE_CORNER);
            public static final Translation2d CLASSIFIER_MINE_CORNER = flipX(BlueFieldConstants.CLASSIFIER_MINE_CORNER);
            public static final Translation2d CLASSIFIER_CENTER = flipX(BlueFieldConstants.CLASSIFIER_CENTER);
            public static final Translation2d CLASSIFIER_AIM_TARGET = flipX(BlueFieldConstants.CLASSIFIER_AIM_TARGET);

            // STATION
            public static final Translation2d SOURCE_SOURCE_CORNER = flipX(BlueFieldConstants.SOURCE_SOURCE_CORNER);
            public static final Translation2d SOURCE_MINE_CORNER = flipX(BlueFieldConstants.SOURCE_MINE_CORNER);
            public static final Translation2d SOURCE_CENTER = flipX(BlueFieldConstants.SOURCE_CENTER);

            // MINE
            public static final Translation2d MINE_CENTER_CORNER = flipX(BlueFieldConstants.MINE_CENTER_CORNER);
            public static final Translation2d MINE_CLASSIFIER_CORNER = flipX(BlueFieldConstants.MINE_CLASSIFIER_CORNER);
            public static final Translation2d MINE_CENTER = flipX(BlueFieldConstants.MINE_CENTER);
        
        }
    }
}
