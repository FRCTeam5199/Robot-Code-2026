package frc.robot.constants;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

public class IntakeRollerConstants {
    public static final int MOTOR_ID = 15;
    public static final double STATOR_CURRENT_LIMIT = 60;
    public static final double SUPPLY_CURRENT_LIMIT = 60;
    public static final boolean INVERTED = true;
    public static final boolean BRAKE = true;

    public static final Slot0Configs SLOT0_CONFIGS = new Slot0Configs()
            .withKP(0.25)
            .withKI(0)
            .withKD(0)
            .withKS(0.9)
            .withKG(0)
            .withKV(0.08787346221441124780316344463972)
            .withKA(0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);

    public static final double ACCELERATION = 400;
    public static final double JERK = 4000;

    public static final double LOWER_TOLERANCE = 2;
    public static final double UPPER_TOLERANCE = 2;

    public static final double[][] GEAR_RATIO = {{1, 1}};

    public static final double INTAKE_SPEED = 90;
}
