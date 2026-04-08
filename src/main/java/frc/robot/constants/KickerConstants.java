package frc.robot.constants;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

public class KickerConstants {
    public static final int UPPER_MOTOR_ID = 15;
    public static final int LOWER_MOTOR_ID = 16;
    public static final double STATOR_CURRENT_LIMIT = 85;
    public static final double SUPPLY_CURRENT_LIMIT = 60;
    public static final boolean LOWER_INVERTED = true;
    public static final boolean UPPER_INVERTED = false;
    public static final boolean UPPER_BRAKE = false;

    public static final double INDEXING_SPEED = 25; //25
    public static final double SHUTTLE_INDEXING_SPEED = 90;

    public static final Slot0Configs UPPER_SLOT0_CONFIGS = new Slot0Configs()
            .withKP(999999)
            .withKI(0)
            .withKD(0)
            .withKS(.40156)
            .withKG(0)
            .withKV(.12621)
            .withKA(0.0056143)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);


    public static final double ACCELERATION = 400;
    public static final double JERK = 4000;

    public static final double LOWER_TOLERANCE = 5d;
    public static final double UPPER_TOLERANCE = 5d;

    public static final double[][] GEAR_RATIO = {{1, 1}};
    public static final double SCALE_FACTOR = (1d);

    public static final CANBus canbus = new CANBus("rio");
}
