package frc.robot.constants;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

public class TurretConstants {
    public static final int MOTOR_ID = 21;
    public static final int ENCODER_ID = 23;

    public static final double STATOR_CURRENT_LIMIT = 60;
    public static final double SUPPLY_CURRENT_LIMIT = 60;

    public static final boolean INVERTED = false;
    public static final boolean BRAKE = true;

    public static final Slot0Configs SLOT0_CONFIGS = new Slot0Configs()
            .withKP(5) //5
            .withKI(0)
            .withKD(.2) //.2
            .withKS(0.54255) //.35
            .withKG(0)
            .withKV(0.091075) //.098522167487684729064039408867
            .withKA(0.0034531) //0
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);

    public static final double VELOCITY = 200;
    public static final double ACCELERATION = 400;
    public static final double JERK = 3000;

    public static final double LOWER_TOLERANCE = 5;
    public static final double UPPER_TOLERANCE = 5;

    public static final double[][] GEAR_RATIO = {{50, 12}, {100, 10}};

    public static final double ENCODER_MAGNET_OFFSET = -0.260986328125;
    public static final double SENSOR_TO_MECH_GEAR_RATIO = 100d / 10d;
    public static final double MOTOR_TO_SENSOR_GEAR_RATIO = 50d / 12d;
    public static final boolean CCW_POSITIVE = false;
    public static final double ABSOLUTE_DISCONTINUITY_POINT = .5;

    public static final double MIN = -200d;
    public static final double MAX = 160d;

    public static final double INDEXING_TIME = .3;

    public static final CANBus CANBUS = new CANBus("Shooter");
}