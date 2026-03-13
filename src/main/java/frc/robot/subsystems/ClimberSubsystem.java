package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.UpdateModeValue;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.units.measure.Distance;
import frc.robot.constants.ClimberConstants;
import frc.robot.subsystems.templates.TemplateSubsystem;
import frc.robot.utility.Type;

public class ClimberSubsystem extends TemplateSubsystem {
    private static ClimberSubsystem climberSubsystem;
    private StatusSignal<Distance> distance;
    private CANrange canRange;
    private CANrangeConfiguration canRangeConfiguration;
    private final LinearFilter distanceFilter = LinearFilter.movingAverage(3);


    private ClimberSubsystem() {
        super(Type.LINEAR, ClimberConstants.MOTOR_ID, ClimberConstants.VELOCITY,
                ClimberConstants.ACCELERATION, ClimberConstants.JERK,
                ClimberConstants.LOWER_TOLERANCE,
                ClimberConstants.UPPER_TOLERANCE,
                ClimberConstants.GEAR_RATIO, "Climber", true);

        configureMotor(ClimberConstants.INVERTED, ClimberConstants.BRAKE,
                ClimberConstants.SUPPLY_CURRENT_LIMIT,
                ClimberConstants.STATOR_CURRENT_LIMIT,
                ClimberConstants.SLOT0_CONFIGS);

        configureLinearMech(ClimberConstants.DRUM_CIRCUMFERENCE, ClimberConstants.MIN,
                ClimberConstants.MAX);

        canRange = new CANrange(ClimberConstants.CAN_RANGE_ID);

        canRangeConfiguration = new CANrangeConfiguration();
        canRangeConfiguration.ToFParams.UpdateMode = UpdateModeValue.LongRangeUserFreq;
        canRangeConfiguration.ToFParams.UpdateFrequency = 50; // Hz, set to what you need
        canRangeConfiguration.FovParams.FOVRangeX = 2;
        canRangeConfiguration.FovParams.FOVRangeY = 2;
        canRangeConfiguration.ProximityParams.ProximityHysteresis = 0.005;
        canRangeConfiguration.ProximityParams.MinSignalStrengthForValidMeasurement = 2500;

        canRange.getConfigurator().apply(canRangeConfiguration);

//        canRange.getDistance().setUpdateFrequency(50);
//        distance = canRange.getDistance();
//        canRange.optimizeBusUtilization();
    }

    public static ClimberSubsystem getInstance() {
        if (climberSubsystem == null) {
            climberSubsystem = new ClimberSubsystem();
        }
        return climberSubsystem;
    }

    public void periodic() {
        super.periodic();
//        distance.refresh();

//        System.out.println("Climber position: " + climberSubsystem.getMechM());
//        System.out.println("Distance: " + getDistance());
    }

    public double getDistance() {
//        return distance.getValueAsDouble();
        return distanceFilter.calculate(canRange.getDistance().getValueAsDouble());
    }
}
