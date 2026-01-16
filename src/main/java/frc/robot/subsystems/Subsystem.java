package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.Constants;

public class Subsystem extends SubsystemBase {

    //Declaring motor variable
    TalonFX motor = new  TalonFX(Constants.motorID);
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    
    // Motion Magic
    MotionMagicVelocityVoltage motionMagicVelocityVoltage;
    VelocityVoltage velocityVoltage;
 
    //Feed forward
    SimpleMotorFeedforward rollerFF = new SimpleMotorFeedforward(Constants.kS, Constants.kV);
    
    public Subsystem() {
        configMotor();
        motionMagicVelocityVoltage = 
            new MotionMagicVelocityVoltage(0)
            .withEnableFOC(true).withSlot(0);
        velocityVoltage = 
            new VelocityVoltage(0)
            .withEnableFOC(true).withSlot(0);
    }

    public void configMotor() {
        motorConfig.MotorOutput.Inverted 
        = InvertedValue.CounterClockwise_Positive; //or InvertedValue.CounterClockWisePositive
        motorConfig.MotorOutput.NeutralMode 
        = NeutralModeValue.Brake; //or NeutralModeValue.Coast
    
        motorConfig.CurrentLimits.SupplyCurrentLimit = Constants.supplyCurrentLimit; 
        motorConfig.CurrentLimits.StatorCurrentLimit = Constants.statorCurrentLimit; 
    
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true; 
        motorConfig.CurrentLimits.StatorCurrentLimitEnable = true; 


        var slot0Configs = motorConfig.Slot0;
            slot0Configs.kS = Constants.kS; // Add 0.25 V output to overcome static friction
            slot0Configs.kV = Constants.kV; // A velocity target of 1 rps results in 0.12 V output
            slot0Configs.kA = Constants.kA; // An acceleration of 1 rps/s requires 0.01 V output
            slot0Configs.kP = Constants.P; // An error of 1 rps results in 0.11 V output
            slot0Configs.kI = Constants.I; // no output for integrated error
            slot0Configs.kD = Constants.D; // no output for error derivative

        motorConfig.MotionMagic.MotionMagicAcceleration = Constants.acceleration; // Target acceleration of 400 rps/s (0.25 seconds to max)
        motorConfig.MotionMagic.MotionMagicJerk = Constants.jerk; // Target jerk of 4000 rps/s/s (0.1 seconds)

        motor.getConfigurator().apply(motorConfig);
        motor.setPosition(0); 
    }

    public void periodic() {
        System.out.println("Motor percentage: " + motor.getRotorPosition());
    }

    public void setVelocityVoltage(double velocity) {
        motor.setControl(velocityVoltage.withVelocity(velocity));
    }   

    public void setMotionMagicVelocity(double velocity) {
        motor.setControl(motionMagicVelocityVoltage.withVelocity(velocity));
    }

    public void setMotionMagicVelocityWithFF(double velocity) {
        motor.setControl(motionMagicVelocityVoltage.withVelocity(velocity)
            .withFeedForward(rollerFF.calculateWithVelocities(motor.getVelocity().getValueAsDouble(), velocity)));
    }
}