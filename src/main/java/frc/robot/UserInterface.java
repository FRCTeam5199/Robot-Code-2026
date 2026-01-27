package frc.robot;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class UserInterface {
    private static ShuffleboardTab autonTab, teleopTab, controlTab;

    public static Map<String, GenericEntry> autonComponents = new HashMap<String, GenericEntry>();
    public static Map<String, GenericEntry> teleopComponents = new HashMap<String, GenericEntry>();
    public static Map<String, GenericEntry> controlComponents = new HashMap<String, GenericEntry>();

    private UserInterface() {}

    public static void init() {
        initalizeTabs();
        initalizeComponents();
        Shuffleboard.selectTab("Autons");
    }

    private static void initalizeTabs() {
        autonTab = Shuffleboard.getTab("Auton");
        teleopTab = Shuffleboard.getTab("Teleop");
        controlTab = Shuffleboard.getTab("Control");
    }

    private static void initalizeComponents() {
        createControlComponent("Actuator Percent", 0.0d, BuiltInWidgets.kTextView, 0, 0, 2, 2, null);
        createControlComponent("Actuator Voltage", 0.0d, BuiltInWidgets.kTextView, 0, 2, 2, 2, null);
        createControlComponent("Actuator Velocity", 0.0d, BuiltInWidgets.kTextView, 0, 4, 2, 2, null);
        createControlComponent("Actuator Position", 0.0d, BuiltInWidgets.kTextView, 0, 6, 2, 2, null);

        createControlComponent("Roller Percent", 0.0d, BuiltInWidgets.kTextView, 2, 0, 2, 2, null);
        createControlComponent("Roller Voltage", 0.0d, BuiltInWidgets.kTextView, 2, 2, 2, 2, null);
        createControlComponent("Roller Velocity", 0.0d, BuiltInWidgets.kTextView, 2, 4, 2, 2, null);

        createControlComponent("Edit Button", false, BuiltInWidgets.kToggleSwitch, 5, 0, 1, 1, null);
    }

    public static void setTab(String key) {
        Shuffleboard.selectTab(key);
    }
    
    public static ShuffleboardTab getTab(String key) {
        return Shuffleboard.getTab(key);
    }

    public static void putData(String key, Sendable data) {
        SmartDashboard.putData(key, data);
    }

    public static Sendable getData(String key) {
        return SmartDashboard.getData(key);
    }

    @SuppressWarnings("rawtypes")
    public static void addSendableChooserComponent(String name, SendableChooser sendableChooser, WidgetType widget, int positionX, int positionY, int width, int height, Map<String, Object> properties) {
        autonTab.add(name, sendableChooser).withWidget(widget).withPosition(positionX, positionY).withSize(width, height).withProperties(properties);
    }

    public static void createAutonComponent(String name, Object defaultValue, WidgetType widget, int positionX, int positionY, int width, int height, Map<String, Object> properties) {
        autonComponents.putIfAbsent(name, autonTab.add(name, defaultValue).withWidget(widget).withPosition(positionX, positionY).withSize(width, height).withProperties(properties).getEntry());
    }

    public static void createTeleopComponent(String name, Object defaultValue, WidgetType widget, int positionX, int positionY, int width, int height, Map<String, Object> properties) {
        teleopComponents.putIfAbsent(name, teleopTab.add(name, defaultValue).withWidget(widget).withPosition(positionX, positionY).withSize(width, height).withProperties(properties).getEntry());
    }

    public static void createControlComponent(String name, Object defaultValue, WidgetType widget, int positionX, int positionY, int width, int height, Map<String, Object> properties) {
        controlComponents.putIfAbsent(name, controlTab.add(name, defaultValue).withWidget(widget).withPosition(positionX, positionY).withSize(width, height).withProperties(properties).getEntry());
    }

    public static GenericEntry getAutonComponent(String key) {
        return autonComponents.get(key);
    }

    public static GenericEntry getTeleopComponent(String key) {
        return teleopComponents.get(key);
    }

    public static GenericEntry getControlComponent(String key) {
        return controlComponents.get(key);
    }

    public static void setAutonComponent(String key, Object value) {
        if (autonComponents.get(key).setValue(value)) {
            autonComponents.replace(key, autonComponents.get(key));
        }
    }

    public static void setTeleopComponent(String key, Object value) {
        if (teleopComponents.get(key).setValue(value)) {
            teleopComponents.replace(key, autonComponents.get(key));
        }
    }

    public static void setControlComponent(String key, Object value) {
        controlComponents.get(key).setValue(value);
    }

    public static void update() {
        Shuffleboard.update();
    }
}
