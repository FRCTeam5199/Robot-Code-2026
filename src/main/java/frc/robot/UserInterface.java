package frc.robot;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utility.Elastic;

public class UserInterface {
    public static Map<String, SimpleWidget> components = new HashMap<String, SimpleWidget>();

    private UserInterface() {}

    public static void init() {
        createComponent("Pivot Percent", "Control", BuiltInWidgets.kTextView, 0, 0, 1, 1);
        createComponent("Pivot Position", "Control", BuiltInWidgets.kTextView, 1, 0, 1, 1);
        createComponent("Pivot Velocity", "Control", BuiltInWidgets.kTextView, 2, 0, 1, 1);
        createComponent("Pivot Voltage", "Control", BuiltInWidgets.kVoltageView, 3, 0, 1, 1);

        // createComponent("Pivot Percent", "Control", BuiltInWidgets.kTextView, 0, 0, 1, 1);
        // createComponent("Pivot Velocity", "Control", BuiltInWidgets.kTextView, 0, 0, 1, 1);
        // createComponent("Pivot Voltage", "Control", BuiltInWidgets.kTextView, 0, 0, 1, 1);

        Shuffleboard.selectTab("Autons");
        Elastic.selectTab("Autons");
    }

    public static void setTab(String key) {
        Shuffleboard.selectTab(key);
        Elastic.selectTab(key);
    }

    public static ShuffleboardTab getTab(String key) {
        return Shuffleboard.getTab(key);
    }

    public static SimpleWidget createComponent(String name, String tab, WidgetType widgetType, int posx, int posy, int sizex, int sizey) {
        return components.putIfAbsent(name, Shuffleboard.getTab(tab)
            .add(name, Sendable.class)
            .withWidget(widgetType)
            .withPosition(posx, posy)
            .withSize(sizex, sizey));
    }

    public static void deleteComponent(String name) {
        components.get(name).close();
        components.remove(name);
    }

    public static void setComponentValue(String name, NetworkTableValue data) {
        components.get(name).getEntry().set(data);
    }

    public static void putData(String key, Sendable data) {
        SmartDashboard.putData(key, data);
    }

    public static Sendable getData(String key) {
        return SmartDashboard.getData(key);
    }

    public static void update() {
        Shuffleboard.update();
    }
}
