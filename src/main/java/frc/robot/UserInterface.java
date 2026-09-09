package frc.robot;

import java.util.HashMap;
import java.util.Map;

import org.wpilib.networktables.GenericEntry;
import org.wpilib.util.sendable.Sendable;
import org.wpilib.shuffleboard.Shuffleboard;
import org.wpilib.shuffleboard.ShuffleboardTab;
import org.wpilib.shuffleboard.SimpleWidget;
import org.wpilib.shuffleboard.WidgetType;
import org.wpilib.smartdashboard.SmartDashboard;

public class UserInterface {
    private static UserInterface self;

    public static Map<String, SimpleWidget> components = new HashMap<String, SimpleWidget>();

    private UserInterface() {}

    public static UserInterface getInstance() { if (self == null) self = new UserInterface(); return self; }

    public void setTab(String key) {
        Shuffleboard.selectTab(key);
    }

    public ShuffleboardTab getTab(String key) {
        return Shuffleboard.getTab(key);
    }

    public SimpleWidget createComponent(String name, String tab, WidgetType widgetType, int posx, int posy, int sizex, int sizey, Object defaultValue) {
        return components.putIfAbsent(name, Shuffleboard.getTab(tab)
            .add(name, defaultValue)
            .withWidget(widgetType)
            .withPosition(posx, posy)
            .withSize(sizex, sizey));
    }

    public SimpleWidget getComponent(String name) {
        return components.get(name);
    }

     public GenericEntry getComponentData(String name) {
        return components.get(name).getEntry();
    }

    public void setComponentData(String name, Object data) {
        components.get(name).getEntry().setValue(data);
    }

    public void deleteComponent(String name) {
        components.get(name).close();
        components.remove(name);
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
