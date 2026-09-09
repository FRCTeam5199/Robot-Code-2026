package frc.robot;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
