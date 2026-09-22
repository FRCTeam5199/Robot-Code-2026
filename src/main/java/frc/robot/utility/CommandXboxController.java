package frc.robot.utility;

import org.wpilib.command2.button.CommandGamepad;
import org.wpilib.command2.button.Trigger;
import org.wpilib.driverstation.Gamepad;
import org.wpilib.event.EventLoop;

public class CommandXboxController extends CommandGamepad {
    private final Gamepad controller;
    private final EventLoop dsEventLoop = new EventLoop();
    private boolean useDs = false;

    public CommandXboxController(int port, boolean ds) {
        super(port);
        controller = new Gamepad(port);
        if (ds) useDs = true;
    }

    public Trigger a() {
        if (useDs) return super.button(0, dsEventLoop);
        return super.button(0);
    }

    public Trigger b() {
        if (useDs) return super.button(1, dsEventLoop);
        return super.button(1);
    }

    public Trigger x() {
        if (useDs) return super.button(2, dsEventLoop);
        return super.button(2);
    }

    public Trigger y() {
        if (useDs) return super.button(3, dsEventLoop);
        return super.button(3);
    }

    public Trigger back() {
        if (useDs) return super.button(4, dsEventLoop);
        return super.button(4);
    }
    
    public Trigger guide() {
        if (useDs) return super.button(5, dsEventLoop);
        return super.button(5);
    }

    public Trigger start() {
        if (useDs) return super.button(6, dsEventLoop);
        return super.button(6);
    }

    public Trigger povDown() {
        if (useDs) return super.dpadDown(dsEventLoop);
        return super.dpadDown();
    }

    public Trigger povUp() {
        if (useDs) return super.dpadUp(dsEventLoop);
        return super.dpadUp();
    }

    public Trigger povLeft() {
        if (useDs) return super.dpadLeft(dsEventLoop);
        return super.dpadLeft();
    }
    public Trigger povRight() {
        if (useDs) return super.dpadRight(dsEventLoop);
        return super.dpadRight();
    }

    public Trigger leftBumper() {
        if (useDs) return super.button(9, dsEventLoop);
        return super.button(9);
    }

    public Trigger rightBumper() {
        if (useDs) return super.button(10, dsEventLoop);
        return super.button(10);
    }

    public Trigger leftTrigger() {
        if (useDs) return super.leftTrigger(0.5, dsEventLoop);
        return super.leftTrigger();
    }

    public Trigger rightTrigger() {
        if (useDs) return super.rightTrigger(0.5, dsEventLoop);
        return super.rightTrigger();
    }

    // TODO: Finish and test
    public double getLeftX() {
        return controller.getLeftX();
    }

    public double getLeftY() {
        return controller.getLeftY();
    }

    public double getRightX() {
        return controller.getRightX();
    }

    public double getRightY() {
        return controller.getRightY();
    }
}