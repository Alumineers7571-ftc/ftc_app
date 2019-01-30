package org.firstinspires.ftc.teamcode.core.hardware;

import com.qualcomm.robotcore.hardware.Gamepad;

public abstract class BaseHardware {

    private String hardwareName = null;

    public BaseHardware(){

    }

    abstract public void controlSystem(Gamepad gamepad);

    abstract public void composeTelemetry();

}
