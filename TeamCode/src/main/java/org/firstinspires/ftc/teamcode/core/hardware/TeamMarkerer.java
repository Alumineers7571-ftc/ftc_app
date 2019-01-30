package org.firstinspires.ftc.teamcode.core.hardware;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.util.RobotConstants;

public class TeamMarkerer extends BaseHardware{

    private String hardwareName = "TeamMarkerer";
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    private Servo tm;

    /**
     *
     * @param hardwareMap instance of hardwareMap
     * @param telemetry instance of telemetry
     */
    public TeamMarkerer(HardwareMap hardwareMap, Telemetry telemetry){

        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        init();
    }

    /**
     *
     * @param hardwareMap instance of hardwareMap
     * @param telemetry instance of telemetry
     * @param name friendly name for telemetry
     */
    public TeamMarkerer(HardwareMap hardwareMap, Telemetry telemetry, String name){

        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        hardwareName = name;

        init();
    }


    public void init(){

        tm = hardwareMap.servo.get("tm");

        setTMUp();
    }

    @Override
    public void composeTelemetry() {

        this.telemetry.addLine(hardwareName + " is inited");

    }

    @Override
    public void controlSystem(Gamepad gamepad) {

        tm.setPosition(RobotConstants.TM_POS_UP);
    }

    /**
     * sets the teammarkerer servo into the DOWN position
     */
    public void setTMDown(){
        tm.setPosition(RobotConstants.TM_POS_DOWN);
    }

    /**
     * sets the teammarkerer servo into the UP position
     */
    public void setTMUp(){
        tm.setPosition(RobotConstants.TM_POS_UP);
    }

}
