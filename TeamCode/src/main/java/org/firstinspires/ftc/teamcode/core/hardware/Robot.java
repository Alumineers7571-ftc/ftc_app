package org.firstinspires.ftc.teamcode.core.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot {

    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private LinearOpMode opMode;
    private boolean isAuto;
    private boolean motor;

    public Hanger hanger;
    public TeamMarkerer tm;
    public MecanumDrive drive;
    public MineralSystem mineralSystem;
    public CV cv;

    /**
     *
     * @param hardwareMap instance of hardwareMap
     * @param telemetry instance of telemetry
     */
    public Robot(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode opMode, boolean isAuto, boolean motor){

        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.opMode = opMode;
        this.isAuto = isAuto;
        this.motor = motor;

        init();

    }

    private void init(){
        hanger = new Hanger(this.hardwareMap, this.telemetry, this.opMode);
        tm = new TeamMarkerer(this.hardwareMap, this.telemetry);
        mineralSystem = new MineralSystem(this.hardwareMap, this.telemetry, this.motor);
        cv = new CV(this.hardwareMap, this.telemetry);
        drive = new MecanumDrive(this.hardwareMap, this.telemetry, this.opMode, this.isAuto);

        hanger.composeTelemetry();
        tm.composeTelemetry();
        mineralSystem.composeTelemetry();
        cv.composeTelemetry();
        drive.composeTelemetry();

        telemetry.update();
    }

}
