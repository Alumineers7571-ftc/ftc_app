package org.firstinspires.ftc.teamcode.core.hardware;

import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.util.PIDController;

public class MineralSystem extends BaseHardware{

    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private String hardwareName = "MineralSystem";

    private boolean motor;

    private DcMotorEx pivoter, extendo, intakeMotor;
    private CRServoImplEx intake;
    private ServoImplEx gate;

    private PIDController pid;

    private boolean pidOn = false;

    private double correction, power = 0.7;

    /**
     *
     * @param hardwareMap instance of hardwareMap
     * @param telemetry instance of telemetry
     */
    public MineralSystem(HardwareMap hardwareMap, Telemetry telemetry){

        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        init();
    }

    public MineralSystem(HardwareMap hardwareMap, Telemetry telemetry, boolean motor){

        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.motor = motor;

        init();
    }

    /**
     *
     * @param hardwareMap instance of hardwareMap
     * @param telemetry instance of telemetry
     * @param name friendly name for telemetry
     */
    public MineralSystem(HardwareMap hardwareMap, Telemetry telemetry, String name){

        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        hardwareName = name;

        init();
    }

    private void init() {

        extendo = hardwareMap.get(DcMotorEx.class, "extendo");
        pivoter = hardwareMap.get(DcMotorEx.class, "pivoter");

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE); // it's geared

        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        gate = hardwareMap.get(ServoImplEx.class, "gate");

        pid = new PIDController(.005, 0, 0);

        pid.setSetpoint(0);
        pid.setOutputRange(0, power);
        pid.setInputRange(-1120*4, 1120*4);
        pid.setTolerance(1);
        pid.enable();

        extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pivoter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivoter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    /**
     *
     * @param gamepad instance of gamepad to control system in teleop
     */
    @Override
    public void controlSystem(Gamepad gamepad) {

        if(pidOn){

            correction = pid.performPID(extendo.getCurrentPosition());
            extendo.setPower(correction);

        } else {

            extendo.setPower(gamepad.right_stick_y);

        }


        if(gamepad.right_bumper) {
            runIntake(0.45);
        } else if(gamepad.left_bumper) {
             runIntake(-0.5);
        } else {
            runIntake(0);
        }


        gate.setPosition(Range.clip(Math.abs(gamepad.right_trigger - 1), 0.4, 1));

        if(gamepad.right_stick_button){
            pidOn = !pidOn;
        }


        pivoter.setPower(gamepad.left_stick_y);

    }

    /**
     *
     * @param power -1 to 1 direct power
     */
    public void runIntake(double power){
        intakeMotor.setPower(power);
    }

    @Override
    public void composeTelemetry() {

        this.telemetry.addLine(hardwareName + " is inited");

    }
}
