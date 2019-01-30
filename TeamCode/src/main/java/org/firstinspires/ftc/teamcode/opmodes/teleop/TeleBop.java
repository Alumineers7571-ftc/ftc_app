package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.core.hardware.Robot;

public class TeleBop extends LinearOpMode {

    private Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Robot(hardwareMap, telemetry, TeleBop.this);

        while(!isStarted() && !isStopRequested()){

            telemetry.update();
        }

        while(opModeIsActive() && !isStopRequested()) {

            robot.hanger.controlSystem(gamepad1);
            robot.mineralSystem.controlSystem(gamepad2);
            robot.drive.controlSystem(gamepad1);
        }
    }
}
