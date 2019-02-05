package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.core.hardware.Robot;

public class TeleBop extends LinearOpMode {

    @Override
    public void runOpMode() {

        Robot robot = new Robot(hardwareMap, telemetry, TeleBop.this, false);

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
