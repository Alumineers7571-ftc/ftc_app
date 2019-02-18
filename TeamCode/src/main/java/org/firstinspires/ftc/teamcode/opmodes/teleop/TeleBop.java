package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.hardware.Robot;

@TeleOp
public class TeleBop extends LinearOpMode {

    @Override
    public void runOpMode() {

        Robot robot = new Robot(hardwareMap, telemetry, TeleBop.this, false, true);
        robot.drive.initIMU();

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
