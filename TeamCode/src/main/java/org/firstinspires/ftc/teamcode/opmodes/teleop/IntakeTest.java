package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class IntakeTest extends LinearOpMode {

    private DcMotorEx intake;

    private double intakePower = 0.00;

    @Override
    public void runOpMode() throws InterruptedException {

        intake = hardwareMap.get(DcMotorEx.class, "intake");

        while(!isStarted() && !isStopRequested()){
            telemetry.addLine("waiting on you...");
            telemetry.update();
        }

        while(opModeIsActive() && !isStopRequested()){

            intake.setPower(intakePower);

            if(gamepad1.dpad_up){
                intakePower += 0.01;
            } else if(gamepad1.dpad_down){
                intakePower -= 0.01;
            }

            telemetry.addData("power:", intakePower);
            telemetry.update();

        }


    }


}
