package org.firstinspires.ftc.teamcode.opmodes.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.core.hardware.MecanumDrive;

@Autonomous (group = "test")
public class ImuTelemtry extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDrive drive = new MecanumDrive(hardwareMap, telemetry, ImuTelemtry.this, false);
        drive.initIMU();

        waitForStart();

        while(opModeIsActive()){
            telemetry.addLine("heading: " + drive.getAngle());
            telemetry.update();
        }

    }
}
