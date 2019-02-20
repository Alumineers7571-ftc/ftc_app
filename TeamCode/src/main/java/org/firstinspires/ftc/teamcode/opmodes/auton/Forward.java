package org.firstinspires.ftc.teamcode.opmodes.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import static org.firstinspires.ftc.teamcode.opmodes.auton.Crater.COUNTS_PER_INCH;

@Autonomous
public class Forward extends LinearOpMode{

    public DcMotorEx FL, FR, BL, BR;

    @Override
    public void runOpMode() throws InterruptedException {


        FL = hardwareMap.get(DcMotorEx.class, "fl");
        FR = hardwareMap.get(DcMotorEx.class, "fr");
        BL = hardwareMap.get(DcMotorEx.class, "bl");
        BR = hardwareMap.get(DcMotorEx.class, "br");

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        drive(12);

    }

    public void drive(double inches){

        int newFLTarget, newFRTarget, newBLTarget, newBRTarget;

        newFLTarget = (int) (inches * COUNTS_PER_INCH);
        newFRTarget = (int) (inches * COUNTS_PER_INCH);
        newBLTarget = (int) (inches * COUNTS_PER_INCH);
        newBRTarget = (int) (inches * COUNTS_PER_INCH);

        while(opModeIsActive()){

            if((Math.abs(FL.getCurrentPosition()) < newFLTarget) && Math.abs(FR.getCurrentPosition()) < newFRTarget) {
                setMotorPowers(0.5);
            } else {
                setMotorPowers(0);
            }

        }

    }

    public void setMotorPowers(double power){

        FL.setPower(power);
        FR.setPower(power);
        BL.setPower(power);
        BR.setPower(power);

    }
}
