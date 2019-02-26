package org.firstinspires.ftc.teamcode.opmodes.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import static org.firstinspires.ftc.teamcode.opmodes.auton.Crater.COUNTS_PER_INCH;

@Autonomous
public class Forward extends LinearOpMode{

    private DcMotorEx FL, FR, BL, BR;
    private DcMotorEx pivoter, extendo, intakeMotor;
    private CRServoImplEx intake;
    private ServoImplEx gate;

    @Override
    public void runOpMode() throws InterruptedException {


        /*FL = hardwareMap.get(DcMotorEx.class, "fl");
        FR = hardwareMap.get(DcMotorEx.class, "fr");
        BL = hardwareMap.get(DcMotorEx.class, "bl");
        BR = hardwareMap.get(DcMotorEx.class, "br");

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);*/

        extendo = hardwareMap.get(DcMotorEx.class, "extendo");
        pivoter = hardwareMap.get(DcMotorEx.class, "pivoter");

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE); // it's geared

        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        gate = hardwareMap.get(ServoImplEx.class, "gate");

        waitForStart();

        //drive(12);

        while(opModeIsActive()){

            extendo.setPower(gamepad2.right_stick_y);

            if(gamepad2.right_bumper) {
                runIntake(0.45);
            } else if(gamepad2.left_bumper) {
                runIntake(-0.5);
            } else {
                runIntake(0);
            }

            pivoter.setPower(gamepad2.left_stick_y);

            gate.setPosition(Range.clip(Math.abs(gamepad2.right_trigger - 1), 0.4, 1));
/*
            FL.setPower(gamepad1.left_stick_y + gamepad1.right_stick_x);
            FR.setPower(gamepad1.left_stick_y - gamepad1.right_stick_x);
            BL.setPower(gamepad1.left_stick_y + gamepad1.right_stick_x);
            BR.setPower(gamepad1.left_stick_y - gamepad1.right_stick_x);*/

            //telemetry.addLine(FL.getCurrentPosition() + " " + FR.getCurrentPosition() + " " + BL.getCurrentPosition() + " " + BR.getCurrentPosition());
            //telemetry.update();
        }

    }

    public void runIntake(double power){
        intakeMotor.setPower(power);
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
