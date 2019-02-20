package org.firstinspires.ftc.teamcode.opmodes.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.core.hardware.Robot;
import org.firstinspires.ftc.teamcode.core.util.ENUMS;

@Autonomous
public class Crater extends LinearOpMode {

    private ENUMS.GoldLocation goldLocation = ENUMS.GoldLocation.UNKNOWN;

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    private Robot robot;
    
    private ENUMS.AutoStates robo = ENUMS.AutoStates.START;

    private int sampleTurnDeg = 180;
    private int sampleHitDist = -14;
    private int wallTurn1Deg = 270;
    private int wallNav1Dist = 30;
    private int wallTurn2Deg = 220;
    private int wallNav2Dist = 7;
    private int depotNavDist = 42;

    private boolean imuDone = false;

    @Override
    public void runOpMode() {

        robot = new Robot(hardwareMap, telemetry, Crater.this, true, true);

        while(!isStopRequested() && !isStarted()) {

            goldLocation = robot.cv.getGoldLocation();

            switch(goldLocation){
                case LEFT:{
                    sampleTurnDeg = -135;
                    wallNav1Dist -= 14.5;
                    break;
                }
                case CENTER:{
                    sampleTurnDeg = 180;
                    break;
                }
                case RIGHT:{
                    sampleTurnDeg = -225;
                    wallNav1Dist += 14.5;
                }
            }


            telemetry.update();
        }

        while(!isStopRequested() && opModeIsActive()){

            switch (robo){
                case START:{

                    robo = ENUMS.AutoStates.DROP;
                    break;
                }

                case DROP:{

                    robot.hanger.moveToGround();

                    robot.drive.initIMU();

                    imuDone = true;

                    telemetry.addData("heading:", robot.drive.getAngle());
                    telemetry.update();

                    robot.drive.turnAbsoulte(180);

                    robo = ENUMS.AutoStates.HITGOLD;
                    break;
                }

                case HITGOLD:{

                    robot.drive.drive(-3);
                    sleep(300);
                    robot.drive.drive(sampleHitDist);
                    sleep(300);
                    robot.drive.drive(6);
                    sleep(300);

                    /*
                    encoderDrive(0.4, -3, 3.0);
                    //robot.drive.turnAbsoulte(sampleTurnDeg);
                    encoderDrive(0.4, sampleHitDist, 4.0);
                    encoderDrive(0.4, 6, 4.0);
                    */

                    robo = ENUMS.AutoStates.NAVTOWALL;
                    break;
                }

                case NAVTOWALL:{


                    robot.drive.turnAbsoulte(wallTurn1Deg);
                    sleep(300);
                    encoderDrive(0.6, wallNav1Dist, 10.0);
                    robot.drive.turnAbsoulte(wallTurn2Deg); //facing depot while parallel to wall
                    sleep(300);
                    encoderDrive(0.6, wallNav2Dist, 2.0);
                    robot.drive.turnAbsoulte(135);
                    sleep(300);
                    robot.drive.setStrafePower(-0.6);
                    sleep(200);
                    robot.drive.setStrafePower(0.2);
                    sleep(200);
                    robot.drive.setStrafePower(0);
                    encoderDrive(0.6, 42, 10.0);

                    robo = ENUMS.AutoStates.END;
                    break;

                }

                case END:{


                    break;
                }

            }

            if(imuDone){
                telemetry.addData("heading:", robot.drive.getAngle());
            }
            telemetry.addLine("state: " + robo);
            telemetry.update();
        }

    }


    public void encoderDrive(double speed,
                             double inches,
                             double timeoutS) {
        int newFLTarget, newFRTarget, newBLTarget, newBRTarget;

        robot.drive.setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.drive.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFLTarget = (int) (inches * COUNTS_PER_INCH);
            newFRTarget = (int) (inches * COUNTS_PER_INCH);
            newBLTarget = (int) (inches * COUNTS_PER_INCH);
            newBRTarget = (int) (inches * COUNTS_PER_INCH);

            /*robot.drive.FL.setTargetPosition(newFLTarget);
            robot.drive.FR.setTargetPosition(newFRTarget);
            robot.drive.BL.setTargetPosition(newBLTarget);
            robot.drive.BR.setTargetPosition(newBRTarget);*/

            // Turn On RUN_TO_POSITION
            robot.drive.setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.

            robot.drive.setMotorPowers(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (robot.drive.isBusy())) {
                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d", newFLTarget, newFRTarget, newBLTarget, newBRTarget);
                //telemetry.addData("Path2",  "Running at %7d :%7d :%7d :%7d",
                //FL.getCurrentPosition(),
                //      FR.getCurrentPosition(),
                //    BL.getCurrentPosition(),
                //  BR.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.drive.setMotorPowers(0);

            // Turn off RUN_TO_POSITION
            robot.drive.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move
        }
    }

}
