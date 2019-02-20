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
    private int craterNavDist = -60;

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

                    robot.drive.turnAbsoulte(sampleTurnDeg);

                    robo = ENUMS.AutoStates.HITGOLD;
                    break;
                }

                case HITGOLD:{

                    robot.drive.drive(sampleHitDist);
                    sleep(200);
                    robot.drive.drive(6);
                    sleep(200);

                    robo = ENUMS.AutoStates.NAVTOWALL;
                    break;
                }

                case NAVTOWALL: {

                    robot.drive.turnAbsoulte(wallTurn1Deg);
                    sleep(200);
                    robot.drive.drive(wallNav1Dist);
                    robot.drive.turnAbsoulte(wallTurn2Deg); //facing depot while parallel to wall
                    sleep(200);
                    robot.drive.drive(wallNav2Dist);

                    robo = ENUMS.AutoStates.ALIGNWITHWALL;
                    break;
                }

                case ALIGNWITHWALL:{

                    robot.drive.turnAbsoulte(135);
                    sleep(200);
                    robot.drive.setStrafePower(-0.6);
                    sleep(200);
                    robot.drive.setStrafePower(0.2);
                    sleep(200);
                    robot.drive.setStrafePower(0);
                    sleep(200);

                    robo = ENUMS.AutoStates.NAVTOTM;
                    break;

                }

                case NAVTOTM:{

                    robot.drive.drive(depotNavDist);

                    robo = ENUMS.AutoStates.DROPTM;
                    break;
                }

                case DROPTM: {

                    robot.tm.setTMDown();
                    sleep(500);

                    robo = ENUMS.AutoStates.PARK;
                    break;
                }

                case PARK: {

                    robot.drive.drive(craterNavDist); // approx.

                    robo = ENUMS.AutoStates.END;
                    break;
                }

                case END:{


                    break;
                }

            }

            robot.tm.setTMUp();

            if(imuDone){
                telemetry.addData("heading:", robot.drive.getAngle());
            }
            telemetry.addLine("state: " + robo);
            telemetry.update();
        }

    }
}
