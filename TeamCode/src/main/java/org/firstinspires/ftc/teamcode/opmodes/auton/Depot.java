package org.firstinspires.ftc.teamcode.opmodes.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.core.hardware.Robot;
import org.firstinspires.ftc.teamcode.core.util.ENUMS;

@Autonomous
public class Depot extends LinearOpMode {
    
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
    private int sampleHitDist = -23;
    private int sampleReturnDist = 10;
    private int wallTurn1Deg = -90;
    private int wallNav1Dist = -30;
    private int wallTurn2Deg = 45;
    private int wallNav2Dist = -14;
    private int depotNavDist = 52;
    private int craterNavDist = -75;

    private boolean imuDone = false;

    @Override
    public void runOpMode() {

        robot = new Robot(hardwareMap, telemetry, Crater.this, true, true);

        while(!isStopRequested() && !isStarted()) {

            goldLocation = robot.cv.getGoldLocation();

            switch(goldLocation){
                case LEFT:{
                    sampleTurnDeg = 225;
                    wallNav1Dist -= 14.5;
                    sampleHitDist = -27;
                    sampleReturnDist = 12;
                    //wallTurn1Deg = 45;
                    break;
                }
                case CENTER:{
                    sampleTurnDeg = 180;
                    break;
                }
                case RIGHT:{
                    sampleTurnDeg = 135;
                    wallNav1Dist += 14.5;
                    sampleHitDist = -27;
                    sampleReturnDist = 12;
                    //wallTurn1Deg = 45 + 90;
                }
            }

            robot.tm.setTMDown();

            telemetry.update();
        }

        robot.drive.initIMU();
        imuDone = true;

        while(!isStopRequested() && opModeIsActive()){

            switch (robo){
                case START:{

                    robo = ENUMS.AutoStates.DROP;
                    break;
                }

                case DROP:{

                    robot.hanger.moveToGround();

                    telemetry.addData("heading:", robot.drive.getAngle());
                    telemetry.update();

                    robot.drive.turnAbsoulte(sampleTurnDeg);

                    robo = ENUMS.AutoStates.HITGOLD;
                    break;
                }

                case HITGOLD:{

                    robot.drive.drive(sampleHitDist);
                    sleep(200);
                    robot.drive.drive(sampleReturnDist);
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
                    sleep(200);

                    robo = ENUMS.AutoStates.ALIGNWITHWALL;
                    break;
                }

                case ALIGNWITHWALL:{

                    robot.drive.turnAbsoulte(135);
                    sleep(200);
                    robot.drive.setStrafePower(0.6);
                    sleep(200);
                    robot.drive.setStrafePower(-0.2);
                    sleep(200);
                    robot.drive.setStrafePower(0);
                    sleep(200);

                    robo = ENUMS.AutoStates.NAVTOTM;
                    break;

                }

                case NAVTOTM:{

                    robot.drive.drive(depotNavDist);
                    sleep(200);

                    robo = ENUMS.AutoStates.DROPTM;
                    break;
                }

                case DROPTM: {

                    robot.tm.setTMUp();
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

            robot.tm.setTMDown();

            idle();
            
            if(imuDone){
                telemetry.addData("heading:", robot.drive.getAngle());
            }
            telemetry.addLine("state: " + robo);
            telemetry.update();
        }

    }
}
