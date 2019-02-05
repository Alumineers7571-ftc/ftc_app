package org.firstinspires.ftc.teamcode.opmodes.auton;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.core.hardware.Robot;
import org.firstinspires.ftc.teamcode.core.util.ENUMS;

public class Crater extends LinearOpMode {

    private Robot robot = new Robot(hardwareMap, telemetry, Crater.this, true);

    private ENUMS.GoldLocation goldLocation = ENUMS.GoldLocation.UNKNOWN;

    private ENUMS.AutoStates robo = ENUMS.AutoStates.START;

    private int sampleTurnDeg = 0;

    @Override
    public void runOpMode() {

        while(!isStopRequested() && !isStarted()) {

            goldLocation = robot.cv.getGoldLocation();

            switch(goldLocation){
                case LEFT:{
                    sampleTurnDeg = 30;
                    break;
                }
                case CENTER:{
                    sampleTurnDeg = 0;
                    break;
                }
                case RIGHT:{
                    sampleTurnDeg = -30;
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

                    robot.drive.rotate(180);

                    robo = ENUMS.AutoStates.HITGOLD;
                    break;
                }

                case HITGOLD:{

                    robot.drive.rotate(sampleTurnDeg);
                    robot.drive.encoderDrive(0.6, 24, 24, 4.0);
                    sleep(300);
                    robot.drive.encoderDrive(0.6, -24, -24, 4.0);

                    robo = ENUMS.AutoStates.NAVTOWALL;
                    break;
                }

                case NAVTOWALL:{

                    //robot.drive.rotate(to hit wall);

                }

                case END:{


                    break;
                }

            }

            telemetry.addLine("state: " + robo);
            telemetry.update();
        }

    }
}
