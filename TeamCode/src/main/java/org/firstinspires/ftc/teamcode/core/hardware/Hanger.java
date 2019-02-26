package org.firstinspires.ftc.teamcode.core.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.core.util.RobotConstants;

/**
 * Class that contains all the code for the hanger class
 */
public class Hanger extends BaseHardware{

    private LinearOpMode opMode;
    private HardwareMap hardwareMap;
    private String hardwareName = "Hanger";
    private DcMotor hanger;
    private DistanceSensor range;
    private DigitalChannel touch;
    private Telemetry telemetry;

    private boolean isAtGround = false;

    private double inchesFromGround = RobotConstants.SENSOR_IN_FROM_GROUND;

    /**
     *
     * @param hMap hardwareMap reference
     * @param telemetry telemetry reference
     */
    public Hanger(HardwareMap hMap, Telemetry telemetry){

        hardwareMap = hMap;
        this.telemetry = telemetry;
        init();
    }

    public Hanger(HardwareMap hMap, Telemetry telemetry, LinearOpMode opMode){

        hardwareMap = hMap;
        this.telemetry = telemetry;
        this.opMode = opMode;
        init();
    }

    /**
     *
     * @param hMap hardwareMap reference
     * @param telemetry telemetry reference
     * @param name friendly name given to the Hanger class (used for telemetry)
     */
    public Hanger(HardwareMap hMap, Telemetry telemetry, String name){

        hardwareMap = hMap;
        hardwareName = name;
        this.telemetry = telemetry;
        init();
    }

    /**
     *
     * @param hMap hardwareMap reference
     * @param telemetry telemetry reference
     * @param name friendly name given to the Hanger class (used for telemetry)
     * @param inFromGround inches the sensor is from the ground when the robot is on the ground
     */
    public Hanger(HardwareMap hMap, Telemetry telemetry, String name, double inFromGround){

        hardwareMap = hMap;
        hardwareName = name;
        inchesFromGround = inFromGround;
        this.telemetry = telemetry;
        init();
    }

    /**
     * initialize the hanger hardware devices
     */
    private void init(){

        hanger = hardwareMap.get(DcMotorEx.class,"hanger");
        hanger.setDirection(DcMotorSimple.Direction.REVERSE); //its geared
        hanger.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        range = hardwareMap.get(DistanceSensor.class, "range");

        touch = hardwareMap.get(DigitalChannel.class, "sensor_digital");
        touch.setMode(DigitalChannel.Mode.INPUT);

    }

    public void composeTelemetry(){
        this.telemetry.addLine(hardwareName + " is inited");
    }

    /**
     *
     * @param gamepad controller object that is used to control the hanger in teleop
     */
    public void controlSystem(Gamepad gamepad){

        hanger.setPower(gamepad.right_trigger);

        if(gamepad.left_trigger > 0.05 && !isTouched()){
            moveToLowerLimit();
        }

    }

    /**
     * For quick and dirty auto moving
     * @param power -1.0 to 1.0, passed directly to the motor
     */
    public void controlSystem(double power){

        hanger.setPower(power);

    }

    public void moveToGround(){

        while((range.getDistance(DistanceUnit.INCH) < (inchesFromGround)) && opMode.opModeIsActive()){

            hanger.setPower(1);
            isAtGround = false;

        }

        hanger.setPower(0);
        isAtGround = true;


    }

    /**
     *
     * @return when the hanger is at it's lower limit (when the touch sensor is touched)
     */
    public boolean moveToLowerLimit(){

        if(isTouched()){

            hanger.setPower(0);
            return true;

        } else {

            hanger.setPower(-1);
            return false;

        }
    }

    /**
     * gets distance from ground in specified units
     * @param unit give the unit you want the distance returned in. ex. inches or mm
     * @return distance in x units
     */
    public double getDistanceToGround(DistanceUnit unit){

        return range.getDistance(unit);
    }

    /**
     *
     * @return if the touch sensor is touched
     */
    private boolean isTouched(){
        return !touch.getState();
    }

    /**
     *
     * @return is the robot at the ground (distance based)
     */
    public boolean isAtGround(){

        return isAtGround;
    }



}
