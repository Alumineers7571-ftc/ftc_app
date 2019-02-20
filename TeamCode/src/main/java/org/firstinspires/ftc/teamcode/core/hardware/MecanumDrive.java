package org.firstinspires.ftc.teamcode.core.hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.motors.NeveRest40Gearmotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.core.util.ENUMS;
import org.firstinspires.ftc.teamcode.core.util.PIDController;
import org.firstinspires.ftc.teamcode.core.util.RobotConstants;

import java.util.Locale;


public class MecanumDrive extends BaseHardware{

    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    private double fl = 0, fr = 0, bl = 0, br = 0;

    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private String hardwareName = "DriveTrain";
    private LinearOpMode opMode = null;

    private boolean turnDone = false;

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotorEx FL, FR, BL, BR;
    private BNO055IMU imu;

    private boolean isAuto = true;

    private PIDController pidRotate;

    private Orientation lastAngles = new Orientation();
    private double startingAngle, globalAngle, power = .4, correction;

    private boolean pidEnabled = false;

    private ENUMS.DriveMode driveMode = ENUMS.DriveMode.MECANUM;

    public MecanumDrive(HardwareMap hardwareMap, Telemetry telemetry){

        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        init();
    }

    public MecanumDrive(HardwareMap hardwareMap, Telemetry telemetry, String name){

        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        hardwareName = name;

        init();
    }

    public MecanumDrive(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode opMode, boolean isAuto){

        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.opMode = opMode;
        this.isAuto = isAuto;

        init();
    }

    private void init(){

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

        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pidRotate = new PIDController(RobotConstants.Kp, 0, 0);

        pidRotate.reset();
        pidRotate.disable();

    }

    public void initIMU(){

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        while(!opMode.isStopRequested() && !imu.isGyroCalibrated()){}

        startingAngle = getAngle();

    }

    @Override
    public void controlSystem(Gamepad gamepad) {

        globalAngle = getAngle();

        if(gamepad.left_stick_button){
            rotateTeleop((int)startingAngle);
        } else if(gamepad.dpad_down){
            startingAngle = globalAngle;
        }

        if(gamepad.right_stick_button){
            if(driveMode == ENUMS.DriveMode.MECANUM){
                driveMode = ENUMS.DriveMode.FIELDCENTRIC;
            } else if (driveMode == ENUMS.DriveMode.FIELDCENTRIC){
                driveMode = ENUMS.DriveMode.MECANUM;
            }
        }


        switch (driveMode) {
            case MECANUM: {

                fl = ((gamepad.left_stick_y) + gamepad.right_stick_x + gamepad.left_stick_x);
                fr = ((gamepad.left_stick_y) - gamepad.right_stick_x - gamepad.left_stick_x);
                bl = ((gamepad.left_stick_y) + gamepad.right_stick_x - gamepad.left_stick_x);
                br = ((gamepad.left_stick_y) - gamepad.right_stick_x + gamepad.left_stick_x);
                break;
            }
            case FIELDCENTRIC: {

                double lx = gamepad.left_stick_x, ly = - gamepad.left_stick_y;
                double v = Math.sqrt(lx * lx + ly * ly);
                double theta = Math.atan2(lx, ly);
                double current = Math.toRadians(globalAngle);

                final double vd = theta + current;
                final double vt = gamepad.right_stick_x;

                double s =  Math.sin(v + Math.PI / 4.0);
                double c = Math.cos(v + Math.PI / 4.0);
                double m = Math.max(Math.abs(s), Math.abs(c));
                s /= m;
                c /= m;

                fl = vd * s + vt;
                fr = vd * c - vt;
                bl = vd * c + vt;
                br = vd * s - vt;

                break;
            }
        }

        fl += correction;
        fr -= correction;
        bl += correction;
        br -= correction;

        fl = ((gamepad.left_stick_y) + gamepad.right_stick_x + gamepad.left_stick_x);
        fr = ((gamepad.left_stick_y) - gamepad.right_stick_x - gamepad.left_stick_x);
        bl = ((gamepad.left_stick_y) + gamepad.right_stick_x - gamepad.left_stick_x);
        br = ((gamepad.left_stick_y) - gamepad.right_stick_x + gamepad.left_stick_x);

        setMotorPowers(fl, fr, bl, br);

        this.telemetry.addLine("Motor powers: fl: " + fl + " fr: " + fr + " bl: " + bl + " br: " + br);
        this.telemetry.addLine("heading: " + globalAngle);
        this.telemetry.addLine("mode: " + driveMode);
        this.telemetry.update();
    }

    public void setStrafePower(double power){
        setMotorPowers(power, -power, -power, power);
    }

    private void setMotorPowers(double fl, double fr, double bl, double br){

        FL.setPower(fl);
        FR.setPower(fr);
        BL.setPower(bl);
        BR.setPower(br);

    }

    public void setMotorPowers(double power){

        FL.setPower(power);
        FR.setPower(power);
        BL.setPower(power);
        BR.setPower(power);

    }

    private void setMotorPowers(double lPower, double rPower){

        FL.setPower(lPower);
        FR.setPower(rPower);
        BL.setPower(lPower);
        BR.setPower(rPower);

    }

    public void setMotorMode(DcMotor.RunMode mode){
        FL.setMode(mode);
        FR.setMode(mode);
        BL.setMode(mode);
        BR.setMode(mode);
    }

    public void encoderDrive(double speed,
                             double inches,
                             double timeoutS) {
        int newFLTarget, newFRTarget, newBLTarget, newBRTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFLTarget = FL.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newFRTarget = FR.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newBLTarget = BL.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newBRTarget = BR.getCurrentPosition() + (int)(inches  * COUNTS_PER_INCH);

            FL.setTargetPosition(newFLTarget);
            FR.setTargetPosition(newFRTarget);
            BL.setTargetPosition(newBLTarget);
            BR.setTargetPosition(newBRTarget);

            // Turn On RUN_TO_POSITION
            setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            setMotorPowers(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (FL.isBusy() && FR.isBusy() && BL.isBusy() && BR.isBusy())) {
                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d :%7d :%7d", newFLTarget,  newFRTarget, newBLTarget, newBRTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d :%7d :%7d",
                        FL.getCurrentPosition(),
                        FR.getCurrentPosition(),
                        BL.getCurrentPosition(),
                        BR.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            setMotorPowers(0);

            // Turn off RUN_TO_POSITION
            setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

            opMode.sleep(250);   // optional pause after each move
        }
    }

    public boolean isBusy(){


        return (FL.isBusy() && FR.isBusy() && BL.isBusy() && BR.isBusy());
    }

    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    public void drive(double inches){

        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int newFLTarget, newFRTarget, newBLTarget, newBRTarget;

        newFLTarget = (int) (inches * COUNTS_PER_INCH);
        newFRTarget = (int) (inches * COUNTS_PER_INCH);
        newBLTarget = (int) (inches * COUNTS_PER_INCH);
        newBRTarget = (int) (inches * COUNTS_PER_INCH);

        while(opModeIsActive()){

            if(newFLTarget < 0){
                if ((Math.abs(FL.getCurrentPosition()) > newFLTarget) && Math.abs(FR.getCurrentPosition()) > newFRTarget) {
                    setMotorPowers(-0.5);
                } else {
                    setMotorPowers(0);
                }
            } else {
                if ((Math.abs(FL.getCurrentPosition()) < newFLTarget) && Math.abs(FR.getCurrentPosition()) < newFRTarget) {
                    setMotorPowers(0.5);
                } else {
                    setMotorPowers(0);
                }
            }

        }

    }

    public void rotate(int degrees) {
        // restart imu angle tracking.
        resetAngle();

        // start pid controller. PID controller will monitor the turn angle with respect to the
        // target angle and reduce power as we approach the target angle with a minimum of 20%.
        // This is to prevent the robots momentum from overshooting the turn after we turn off the
        // power. The PID controller reports onTarget() = true when the difference between turn
        // angle and target angle is within 2% of target (tolerance). This helps prevent overshoot.
        // The minimum power is determined by testing and must enough to prevent motor stall and
        // complete the turn. Note: if the gap between the starting power and the stall (minimum)
        // power is small, overshoot may still occur. Overshoot is dependant on the motor and
        // gearing configuration, starting power, weight of the robot and the on target tolerance.

        pidRotate.reset();
        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0, 180);
        pidRotate.setOutputRange(.20, power);
        pidRotate.setTolerance(2);
        pidRotate.enable();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        // rotate until turn is completed.

        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0)
            {
                setMotorPowers(-power, power); //left
                opMode.sleep(100);
            }

            do
            {
                power = pidRotate.performPID(getAngle()); // power will be - on right turn.
                setMotorPowers(-power, power);
            } while (opModeIsActive() && !pidRotate.onTarget());
        }
        else    // left turn.
            do
            {
                power = pidRotate.performPID(getAngle()); // power will be + on left turn.
                setMotorPowers(-power, power);
            } while (opModeIsActive() && !pidRotate.onTarget());

        // turn the motors off.
        setMotorPowers(0);

        // wait for rotation to stop.
        opMode.sleep(500);

        // reset angle tracking on new heading.
        resetAngle();
    }

    public void turnAbsoulte(double target){

        while(opModeIsActive() && !turnDone) {

            double heading = getAngle();

            double Error = heading - target;
            double Kp = 0.03;
            double leftPower;
            double rightPower;

            if ((Math.abs(Error)) > 2) {
                leftPower = Error * Kp;
                rightPower = -Error * Kp;
                leftPower = Range.clip(leftPower, -1, 1);
                rightPower = Range.clip(rightPower, -1, 1);

                FL.setPower(leftPower);
                FR.setPower(rightPower);
                BL.setPower(leftPower);
                BR.setPower(rightPower);
            } else {
                FL.setPower(0);
                FR.setPower(0);
                BL.setPower(0);
                BR.setPower(0);

                turnDone = true;
            }
            telemetry.addLine("Error: " + Error);
            telemetry.update();

        }

    }

    public void rotateTeleop(int degrees) {
        // restart imu angle tracking.
        resetAngle();

        // start pid controller. PID controller will monitor the turn angle with respect to the
        // target angle and reduce power as we approach the target angle with a minimum of 20%.
        // This is to prevent the robots momentum from overshooting the turn after we turn off the
        // power. The PID controller reports onTarget() = true when the difference between turn
        // angle and target angle is within 2% of target (tolerance). This helps prevent overshoot.
        // The minimum power is determined by testing and must enough to prevent motor stall and
        // complete the turn. Note: if the gap between the starting power and the stall (minimum)
        // power is small, overshoot may still occur. Overshoot is dependant on the motor and
        // gearing configuration, starting power, weight of the robot and the on target tolerance.

        pidRotate.reset();
        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0, 180);
        pidRotate.setOutputRange(.20, power);
        pidRotate.setTolerance(2);
        pidRotate.enable();

        // rb.drive.getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        // rotate until turn is completed.

        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (opMode.opModeIsActive() && getAngle() == 0) {
                //setMotorPowers(power, power, -power, -power);
                //opMode.sleep(100);
            }

            do {
                correction = pidRotate.performPID(getAngle()); // power will be - on right turn.
                //setMotorPowers(-power, -power, power, power);
            } while (opMode.opModeIsActive() && !pidRotate.onTarget());
        } else    // left turn.
            do {
                correction = pidRotate.performPID(getAngle()); // power will be + on left turn.
                //setMotorPowers(-power, -power, power, power);
            } while (opMode.opModeIsActive() && !pidRotate.onTarget());

        // turn the motors off.
        //setThrottle(0);


        // wait for rotation to stop.
        //opMode.sleep(500);

        // reset angle tracking on new heading.
        resetAngle();
    }

    public double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }


    public String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    public String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    @Override
    public void composeTelemetry() {

        this.telemetry.addData(hardwareName + ":", "%7d :%7d :%7d :%7d",
                FL.getCurrentPosition(),
                FR.getCurrentPosition(),
                BL.getCurrentPosition(),
                BR.getCurrentPosition());

    }

    private boolean opModeIsActive(){
        return opMode.opModeIsActive();
    }


}
