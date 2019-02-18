package org.firstinspires.ftc.teamcode.core.hardware;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.Dogeforia;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.core.util.ENUMS;
import org.firstinspires.ftc.teamcode.core.util.RobotConstants;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

public class CV extends  BaseHardware{

    private ElapsedTime runtime = new ElapsedTime();
    private static final float mmPerInch = 25.4f;
    private static final float mmFTCFieldWidth = (12 * 6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Select which camera you want use.  The FRONT camera is the one on the same side as the screen.
    // Valid choices are:  BACK or FRONT
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = FRONT;

    private OpenGLMatrix lastLocation = null;
    boolean targetVisible;
    Dogeforia vuforia;
    WebcamName webcamName;
    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();

    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private String hardwareName;

    private GoldAlignDetector detector;
    private ENUMS.GoldLocation goldLocation = ENUMS.GoldLocation.UNKNOWN;

    public CV(HardwareMap hardwareMap, Telemetry telemetry){

        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        init();

    }

    private void init(){

        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters vuforiaParameters = new VuforiaLocalizer.Parameters();

        vuforiaParameters.vuforiaLicenseKey = "AVLeWaD/////AAABmaCnSNdyYU2IrXuTK6ZXofgXezmDHUWmdN65PXvQbYrt89npFHJWC5yBgXMWvuHQJfgkJg3f5/U4ZwnCdlNiuIuW+eOi7Nhkob8UlCzcciD8tieNsPD+sYd4O/E/v2WFIDdgx9o4rcAcAM8gHu48dGUrlRQsfruhANKUk1RE6h0/xFJMKimqs0/7U3z6aoEoRnPajrSBifkIVpePsARuwDhlaszzsbc2vijwg+dkIVcEMckjPAk5YoPi7i30N8/vrmhrvdNaVUUzCFPsGo42ad3U/XmZwb7xKAEpU03goBoRQIjBgJegGWNBkKw7So8ilLSU9uHAYtZoiRyFnUy8YcVyU7YkTjA7ixzrDpV5Zah/";
        vuforiaParameters.fillCameraMonitorViewParent = true;

        vuforiaParameters.cameraName = webcamName;

        vuforia = new Dogeforia(vuforiaParameters);
        vuforia.enableConvertFrameToBitmap();

        VuforiaTrackables targetsRoverRuckus = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
        VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
        blueRover.setName("Blue-Rover");
        VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
        redFootprint.setName("Red-Footprint");
        VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
        frontCraters.setName("Front-Craters");
        VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
        backSpace.setName("Back-Space");

        //For convenience, gather together all the trackable objects in one easily-iterable collection

        allTrackables.addAll(targetsRoverRuckus);

        OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix
                .translation(0, mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
        blueRover.setLocation(blueRoverLocationOnField);

        OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix
                .translation(0, -mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
        redFootprint.setLocation(redFootprintLocationOnField);

        OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix
                .translation(-mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90));
        frontCraters.setLocation(frontCratersLocationOnField);

        OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix
                .translation(mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
        backSpace.setLocation(backSpaceLocationOnField);

        //--------------------------------------------------------------------------------------------------
        // CHANGE these when you can
        final int CAMERA_FORWARD_DISPLACEMENT = 110;   // eg: Camera is 110 mm in front of robot center
        final int CAMERA_VERTICAL_DISPLACEMENT = 200;   // eg: Camera is 200 mm above ground
        final int CAMERA_LEFT_DISPLACEMENT = 0;     // eg: Camera is ON the robot's center line
        //--------------------------------------------------------------------------------------------------

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                        CAMERA_CHOICE == FRONT ? 90 : -90, 0, 0));

        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, vuforiaParameters.cameraDirection);
        }

        targetsRoverRuckus.activate();

        detector = new GoldAlignDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance(), 0, true);
        detector.useDefaults();

        detector.downscale = RobotConstants.DOWNSCALE; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = RobotConstants.WEIGHT;

        vuforia.setDogeCVDetector(detector);
        vuforia.enableDogeCV();
        vuforia.showDebug();
        vuforia.start();

    }

    /**
     * this doesnt need to exist for this class
     * @param gamepad
     */
    @Override
    public void controlSystem(Gamepad gamepad) {
        //no need
    }

    /**
     *
     * @return gold location in GoldLocation and null hopefully never
     */
    public ENUMS.GoldLocation getGoldLocation(){

        double xPos = getGoldX();

        if(detector.isFound()) {
            if (xPos > -10 && xPos < 180) {
                goldLocation = ENUMS.GoldLocation.LEFT;
            } else if (xPos >= 180 && xPos < 420) {
                goldLocation = ENUMS.GoldLocation.CENTER;
            } else if (xPos >= 420 && xPos < 610) {
                goldLocation = ENUMS.GoldLocation.RIGHT;
            }
        } else {
            goldLocation =  ENUMS.GoldLocation.UNKNOWN;
        }

        return goldLocation;
    }

    /**
     *
     * @return gets the gold on-screen x position 0-600
     */
    public double getGoldX(){
        return detector.getXPosition();
    }

    /**
     *
     * @return if a gold block is seen
     */
    public boolean isGoldFound(){

        return detector.isFound();
    }

    @Override
    public void composeTelemetry() {

        this.telemetry.addLine("cv: " + goldLocation);

    }
}
