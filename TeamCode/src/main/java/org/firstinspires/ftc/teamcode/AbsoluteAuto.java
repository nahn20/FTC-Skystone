package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
//Shoop Kinda High Pos: 851
//Zhoop Kinda High Pos: 730


/*COPY & PASTE REFERENCE
//Time Limited Loop Action
    double start = runtime.milliseconds();
    while(runtime.milliseconds() - start < 200 && opModeIsActive()){
        doStuff();
    }
 */


@Autonomous(name="AbsoluteAuto", group="Pushboat")
public class AbsoluteAuto extends LinearOpMode {
    ladle ldl = new ladle();
    private ElapsedTime runtime = new ElapsedTime();

    int fullRotationCount = 0;
    double previousAngle = 0;
    double angle = 0;

    double lastTime = 0;

    double startTime = 0;

    double skyX = 0; //Relative to robot
    double skyZ = 0;

    int[] motorPos1 = new int[4];
    int[] motorPos2 = new int[4];
    //VUFORIA STUFF\\
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = true;


    private static final String VUFORIA_KEY = "AQ6M0kj/////AAAAGYsMfGvfjkGGlaBJBo84DksXdDgs4AmpEDWNkoag1HAlZ93v7JEK967tDDswHjp6gNrANoJqgPDZCawn6YEnlYzDTzaoufvvImFMlSa94j0OW28rElHTniEc0hbRblm1qEX5pHri02/FTyEAmdbKNW324ljfpHWZnwHp65Bwr8WR9vB6QxkAPJBf+0R3f9H0MuHdKVQkMBC2E97MJVy9fBc3huI5zBOrdEYIvZCf32ktKrw6uTenPZGdpJF4x4VqS4VXFrJ2w+tpWU6pHn2JZM+wLGDy8gYtKKMXKmX2Jfz1U6THFBxlFiXOojOuaFIBN9iPlWvG2AIBRNvbLw8sOW0jmhSWRvOx0bSc/QH3l8By";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch = 25.4f;
    private static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    private boolean targetVisible = false;
    private float phoneXRotate = 0;
    private float phoneYRotate = 0;
    private float phoneZRotate = 0;

    @Override
    public void runOpMode() {
        ldl.init(hardwareMap);
        ldl.imu();

		ldl.resetDrive();
        ldl.runWithoutEncoderDrive();

        //VUFORIA STUFF\\
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CAMERA_CHOICE;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");

        // Set the position of the Stone Target.  Since it's not fixed in position, assume it's at the field origin.
        // Rotated it to to face forward, and raised it to sit on the ground correctly.
        // This can be used for generic target-centric approach algorithms
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        // We need to rotate the camera around it's long axis to bring the correct camera forward.
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT = 0.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot center
        final float CAMERA_VERTICAL_DISPLACEMENT = 0.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the stoneTarget listener know where the phone is.  */
        ((VuforiaTrackableDefaultListener) stoneTarget.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);

        while (!opModeIsActive()) {

        }
        startTime = runtime.milliseconds();
		targetsSkyStone.activate();
		//Start\\

		slideFuckExtend();

        //This part does stuff to get to the blocks
		motorPos1 = saveMotorPos(); //Saves motor pos to return to later
		scanForSkyStone(1, stoneTarget); //Slowly drives sideways to find sky stone
        motorPos2 = saveMotorPos();
        attackSkyStone();
        returnToMotorPos(motorPos2);
		returnToMotorPos(motorPos1);
        //This part brings the blocks back to the platform

    }
    public boolean detectTarget(VuforiaTrackable target) {
        if (((VuforiaTrackableDefaultListener)target.getListener()).isVisible()) {
            OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)target.getListener()).getUpdatedRobotLocation();
            if (robotLocationTransform != null) {
                VectorF translation = robotLocationTransform.getTranslation();
                //telemetry.addData("X (mm)", "{X} = %.3f", translation.get(1)); //Horizontal distance from center of object. Positive -> camera is to left of object. Negative -> camera is to right of object.
                //telemetry.addData("Z (mm)", "{Z} = %.3f", translation.get(0)); //Distance away from object
                skyX = translation.get(1);
                skyZ = translation.get(0);
                return true;
            }
        }
        return false;
    }
    public void scanForSkyStone(int direction, VuforiaTrackable stoneTarget) {
        ldl.runWithoutEncoderDrive();
        autoDrive(-Math.PI/2 + direction*Math.PI/2, 0.4, 0);
        double start = runtime.milliseconds();
        boolean isFound = false;
        while(runtime.milliseconds() - start < 10000 && !isFound && opModeIsActive()){
            isFound = detectTarget(stoneTarget);
        }
        ldl.powerDrive(0);
        if(isFound) {
            telemetry.addData(">", "Found Target!");
            telemetry.update();
        }
        else{
            telemetry.addData(">", "Stone not found");
            telemetry.addData(">", "Fuck it, we're going straight");
            telemetry.update();
            skyZ = -300;
        }
    }
    public void attackSkyStone(){
        ldl.runWithoutEncoderDrive();
        double theta = Math.atan2(skyZ, skyX);
        double power = 0.8;
        ldl.succ.setPower(1);
        ldl.succc.setPower(1);
        autoDrive(theta, power, 0);
        sleepNotSleep(2000);
        ldl.powerDrive(0);
        ldl.succ.setPower(0);
        ldl.succc.setPower(0);
    }
	public void slideFuckExtend(){
		ldl.shoop.setTargetPosition(851);
		ldl.zhoop.setTargetPosition(730);
		ldl.shoop.setPower(1);
		ldl.zhoop.setPower(1);
		ldl.uwu.setPower(-1);
		sleepNotSleep(1600);
		ldl.shoop.setTargetPosition(0);
		ldl.zhoop.setTargetPosition(0);
		ldl.uwu.setPower(0);
		sleepNotSleep(500);
		ldl.shoop.setPower(0);
		ldl.zhoop.setPower(0);
	}
	public void slideFuckRetract(){
		ldl.shoop.setTargetPosition(851);
		ldl.zhoop.setTargetPosition(730);
		ldl.shoop.setPower(1);
		ldl.zhoop.setPower(1);
		sleepNotSleep(500);

		ldl.uwu.setPower(1);
		sleepNotSleep(700);
		ldl.uwu.setPower(0);
		sleepNotSleep(300);

		ldl.shoop.setTargetPosition(0);
		ldl.zhoop.setTargetPosition(0);
		sleepNotSleep(500);
		ldl.shoop.setPower(0);
		ldl.zhoop.setPower(0);
	}
    public void returnToMotorPos(int[] motorSave){ //Drives the robot to a certain set of encoder values
        ldl.runToPosDrive();
        ldl.frontLeft.setTargetPosition(motorSave[0]);
        ldl.backLeft.setTargetPosition(motorSave[1]);
        ldl.backRight.setTargetPosition(motorSave[2]);
        ldl.frontRight.setTargetPosition(motorSave[3]);
        ldl.powerDrive(1);
        double start = runtime.milliseconds();
        while(Math.abs(ldl.frontLeft.getCurrentPosition()-motorSave[0]) > 30 && runtime.milliseconds() - start < 3000 && opModeIsActive()){
        }
        ldl.powerDrive(0);
    }
    public int[] saveMotorPos(){ //Saves encoder values for future use
        int[] motorSave = new int[4];
        motorSave[0] = ldl.frontLeft.getCurrentPosition();
        motorSave[1] = ldl.backLeft.getCurrentPosition();
        motorSave[2] = ldl.backRight.getCurrentPosition();
        motorSave[3] = ldl.frontRight.getCurrentPosition();
        return motorSave;
    }
    public void rotate(double theta){ //Rotation is relative to starting gyro angle
        double kp = 1;
        double error = angle-getHeading();
        double start = runtime.milliseconds();
        while(error > 0.1 && runtime.milliseconds() - start < 3000 && opModeIsActive()){
            ldl.mecanumDrive(0, 0, kp*error);
        }
    }
    public void autoDrive(double theta, double magnitude, double pRot) {
        double modifiedTheta = theta + Math.PI / 4; //Got rid of a - angle here

        magnitude *= (1 - Math.abs(pRot)); //Multiplied by (1-Protate) so it doesn't go over 1 with rotating
        double pX = magnitude * Math.cos(modifiedTheta);
        double pY = magnitude * Math.sin(modifiedTheta);

        ldl.mecanumDrive(pX, pY, pRot);
    }
    public void sleepNotSleep(double time){ //Time in milliseconds
        double start = runtime.milliseconds();
        while(runtime.milliseconds() - start < time && opModeIsActive()){
        }
    }
    public void angleOverflow() { //Increase fullRotationCount when angle goes above 2*PI or below 0
        double heading = getHeading() - fullRotationCount * (2 * Math.PI);
        //Warning: Will break if the robot does a 180 in less thank 1 tick, but that probably won't happen
        if (heading < Math.PI / 2 && previousAngle > 3 * Math.PI / 2) {
            fullRotationCount++;
        }
        if (heading > 3 * Math.PI / 2 && previousAngle < Math.PI / 2) {
            fullRotationCount--;
        }
        previousAngle = heading;
    }

    public double getHeading() { //Includes angle subtraction, angle to radian conversion, and 180>-180 to regular system conversion
        Orientation angles = ldl.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double heading = angles.firstAngle;
        heading = (Math.PI / 180) * heading;
        if (heading < 0) {
            heading = (2 * Math.PI) + heading;
        }
        heading += fullRotationCount * (2 * Math.PI);
        return heading;
    }
}


