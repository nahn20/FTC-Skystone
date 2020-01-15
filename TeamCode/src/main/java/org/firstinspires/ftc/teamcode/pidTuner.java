package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

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

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

@TeleOp(name="PIDTuner", group="Pushboat")
public class pidTuner extends LinearOpMode {
    ladle ldl = new ladle();
    toggleMap toggleMap1 = new toggleMap();
    useMap useMap1 = new useMap();

    toggleMap toggleMap2 = new toggleMap();
    useMap useMap2 = new useMap();

    int fullRotationCount = 0;
    double previousAngle = 0;
    double angle = 0;
    double savedX = 0;
    double savedY = 0;
    double savedAngle = Math.PI/2;
    ////////////////
    // PID Stuffs \\

    //PID constants are [p, i, d] in order.
    double kDrive[] = {0.38, 1, 0.24}; //PID constants for linear drive power
    //{0.44, 1.27, 0.54}

    double kRotate[] = {0.84, 3.188, 0.41}; //PID constants for rotation


    double errorDrive = 0;
    double previousErrorDrive = 0;
    double errorRotate = 0;
    double previousErrorRotate = 0;
    double integralDrive = 0;
    double derivativeDrive = 0;
    double integralRotate = 0;
    double derivativeRotate = 0;

    double lastTime = 0;

    private ElapsedTime runtime = new ElapsedTime();

    //VUFORIA STUFF\\
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = true;


    private static final String VUFORIA_KEY = "AQ6M0kj/////AAAAGYsMfGvfjkGGlaBJBo84DksXdDgs4AmpEDWNkoag1HAlZ93v7JEK967tDDswHjp6gNrANoJqgPDZCawn6YEnlYzDTzaoufvvImFMlSa94j0OW28rElHTniEc0hbRblm1qEX5pHri02/FTyEAmdbKNW324ljfpHWZnwHp65Bwr8WR9vB6QxkAPJBf+0R3f9H0MuHdKVQkMBC2E97MJVy9fBc3huI5zBOrdEYIvZCf32ktKrw6uTenPZGdpJF4x4VqS4VXFrJ2w+tpWU6pHn2JZM+wLGDy8gYtKKMXKmX2Jfz1U6THFBxlFiXOojOuaFIBN9iPlWvG2AIBRNvbLw8sOW0jmhSWRvOx0bSc/QH3l8By";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;                                 // Units are degrees
    private static final float bridgeRotZ = 180;

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;

    @Override
    public void runOpMode() {
        ldl.init(hardwareMap);
        ldl.imu();
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
        parameters.cameraDirection   = CAMERA_CHOICE;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
        VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
        blueRearBridge.setName("Blue Rear Bridge");
        VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
        redRearBridge.setName("Red Rear Bridge");
        VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
        redFrontBridge.setName("Red Front Bridge");
        VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
        blueFrontBridge.setName("Blue Front Bridge");
        VuforiaTrackable red1 = targetsSkyStone.get(5);
        red1.setName("Red Perimeter 1");
        VuforiaTrackable red2 = targetsSkyStone.get(6);
        red2.setName("Red Perimeter 2");
        VuforiaTrackable front1 = targetsSkyStone.get(7);
        front1.setName("Front Perimeter 1");
        VuforiaTrackable front2 = targetsSkyStone.get(8);
        front2.setName("Front Perimeter 2");
        VuforiaTrackable blue1 = targetsSkyStone.get(9);
        blue1.setName("Blue Perimeter 1");
        VuforiaTrackable blue2 = targetsSkyStone.get(10);
        blue2.setName("Blue Perimeter 2");
        VuforiaTrackable rear1 = targetsSkyStone.get(11);
        rear1.setName("Rear Perimeter 1");
        VuforiaTrackable rear2 = targetsSkyStone.get(12);
        rear2.setName("Rear Perimeter 2");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        // Set the position of the Stone Target.  Since it's not fixed in position, assume it's at the field origin.
        // Rotated it to to face forward, and raised it to sit on the ground correctly.
        // This can be used for generic target-centric approach algorithms
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //Set the position of the bridge support targets with relation to origin (center of field)
        blueFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, bridgeRotZ)));

        blueRearBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, bridgeRotZ)));

        redFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, 0)));

        redRearBridge.setLocation(OpenGLMatrix
                .translation(bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, 0)));

        //Set the position of the perimeter targets with relation to origin (center of field)
        red1.setLocation(OpenGLMatrix
                .translation(quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        red2.setLocation(OpenGLMatrix
                .translation(-quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        front1.setLocation(OpenGLMatrix
                .translation(-halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));

        front2.setLocation(OpenGLMatrix
                .translation(-halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        blue1.setLocation(OpenGLMatrix
                .translation(-quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        blue2.setLocation(OpenGLMatrix
                .translation(quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        rear1.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));

        rear2.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //
        // Create a transformation matrix describing where the phone is on the robot.
        //
        // NOTE !!!!  It's very important that you turn OFF your phone's Auto-Screen-Rotation option.
        // Lock it into Portrait for these numbers to work.
        //
        // Info:  The coordinate frame for the robot looks the same as the field.
        // The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
        // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
        //
        // The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
        // pointing to the LEFT side of the Robot.
        // The two examples below assume that the camera is facing forward out the front of the robot.

        // We need to rotate the camera around it's long axis to bring the correct camera forward.
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90 ;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT  = 0.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot center
        final float CAMERA_VERTICAL_DISPLACEMENT = 0.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }

        while(!opModeIsActive()){
        }
        runtime.reset();
        toggleMap2.b = true;
        //toggleMap1.left_bumper = true;
        toggleMap1.x = true;
        lastTime = runtime.milliseconds();
        targetsSkyStone.activate();
        while(opModeIsActive()){

            //Priority 1
            updateKeys();
            angleOverflow();

            //Vuforia
            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible() && trackable == stoneTarget) { //I added the trackable == stoneTarget bit to only track skystones
                    telemetry.addData(">", "Sky Stone Found");
                    targetVisible = true;

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }
            if (targetVisible) {
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                telemetry.addData("X (mm)", "{X} = %.3f", translation.get(1)); //Horizontal distance from center of object. Positive -> camera is to left of object. Negative -> camera is to right of object.
				telemetry.addData("Z (mm)", "{Z} = %.3f", translation.get(0)); //Distance away from object
                //Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                //telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
            }
            else {
                telemetry.addData("Visible Target", "none");
            }

            //Priority 2
            constantModifier();
            telemetry.addData("Angle", (double) (Math.round(getHeading()*100))/100);
            if(!toggleMap1.x){
                drive();
            }

            if(toggleMap1.right_bumper){
                //telemetry.addData("Going to", 0 + " " + 0 + " " + 0);
                doublePID(0, 0, 0);
            }
            else if(toggleMap1.left_bumper){
                //telemetry.addData("Going to", savedX + " " + savedY + " " + savedAngle);
                doublePID(savedX, savedY, savedAngle);
            }
            else{
                lastTime = runtime.milliseconds();
            }
            if(gamepad1.b){
                savedAngle = getHeading();
            }
            telemetry.update();//THIS GOES AT THE END
        }
    }
    public void constantModifier(){
        //Usage:
        //B toggled on modifies kRotate. B toggled off modified kPower
        //Y toggled on modifies kp
        //X toggled on modifies ki
        //A toggled on modifies kd
        //Holding right bumper increases increment amount.
        double incrementAmount = 0.01;
        if(gamepad2.left_trigger > 0){
            incrementAmount = 0.1;
        }
        if(gamepad2.right_trigger > 0){
            incrementAmount = 0.001;
        }
        if(gamepad2.dpad_down){
            incrementAmount = -incrementAmount; //Subtracts instead if using dpad_down
        }
        if(toggleMap2.b){
            telemetry.addData("Editing", "Rotation Constants");
            // telemetry.addData("kP", kRotate[0]);
            // telemetry.addData("kI", kRotate[1]);
            // telemetry.addData("kD", kRotate[2]);
        }
        else if(!toggleMap2.b){
            telemetry.addData("Editing", "Drive Constants");
            // telemetry.addData("kP", kDrive[0]);
            // telemetry.addData("kI", kDrive[1]);
            // telemetry.addData("kD", kDrive[2]);
        }
        if(toggleMap2.y){
            telemetry.addData("Editing", "kP");
        }
        if(toggleMap2.x){
            telemetry.addData("Editing", "kI");
        }
        if(toggleMap2.a){
            telemetry.addData("Editing", "kD");
        }
        if((gamepad2.dpad_up && cdCheck(useMap2.dpad_up, 20)) || (gamepad2.dpad_down && cdCheck(useMap2.dpad_down, 20))){ //More compact this way
            if(toggleMap2.b){
                if(toggleMap2.y){
                    kRotate[0] += incrementAmount;
                }
                if(toggleMap2.x){ //Don't worry these three will never be on at the same time. Read updateKeys();
                    kRotate[1] += incrementAmount;
                }
                if(toggleMap2.a){
                    kRotate[2] += incrementAmount;
                }
            }
            else if(!toggleMap2.b){
                if(toggleMap2.y){
                    kDrive[0] += incrementAmount;
                }
                if(toggleMap2.x){ //Don't worry these three will never be on at the same time. Read updateKeys();
                    kDrive[1] += incrementAmount;
                }
                if(toggleMap2.a){
                    kDrive[2] += incrementAmount;
                }
            }
        }
        for(int i = 0; i < 3; i++){
            if(kRotate[i] < 0){
                kRotate[i] = 0;
            }
            if(kDrive[i] < 0){
                kDrive[i] = 0;
            }
        }
    }
    //I'm leaving a lot of notes labeled TODOInAutonomous because they're things I need to do once this is converted to an autonomous
    public void doublePID(double desiredX, double desiredY, double desiredAngle){
        double deltaT = (runtime.milliseconds() - lastTime); //Delta time. Subtracts last time of tick from current time
        double tempErrorDrive = errorDrive;
        double tempErrorRotate = errorRotate;
        errorDrive = 0; //Fill in
        errorRotate = angle - desiredAngle;
        //integralDrive; Just reminding myself that these are things
        //derivativeDrive;
        double pDrive = 0;
        //integralRotate;
        //derivativeRotate;
        double pRotate = 0;

        integralDrive += errorDrive*deltaT/10000;
        if(Math.abs(errorDrive) > 1){
            integralDrive = 0;
        }
        derivativeDrive = (tempErrorDrive-errorDrive)/deltaT;
        pDrive = kDrive[0]*errorDrive + kDrive[1]*integralDrive + -10*kDrive[2]*derivativeDrive;

        integralRotate += errorRotate*deltaT/10000;
        if(Math.abs(errorRotate) > Math.PI/8){
            integralRotate = 0;
        }
        derivativeRotate = (tempErrorRotate-errorRotate)/deltaT;
        pRotate = kRotate[0]*errorRotate + kRotate[1]*integralRotate + -10*kRotate[2]*derivativeRotate;

        if(toggleMap1.x){
            autoDrive(pDrive, pRotate);
        }
        lastTime = runtime.milliseconds();
    }
    public void autoDrive(double pDrive, double pRotate){
        ldl.frontLeft.setPower(pDrive + pRotate);
        ldl.backLeft.setPower(pDrive - pRotate);
        ldl.backRight.setPower(pDrive - pRotate);
        ldl.frontRight.setPower(pDrive + pRotate);
    }
    public void drive(){
        double pDrive = gamepad1.left_stick_y;
        double pRotate = gamepad1.right_stick_x/2;

        ldl.frontLeft.setPower(pDrive + pRotate);
        ldl.backLeft.setPower(pDrive - pRotate);
        ldl.backRight.setPower(pDrive - pRotate);
        ldl.frontRight.setPower(pDrive + pRotate);
    }
    public void angleOverflow(){ //Increase fullRotationCount when angle goes above 2*PI or below 0
        double heading = getHeading() - fullRotationCount*(2*Math.PI);
        //Warning: Will break if the robot does a 180 in less thank 1 tick, but that probably won't happen
        if(heading < Math.PI/2 && previousAngle > 3*Math.PI/2){
            fullRotationCount++;
        }
        if(heading > 3*Math.PI/2 && previousAngle < Math.PI/2){
            fullRotationCount--;
        }
        previousAngle = heading;
    }
    public double getHeading(){ //Includes angle subtraction, angle to radian conversion, and 180>-180 to regular system conversion
        Orientation angles = ldl.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double heading = angles.firstAngle;
        heading = (Math.PI/180)*heading;
        if(heading < 0){
            heading = (2*Math.PI) + heading;
        }
        heading += fullRotationCount*(2*Math.PI);
        return heading;
    }
    ////////////////////////////////
    // TOGGLES ////////// USE MAP //
    ////////////////////////////////
    public void updateKeys(){ //a, x, and y are conflicting keys
        if(gamepad2.b && cdCheck(useMap2.b, 500)){
            toggleMap2.b = toggle(toggleMap2.b);
            useMap2.b = runtime.milliseconds();
        }
        if(gamepad1.x && cdCheck(useMap1.x, 500)){
            toggleMap1.x = toggle(toggleMap1.x);
            useMap1.x = runtime.milliseconds();
        }
        if(gamepad1.y && cdCheck(useMap1.y, 500)){
            toggleMap1.y = toggle(toggleMap1.y);
            useMap1.y = runtime.milliseconds();
        }
        if(gamepad2.a && cdCheck(useMap2.a, 500)){
            toggleMap2.a = toggle(toggleMap2.a);
            useMap2.a = runtime.milliseconds();
            toggleMap2.y = false;
            toggleMap2.x = false;
        }
        if(gamepad2.x && cdCheck(useMap2.x, 500)){
            toggleMap2.x = toggle(toggleMap2.x);
            useMap2.x = runtime.milliseconds();
            toggleMap2.y = false;
            toggleMap2.a = false;
        }
        if(gamepad2.y && cdCheck(useMap2.y, 500)){
            toggleMap2.y = toggle(toggleMap2.y);
            useMap2.y = runtime.milliseconds();
            toggleMap2.x = false;
            toggleMap2.a = false;
        }
        if((gamepad1.left_bumper || gamepad2.left_bumper) && cdCheck(useMap1.left_bumper, 500)){ //Bumpers on both controllers do the same thing
            toggleMap1.left_bumper = toggle(toggleMap1.left_bumper);
            useMap1.left_bumper = runtime.milliseconds();
            toggleMap1.right_bumper = false;
        }
        if((gamepad1.right_bumper || gamepad2.right_bumper) && cdCheck(useMap1.right_bumper, 500)){
            toggleMap1.right_bumper = toggle(toggleMap1.right_bumper);
            useMap1.right_bumper = runtime.milliseconds();
            toggleMap1.left_bumper = false;
        }
        if(gamepad1.left_stick_button){
            useMap1.left_stick_button = runtime.milliseconds();
        }
        if(gamepad2.left_stick_button){
            useMap2.left_stick_button = runtime.milliseconds();
        }
    }
    public boolean cdCheck(double key, int cdTime){
        return runtime.milliseconds() - key > cdTime;
    }
    public boolean toggle(boolean variable){
        if(variable == true){
            variable = false;
        }
        else if(variable == false){
            variable = true;
        }
        return variable;
    }
}
