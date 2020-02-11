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


@Autonomous(name="AutoWithoutVuforia", group="Pushboat")
public class AutoWithoutVuforia extends LinearOpMode {
    ladle ldl = new ladle();
    private ElapsedTime runtime = new ElapsedTime();

    int fullRotationCount = 0;
    double previousAngle = 0;
    double angle = 0;

    double lastTime = 0;

    double startTime = 0;

    double skyX = 0; //Relative to robot
    double skyZ = 0;

    int[] pos = new int[4];
    int[] motorPos1 = new int[4];
    int[] motorPos2 = new int[4];

    @Override
    public void runOpMode() {
        ldl.init(hardwareMap);
        ldl.imu();

        ldl.resetDrive();
		ldl.frontLeft.setTargetPosition(0);
		ldl.backLeft.setTargetPosition(0);
		ldl.backRight.setTargetPosition(0);
		ldl.frontRight.setTargetPosition(0);
        ldl.runToPosDrive();


        while (!opModeIsActive()) {

        }
		ldl.shoop.setPower(1);
		ldl.zhoop.setPower(1);
		ldl.uwu.setPower(1);
        startTime = runtime.milliseconds();
        //Start\\
		// rotate(-Math.PI);
		//Version 1

        //driveTo(-1508, 1309, -2903, 1234, 0.3); //Drives in a straight line 2 tiles
		driveTo(836, 848, 662, 703, 0.5); //Side drive
		ldl.resetDrive();
		driveWhileExtend(1014, -1044, 955, -984, 0.2);
		// slideFuckExtend();
		sleepNotSleep(5000);
		slideFuckRetract();
		while(opModeIsActive()){}

		//Version 2
		/*
		driveTo(1056, -1091, 1068, -1060, 0.3);
		slideFuckExtend();
		//driveTo(-806, 774, -779, 808); //Values if reset
		driveTo(0, 0, 0, 0, 0.3);
		// rotate(-Math.PI/2);
		// sleepNotSleep(1000);
		*/


		// driveTo(-728, 935, 694, -392, 0.3);
		// slideFuckRetract();
/*
        //This part does stuff to get to the blocks
        motorPos1 = saveMotorPos(); //Saves motor pos to return to later
        scanForSkyStone(1, stoneTarget); //Slowly drives sideways to find sky stone
        motorPos2 = saveMotorPos();
        attackSkyStone();
        returnToMotorPos(motorPos2);
        returnToMotorPos(motorPos1);
        //This part brings the blocks back to the platform
        */
    }
	public void driveWhileExtend(int fl, int bl, int br, int fr, double p){
        ldl.runToPosDrive();
        ldl.frontLeft.setTargetPosition(fl);
        ldl.backLeft.setTargetPosition(bl);
        ldl.backRight.setTargetPosition(br);
        ldl.frontRight.setTargetPosition(fr);
        ldl.powerDrive(p);
		double start = runtime.milliseconds();
		slideFuckExtend();
        while(Math.abs(ldl.frontLeft.getCurrentPosition()-fl) > 30 && Math.abs(ldl.backLeft.getCurrentPosition()-bl) > 30 && Math.abs(ldl.backRight.getCurrentPosition()-br) > 30 && Math.abs(ldl.frontRight.getCurrentPosition()-fr) > 30 && runtime.milliseconds() - start < 5000 && opModeIsActive()){
        }
        ldl.powerDrive(0);
	}
    public void slideFuckExtend(){
		shoopZhoopLift(ldl.shoopClearance, ldl.zhoopClearance, 2000);
		uwuExtend(ldl.uwuBarelyOut, 2000);
		shoopZhoopLift(ldl.shoopLoad, ldl.zhoopLoad, 500);
    }
    public void slideFuckRetract(){
		shoopZhoopLift(ldl.shoopClearance, ldl.zhoopClearance, 2000);
		uwuExtend(0, 2000);
		sleepNotSleep(300);
		shoopZhoopLift(ldl.shoopLoad, ldl.zhoopLoad, 2000);
    }
	public void shoopZhoopLift(int shoopPos, int zhoopPos, double timeout){
		ldl.shoop.setTargetPosition(shoopPos);
		ldl.zhoop.setTargetPosition(zhoopPos);
		double start = runtime.milliseconds();
		while(Math.abs(ldl.shoop.getCurrentPosition()-shoopPos) > 40 && Math.abs(ldl.zhoop.getCurrentPosition()-zhoopPos) > 40 && runtime.milliseconds() - start < timeout && opModeIsActive()){}
	}
	public void uwuExtend(int uwuPos, double timeout){
		ldl.uwu.setTargetPosition(uwuPos);
		double start = runtime.milliseconds();
		while(Math.abs(ldl.uwu.getCurrentPosition()-uwuPos) > 20 && runtime.milliseconds() - start < timeout && opModeIsActive()){}
	}
	
    public void driveTo(int fl, int bl, int br, int fr, double p){ //Drives the robot to a certain set of encoder values
        ldl.runToPosDrive();
        ldl.frontLeft.setTargetPosition(fl);
        ldl.backLeft.setTargetPosition(bl);
        ldl.backRight.setTargetPosition(br);
        ldl.frontRight.setTargetPosition(fr);
        ldl.powerDrive(p);
        double start = runtime.milliseconds();
        while(Math.abs(ldl.frontLeft.getCurrentPosition()-fl) > 30 && Math.abs(ldl.backLeft.getCurrentPosition()-bl) > 30 && Math.abs(ldl.backRight.getCurrentPosition()-br) > 30 && Math.abs(ldl.frontRight.getCurrentPosition()-fr) > 30 && runtime.milliseconds() - start < 3000 && opModeIsActive()){
        }
        ldl.powerDrive(0);
    }
    public void strafePull(int direction){
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
    public void rotate(double angle){ //Rotation is relative to starting gyro angle
	//Positive is ccw
        double kp = 1;
        double error = getHeading()-angle;
        double start = runtime.milliseconds();
        while(runtime.milliseconds() - start < 2000 && opModeIsActive()){
			telemetry.addData("error", error);
			telemetry.update();
			error = getHeading()-angle;
            ldl.mecanumDrive(0, 0, kp*error);
        }
		ldl.powerDrive(0);
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


