package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotor;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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


@Autonomous(name="AutoWithoutVuforia", group="main")
public class AutoWithoutVuforia extends LinearOpMode {
    ladle ldl = new ladle();
    private ElapsedTime runtime = new ElapsedTime();

    int fullRotationCount = 0;
    double previousAngle = 0;
    double angle = 0;

    double lastTime = 0;

    double startTime = 0;

    int[] pos = new int[4];
    int[] motorPos1 = new int[4];
    int[] motorPos2 = new int[4];

	String allianceColor = "blue";

    @Override
    public void runOpMode() {
        AutoTransitioner.transitionOnStop(this, "StatesTeleOp");
        ldl.init(hardwareMap);
        ldl.imu();
		ldl.resetSlides();


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
		//Drives in a straight line 2 tiles



        //driveTo(-1250, 1250, -1250, 1250, 0.3);
		driveTo(-300, 300, -300, 300, 0.3);

		/*
		if(allianceColor == "red"){
			int straight = 400;
			driveTo(-straight, straight, -straight, straight, 0.4); //Drives to block line from wall
			rotate(Math.PI/2, 2000);
			straight = 400; //Untested
			driveTo(-straight, straight, -straight, straight, 0.4);
			rotate(0, 2000);
			straight = 850; //Untested
			driveTo(-straight, straight, -straight, straight, 0.4);
			rotate(Math.PI/2, 2000);
		}
		if(allianceColor == "blue"){
			int front = 1250;
			driveTo(-front, front, -front, front, 0.4); //Drives to block line from wall
			rotate(Math.PI/2, 2000);
		}
		//Extend and grab
		slideFuckExtend(300);
		sleepNotSleep(300);
		findAndGrabSkyStone();
		ldl.shoop.setPower(0);
		ldl.zhoop.setPower(0);
		ldl.uwu.setPower(0);
		ldl.resetDrive();
		ldl.runToPosDrive();
		if(allianceColor == "red"){
			rotate(Math.PI, 2000);
			int front = 600;
			driveTo(-front, front, -front, front, 0.4); //Drives to block line from wall
			rotate(Math.PI/2, 2000);
		}
		if(allianceColor == "blue"){
			rotate(Math.PI, 2000);
			int front = 600;
			driveTo(-front, front, -front, front, 0.4); //Drives to block line from wall
			rotate(3*Math.PI/2, 2000);
		}
		ldl.shoop.setPower(1);
		ldl.zhoop.setPower(1);
		ldl.uwu.setPower(1);
		shoopZhoopLift(ldl.shoopClearance, ldl.zhoopClearance, 2000);
		uwuExtend(ldl.uwuDeposit, 2000);
		shoopZhoopLift(ldl.shoopMin, ldl.zhoopMin, 2000);
		if(allianceColor == "red"){
			ldl.resetDrive();
			ldl.runToPosDrive();
			int back = -1635;
			driveTo(-back, back, -back, back, 0.4);
		}
		if(allianceColor == "blue"){
			ldl.resetDrive();
			ldl.runToPosDrive();
			int back = -1635;
			driveTo(-back, back, -back, back, 0.4);
		}
		ldl.crGrap.setPower(1);
		sleepNotSleep(1.5*ldl.grabReleaseTime);
		ldl.crGrap.setPower(0);
		if(allianceColor == "red"){

		}
		if(allianceColor == "blue"){
			shoopZhoopLift(180, 180, 2000);
			int front = -1400;
			driveTo(-front, front, -front, front, 0.4);
			shoopZhoopLift(0, 0, 2000);
			front = -900;
			driveTo(-front, front, -front, front, 0.4);
		}
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
		slideFuckExtend(100);
        while(Math.abs(ldl.frontLeft.getCurrentPosition()-fl) > 30 && Math.abs(ldl.backLeft.getCurrentPosition()-bl) > 30 && Math.abs(ldl.backRight.getCurrentPosition()-br) > 30 && Math.abs(ldl.frontRight.getCurrentPosition()-fr) > 30 && runtime.milliseconds() - start < 5000 && opModeIsActive()){
        }
        ldl.powerDrive(0);
	}
    public void slideFuckExtend(int lowerAmount){
		shoopZhoopLift(ldl.shoopClearance, ldl.zhoopClearance, 2000);
		uwuExtend(ldl.uwuBarelyOut, 2000);
		shoopZhoopLift(lowerAmount, lowerAmount, 500);
    }
    public void slideFuckRetract(){
		shoopZhoopLift(ldl.shoopClearance, ldl.zhoopClearance, 2000);
		uwuExtend(0, 2000);
		sleepNotSleep(300);
		shoopZhoopLift(ldl.shoopMin, ldl.zhoopMin, 2000);
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
	public void driveTime(String direction, double mag, double pRot, double time){
		ldl.runWithoutEncoderDrive();
		double start = runtime.milliseconds();
		double pX = 0;
		double pY = 0;
		if (direction == "forward") {
			pX = mag;
			pY = -mag;
		} else if (direction == "left") {
			pX = mag;
			pY = mag;
		} else if (direction == "backward") {
			pX = -mag;
			pY = mag;
		} else if (direction == "right") {
			pX = -mag;
			pY = -mag;
		}
		else{
			telemetry.addData(">", "Oops, you're dumb");
			telemetry.update();
		}
		ldl.mecanumDrive(pX, pY, pRot);
		while(runtime.milliseconds()-start < time){}
		ldl.powerDrive(0);
		ldl.resetDrive();
		ldl.runToPosDrive();
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
		sleepNotSleep(200);
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
    public void rotate(double angle, int time){ //Rotation is relative to starting gyro angle. + for ccw, - for cw
	//Positive is ccw
        double kp = 0.5;
		double ki = 0.03;
		angleOverflow();
        double error = getHeading()-angle;
		double previousError = error;
		double integral = 0;
        double start = runtime.milliseconds();
		ldl.runWithoutEncoderDrive();
        while(runtime.milliseconds() - start < time && opModeIsActive()){
			angleOverflow();
			error = getHeading()-angle;
			if((error<0 && previousError>0) || (error>0 && previousError<0)){
				integral = 0;
			}
			double pRot = -(kp*error+ki*integral);
			if(Math.abs(error) < 0.4){
				integral += error;
			}
			previousError = error;
			telemetry.addData("error", error);
			telemetry.addData("p", -kp*error);
			telemetry.addData("i", -ki*integral);
			telemetry.update();
            ldl.mecanumDrive(0, 0, pRot);
        }
		ldl.powerDrive(0);
		ldl.resetDrive();
		ldl.runToPosDrive();
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
	float hsvValues[] = {0F, 0F, 0F};
	final float values[] = hsvValues;
	final double SCALE_FACTOR = 255;
	public boolean isSkyStone() {
		Color.RGBToHSV((int) (ldl.glitchColor.red() * SCALE_FACTOR), (int) (ldl.glitchColor.green() * SCALE_FACTOR), (int) (ldl.glitchColor.blue() * SCALE_FACTOR), hsvValues);
		/*
		telemetry.addData("Alpha", ldl.glitchColor.alpha());
		telemetry.addData("Red  ", ldl.glitchColor.red());
		telemetry.addData("Green", ldl.glitchColor.green());
		telemetry.addData("Blue ", ldl.glitchColor.blue());
		telemetry.addData("Hue", hsvValues[0]);
		telemetry.addData("Saturation", hsvValues[1]); //0 gray > 1 primary color
		telemetry.addData("Value", hsvValues[2]); //0 black > 100 bright
		*/

		telemetry.addData("Hue", hsvValues[0]);
		telemetry.addData("Distance", ldl.glitchDist.getDistance(DistanceUnit.MM));
		telemetry.update();
		if(hsvValues[0] > 100 && ldl.glitchDist.getDistance(DistanceUnit.MM) < 45){
			return true;
		}
		return false;
	}
	public boolean isLine(){
		Color.RGBToHSV((int) (ldl.lineSensor.red() * SCALE_FACTOR), (int) (ldl.lineSensor.green() * SCALE_FACTOR), (int) (ldl.lineSensor.blue() * SCALE_FACTOR), hsvValues);
		telemetry.addData("Hue", hsvValues[0]);
		telemetry.update();
				//Blue (200)									Red (35)
		if(Math.abs(hsvValues[0]-200) < 20 || Math.abs(hsvValues[0]-35) < 20){
			return true;
		}
		return false;
	}
	public void driveToXDistAway(double dist){ //Used for being within a certain dist of stone
		ldl.runWithoutEncoderDrive();
		double start = runtime.milliseconds();
		
		ldl.runWithoutEncoderDrive();

		while(ldl.glitchDist.getDistance(DistanceUnit.MM) > 300 && runtime.milliseconds() - start < 2000){
			double mag = 0.2;
			double pX = -mag;
			double pY = mag;
			ldl.mecanumDrive(pX, pY, 0);
		}
		start = runtime.milliseconds();
		while(ldl.glitchDist.getDistance(DistanceUnit.MM) > dist && runtime.milliseconds() - start < 1000){
			double mag = 0.5;
			double pX = -mag;
			double pY = -mag;
			ldl.mecanumDrive(pX, pY, 0);
		}
		ldl.powerDrive(0);
		ldl.resetDrive();
		ldl.runToPosDrive();
	}
	public void findAndGrabSkyStone() {
		// driveToXDistAway(20);
		// sleepNotSleep(500);
		//Finds the skystone
		int pos = ldl.uwu.getCurrentPosition();
		double start = runtime.milliseconds();
		ldl.uwu.setPower(1);
		ldl.uwu.setTargetPosition(ldl.uwuMax);
		int count = 0;
		double start2 = runtime.milliseconds();
		while(count < 3 && pos < ldl.uwuMax && runtime.milliseconds() - start < 5000 && opModeIsActive()){
			boolean isFound = isSkyStone();
			while(!isFound && pos < ldl.uwuMax && runtime.milliseconds() - start < 5000 && opModeIsActive()){
				/* //Missing withoutEncoderDrive
				double dist = 20;
				double sensorDist = ldl.glitchDist.getDistance(DistanceUnit.MM);
				double error = sensorDist - dist;
				double kp = 0.1;
				double mag = error*kp; //Not actually mag--sometimes negative
				double pX = 0;
				double pY = 0;
				mag = Range.clip(mag, -0.5, 0.5);
				if(allianceColor == "blue"){
					pX = -mag;
					pY = -mag;
				}
				if(allianceColor == "red"){
					pX = mag;
					pY = mag;
				}
				ldl.mecanumDrive(pX, pY, 0);
				*/
				pos = ldl.uwu.getCurrentPosition();
				isFound = isSkyStone();
			}
			if(isFound){
				count++;
			}
		}
		if(count < 3){
			telemetry.addData(">", "Failed Search");
			telemetry.update();
		}
		//Extends a bit extra to center itself on skystone and lowers onto SkyStone
		pos = ldl.uwu.getCurrentPosition();
		if(pos > 800){
			pos += 110; //Additional shift factor here
			pos = Range.clip(pos, 0, ldl.uwuMax);
		}
		//ldl.uwu.setPower(1); //Add this if above power isn't 1
		uwuExtend(pos, 2000);
		shoopZhoopLift(ldl.shoopMin, ldl.zhoopMin, 2000);
		//Grabs skystone
		ldl.crGrap.setPower(-1);
		sleepNotSleep(1500);
		//Pull out game strongk
		shoopZhoopLift(ldl.shoopClearance, ldl.zhoopClearance, 2000);
		uwuExtend(380, 4000);
	}
}