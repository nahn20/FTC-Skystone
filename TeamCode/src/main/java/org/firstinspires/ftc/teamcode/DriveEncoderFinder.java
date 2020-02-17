package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@TeleOp(name="DriveEncoderFinder", group="Pushboat")
public class DriveEncoderFinder extends LinearOpMode {
    ladle ldl = new ladle();
    toggleMap toggleMap1 = new toggleMap();
    useMap useMap1 = new useMap();

    toggleMap toggleMap2 = new toggleMap();
    useMap useMap2 = new useMap();
    ////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    /* V * A * R * I * A * B * E * S *////* V * A * R * I * A * B * E * S */
    ////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////

    //int chungoidPos = 0;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode(){
        ldl.init(hardwareMap);
        ldl.imu();

        ldl.runWithoutEncoderDrive();

        double startTime = 0;
        
        while(!opModeIsActive()){
            telemetry.addData("shoop position", ldl.shoop.getCurrentPosition());
            telemetry.addData("zhoop position", ldl.zhoop.getCurrentPosition());
			telemetry.addData("uwu position", ldl.uwu.getCurrentPosition());
            telemetry.update();
        }
		ldl.shoop.setPower(1);
		ldl.zhoop.setPower(1);
		ldl.uwu.setPower(1);

		ldl.resetDrive();
		if(toggleMap1.y){
			ldl.runWithoutEncoderDrive();
		}
		else{
			ldl.runToPosDrive();
		}
        startTime = runtime.milliseconds();
        while(opModeIsActive()){
            updateKeys();
            //Player 1
			if(toggleMap1.y){
				telemetry.addData("Drive", "Manual");
				drive();
			}
			else{
				telemetry.addData("Drive", "Encoder");
				encoderDrive(0.5);
			}
			reset();
			telemetry.addData("fl", ldl.frontLeft.getCurrentPosition());
			telemetry.addData("bl", ldl.backLeft.getCurrentPosition());
			telemetry.addData("br", ldl.backRight.getCurrentPosition());
			telemetry.addData("fr", ldl.frontRight.getCurrentPosition());
			telemetry.update();
        }
    }
    //Player 1
	double flPos = 0;
	double blPos = 0;
	double brPos = 0;
	double frPos = 0;
	public void encoderDrive(double power){
        double stick_x = -gamepad1.left_stick_x;
        double stick_y = gamepad1.left_stick_y;
        double pX = 0;
        double pY = 0;
        double pRot = 0;
        double theta = Math.atan2(stick_y, stick_x); //Arctan2 doesn't have bad range restriction
        if (toggleMap1.right_bumper || gamepad1.dpad_up || gamepad1.dpad_right || gamepad1.dpad_left || gamepad1.dpad_down) {
            //CCCCCCC      HHH      HHH           AA           DDDDDDD \\
            //C            HHH      HHH          AAAA          DDD   DD\\
            //C            HHHHHHHHHHHH         AA  AA         DDD   DD\\
            //C            HHHHHHHHHHHH        AAAAAAAA        DDD   DD\\
            //C            HHH      HHH       AA      AA       DDD   DD\\
            //CCCCCCC      HHH      HHH      AA        AA      DDDDDDD \\
            double mag = 0.25;
            if (gamepad1.dpad_up || (toggleMap1.right_bumper && theta > Math.PI / 4 && theta <= 3 * Math.PI / 4)) {
                pX = mag;
                pY = -mag;
            } else if (gamepad1.dpad_left || (toggleMap1.right_bumper && (theta > 3 * Math.PI / 4 || theta <= -3 * Math.PI / 4))) {
                pX = 2*mag;
                pY = 2*mag;
            } else if (gamepad1.dpad_down || (toggleMap1.right_bumper && theta < -Math.PI / 4 && theta >= -3 * Math.PI / 4)) {
                pX = -mag;
                pY = mag;
            } else if (gamepad1.dpad_right || (toggleMap1.right_bumper && theta > -Math.PI / 4 && theta <= Math.PI / 4 && !(stick_y == 0 && stick_x == 0))) {
                pX = -2*mag;
                pY = -2*mag;
            } else if (toggleMap1.right_bumper) {
                pX = 0;
                pY = 0;
            }
            pRot = -gamepad1.right_stick_x;
        } else {
            pRot = -0.6 * gamepad1.right_stick_x;
            if (gamepad1.left_stick_y == 0 && gamepad1.left_stick_x == 0) {
                pRot = -gamepad1.right_stick_x;
            }
            //double gyroAngle = getHeading(); //In radiants, proper rotation, yay!!11!!
            double gyroAngle = 0;
            double magnitudeMultiplier = 0;

            // if(!toggleMap1.left_bumper){ //Removes gyroAngle from the equation meaning the robot drives normally
            //     gyroAngle = 0;
            // }
            double modifiedTheta = theta + Math.PI / 4 - gyroAngle;

            double thetaInFirstQuad = Math.abs(Math.atan(stick_y / stick_x)); //square to circle conversion
            if (thetaInFirstQuad > Math.PI / 4) {
                magnitudeMultiplier = Math.sin(thetaInFirstQuad); //Works because we know y is 1 when theta > Math.pi/4
            } else if (thetaInFirstQuad <= Math.PI / 4) {
                magnitudeMultiplier = Math.cos(thetaInFirstQuad); //Works because we know x is 1 when theta < Math.pi/4
            }
            double magnitude = Math.sqrt(Math.pow(stick_x, 2) + Math.pow(stick_y, 2)) * magnitudeMultiplier * (1 - Math.abs(pRot)); //Multiplied by (1-pRot) so it doesn't go over 1 with rotating
            pX = magnitude * Math.cos(modifiedTheta);
            pY = magnitude * Math.sin(modifiedTheta);
        }
		flPos += (pY + pRot);
		blPos += (pX - pRot);
		brPos += (pY - pRot);
		frPos += (pX + pRot);
		ldl.frontLeft.setTargetPosition(flPos);
		ldl.backLeft.setTargetPosition(flPos);
		ldl.backRight.setTargetPosition(flPos);
		ldl.frontRight.setTargetPosition(flPos);
		ldl.powerDrive(power);
	}
    public void drive() {
        double stick_x = -gamepad1.left_stick_x;
        double stick_y = gamepad1.left_stick_y;
        double pX = 0;
        double pY = 0;
        double pRot = 0;
        double theta = Math.atan2(stick_y, stick_x); //Arctan2 doesn't have bad range restriction
        if (toggleMap1.right_bumper || gamepad1.dpad_up || gamepad1.dpad_right || gamepad1.dpad_left || gamepad1.dpad_down) {
            //CCCCCCC      HHH      HHH           AA           DDDDDDD \\
            //C            HHH      HHH          AAAA          DDD   DD\\
            //C            HHHHHHHHHHHH         AA  AA         DDD   DD\\
            //C            HHHHHHHHHHHH        AAAAAAAA        DDD   DD\\
            //C            HHH      HHH       AA      AA       DDD   DD\\
            //CCCCCCC      HHH      HHH      AA        AA      DDDDDDD \\
            double mag = 0.25;
            if (gamepad1.dpad_up || (toggleMap1.right_bumper && theta > Math.PI / 4 && theta <= 3 * Math.PI / 4)) {
                pX = mag;
                pY = -mag;
            } else if (gamepad1.dpad_left || (toggleMap1.right_bumper && (theta > 3 * Math.PI / 4 || theta <= -3 * Math.PI / 4))) {
                pX = 2*mag;
                pY = 2*mag;
            } else if (gamepad1.dpad_down || (toggleMap1.right_bumper && theta < -Math.PI / 4 && theta >= -3 * Math.PI / 4)) {
                pX = -mag;
                pY = mag;
            } else if (gamepad1.dpad_right || (toggleMap1.right_bumper && theta > -Math.PI / 4 && theta <= Math.PI / 4 && !(stick_y == 0 && stick_x == 0))) {
                pX = -2*mag;
                pY = -2*mag;
            } else if (toggleMap1.right_bumper) {
                pX = 0;
                pY = 0;
            }
            pRot = -gamepad1.right_stick_x;
            ldl.mecanumDrive(pX, pY, pRot);
        } else {
            pRot = -0.6 * gamepad1.right_stick_x;
            if (gamepad1.left_stick_y == 0 && gamepad1.left_stick_x == 0) {
                pRot = -gamepad1.right_stick_x;
            }
            //double gyroAngle = getHeading(); //In radiants, proper rotation, yay!!11!!
            double gyroAngle = 0;
            double magnitudeMultiplier = 0;

            // if(!toggleMap1.left_bumper){ //Removes gyroAngle from the equation meaning the robot drives normally
            //     gyroAngle = 0;
            // }
            double modifiedTheta = theta + Math.PI / 4 - gyroAngle;

            double thetaInFirstQuad = Math.abs(Math.atan(stick_y / stick_x)); //square to circle conversion
            if (thetaInFirstQuad > Math.PI / 4) {
                magnitudeMultiplier = Math.sin(thetaInFirstQuad); //Works because we know y is 1 when theta > Math.pi/4
            } else if (thetaInFirstQuad <= Math.PI / 4) {
                magnitudeMultiplier = Math.cos(thetaInFirstQuad); //Works because we know x is 1 when theta < Math.pi/4
            }
            double magnitude = Math.sqrt(Math.pow(stick_x, 2) + Math.pow(stick_y, 2)) * magnitudeMultiplier * (1 - Math.abs(pRot)); //Multiplied by (1-pRot) so it doesn't go over 1 with rotating
            pX = magnitude * Math.cos(modifiedTheta);
            pY = magnitude * Math.sin(modifiedTheta);

            ldl.mecanumDrive(pX, pY, pRot);
        }
    }
	public void reset(){
		if(gamepad1.b){
			ldl.resetDrive();
			if(toggleMap1.y){
				ldl.runWithoutEncoderDrive();
			}
			else{
				ldl.runToPosDrive();
			}
		}
	}
    ////////////////////////////////
    // TOGGLES ////////// USE MAP //
    //[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[

    public void updateKeys(){
		if(gamepad2.left_bumper && cdCheck(useMap2.left_bumper, 200)){ 
            toggleMap2.left_bumper = toggle(toggleMap2.left_bumper);
            useMap2.left_bumper = runtime.milliseconds();
        }
		if(gamepad2.right_bumper && cdCheck(useMap2.right_bumper, 200)){ 
            toggleMap2.right_bumper = toggle(toggleMap2.right_bumper);
            useMap2.right_bumper = runtime.milliseconds();
        }
		if(gamepad1.y && cdCheck(useMap1.y, 200)){ 
            toggleMap1.y = toggle(toggleMap1.y);
            useMap1.y = runtime.milliseconds();
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


