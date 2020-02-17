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
@TeleOp(name="StatesTeleOp", group="Pushboat")
public class StatesTeleOp extends LinearOpMode {
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

	private int stackHeight = 0;
	private int[] shoopStackHeight = new int[10];
	private int[] zhoopStackHeight = new int[10];
	shoopStackHeight[0] = 10;
	shoopStackHeight[1] = 10;
	shoopStackHeight[2] = 10;
	shoopStackHeight[3] = 10;
	shoopStackHeight[4] = 10;
	shoopStackHeight[5] = 10;
	shoopStackHeight[6] = 10;
	shoopStackHeight[7] = 10;
	shoopStackHeight[8] = 10;
	shoopStackHeight[9] = 10;
	zhoopStackHeight[0] = shoopStackHeight[0];
	zhoopStackHeight[1] = shoopStackHeight[1];
	zhoopStackHeight[2] = shoopStackHeight[2];
	zhoopStackHeight[3] = shoopStackHeight[3];
	zhoopStackHeight[4] = shoopStackHeight[4];
	zhoopStackHeight[5] = shoopStackHeight[5];
	zhoopStackHeight[6] = shoopStackHeight[6];
	zhoopStackHeight[7] = shoopStackHeight[7];
	zhoopStackHeight[8] = shoopStackHeight[8];
	zhoopStackHeight[9] = shoopStackHeight[9];


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
        startTime = runtime.milliseconds();
        while(opModeIsActive()){
            updateKeys();
            //Player 1
            drive();
            //Player 2
			resetEncoders();
			telemetry.update();
        }
    }
    //Player 1
    public void drive() {
        double stick_x = -gamepad1.left_stick_x;
        double stick_y = gamepad1.left_stick_y;
        double pX = 0;
        double pY = 0;
        double pRot = 0;
        if (toggleMap1.x) {
            stick_x *= 0.5;
            stick_y *= 0.5;
        }
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
            pRot = -(gamepad1.right_trigger-gamepad1.left_trigger);
            ldl.mecanumDrive(pX, pY, pRot);
        } else {
            pRot = -0.6 * (gamepad1.right_trigger-gamepad1.left_trigger);
            if (gamepad1.left_stick_y == 0 && gamepad1.left_stick_x == 0) {
                pRot = -(gamepad1.right_trigger-gamepad1.left_trigger);
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
    public void suc(){
        if(toggleMap1.left_bumper && gamepad1.right_trigger == 0){
            ldl.succ.setPower(1);
			telemetry.addData("Intake", "ON");
        }
        else if(gamepad1.left_trigger > 0){
            ldl.succ.setPower(gamepad1.left_trigger);
			telemetry.addData("Intake", "ON");
        }
        if(gamepad1.right_trigger > 0){
            ldl.succ.setPower(-gamepad1.right_trigger);
			telemetry.addData("Intake", "OUTTAKING");
        }
		if(gamepad1.left_trigger == 0 && gamepad1.right_trigger == 0 && !toggleMap1.left_bumper){
			ldl.succ.setPower(0);
			telemetry.addData("Intake", "OFF");
		}
    }
    //Player 2
	int shoopLockPos = 0;
	int zhoopLockPos = 0;
	int uwuLockPos = 0;
	int weirdRetractionVar = 0;
	int weirdRetractionStage = 0;
    public void slide(){
		int shoopPos = ldl.shoop.getCurrentPosition();
		int zhoopPos = ldl.zhoop.getCurrentPosition();
		int uwuPos = ldl.uwu.getCurrentPosition();
		boolean tooLow = (shoopPos < ldl.shoopBoundsClearance || zhoopPos < ldl.zhoopBoundsClearance);
		if(Math.abs(gamepad1.right_stick_y) > 0 || Math.abs(gamepad1.right_stick_x) > 0){{
			toggleMap1.y = false;
			toggleMap1.a = false;
			shoopLockPos += Math.round(-gamepad1.right_stick_y*ldl.shoopMax*0.005);
			zhoopLockPos += Math.round(-gamepad1.right_stick_y*ldl.zhoopMax*0.005);
			uwuLockPos += Math.round(gamepad1.right_stick_x*ldl.uwuMax*0.007);
		}
		if(gamepad1.y){ //Auto stack
			toggleMap1.y = true;
			toggleMap1.a = false;
			if(shoopStackHeight[stackHeight] < uwu.shoopClearance || zhoopStackHeight[stackHeight] < uwu.zhoopClearance){ //If the thing has to lift, extend, then lower
				shoopLockPos = shoopClearance;
				zhoopLockPos = zhoopClearance;
				uwuLockPos = ldl.uwuDeposit;
			}
			else{ //Regular lift
				shoopLockPos = shoopStackHeight[stackHeight];
				zhoopLockPos = zhoopStackHeight[zhoopHeight];
			}
		}
		if(toggleMap1.y){
			if(shoopStackHeight[stackHeight] < uwu.shoopClearance || zhoopStackHeight[stackHeight] < uwu.zhoopClearance){ //If the thing has to lift, extend, then lower
				if(uwuPos > ldl.uwuBoundsMinOut){
					shoopLockPos = shoopStackHeight[stackHeight];
					zhoopLockPos = zhoopStackHeight[zhoopHeight];
					toggleMap1.y = false;
					stackHeight++;
				}
			}
			else{ //Regular lift
				if(Math.abs(shoopPos-shoopStackHeight[stackHeight]) < 20 && Math.abs(zhoopPos-zhoopStackHeight[stackHeight]) < 20){
					uwuLockPos = ldl.uwuDeposit;
					toggleMap1.y = false;
					stackHeight++;
				}
			}
		}
		if(gamepad1.a){
			toggleMap1.a = true;
			toggleMap1.y = false;
			weirdRetractionStage = 0; //Used for determining which stage of the 3 step process this is
			shoopLockPos = shoopPos + 100;
			zhoopLockPos = zhoopPos + 100;
			weirdRetractionVar = shoopLockPos;
		}
		if(toggleMap1.a){
			if(weirdRetractionStage == 0){
				if(Math.abs(shoopPos-weirdRetractionVar) < 20){
					uwuLockPos = 0;
					weirdRetractionStage = 1;
				}
			}
			if(weirdRetractionStage == 1){
				if(uwuPos < ldl.uwuBoundsMinOut){
					shoopLockPos = ldl.shoopMin;
					zhoopLockPos = ldl.zhoopMin;
					toggleMap1.a = false;
				}
			}
		}

		//Setting positions
		if(!gamepad2.y){
			shoopLockPos = Range.clip(shoopLockPos, 0, ldl.shoopMax);
			zhoopLockPos = Range.clip(zhoopLockPos, 0, ldl.zhoopMax);
			uwuLockPos = Range.clip(uwuLockPos, 0, ldl.uwuMax);
			if(uwuLockPos < ldl.uwuBoundsMinOut && uwuPos > ldl.uwuBoundsMinOut && tooLow){ //Horizontal slide trying to retract but vertical slides are too low
				ldl.shoop.setTargetPosition(shoopLockPos);
				ldl.zhoop.setTargetPosition(zhoopLockPos);
				ldl.uwu.setTargetPosition(ldl.uwuBarelyOut);
			}
			else if(uwuLockPos > ldl.uwuBoundsContained && uwuPos < ldl.uwuBoundsContained && tooLow){ //Horizontal slide trying to extend (assumes a block is held) but vertical slides are too low
				ldl.shoop.setTargetPosition(shoopLockPos);
				ldl.zhoop.setTargetPosition(zhoopLockPos);
				ldl.uwu.setTargetPosition(0);
			}
			else if(uwuPos > ldl.uwuBoundsMinOut && uwuPos < ldl.uwuBoundsContained && tooLow){
				telemetry.addData("WARNING", "Slides going places they shouldn't");
				ldl.shoop.setTargetPosition(ldl.shoopClearance);
				ldl.zhoop.setTargetPosition(ldl.zhoopClearance);
				ldl.uwu.setTargetPosition(uwuLockPos);
			}
			else if((shoopLockPos < ldl.shoopBoundsClearance || zhoopLockPos < ldl.shoopBoundsClearance) && uwuPos < ldl.uwuBoundsMinOut && uwuPos > ldl.uwuBoundsContained){ //Vertical slide trying to lower but horizontal slide is in that awkward middle space
				ldl.shoop.setTargetPosition(ldl.shoopClearance);
				ldl.zhoop.setTargetPosition(ldl.zhoopClearance);
				ldl.uwu.setTargetPosition(uwuLockPos);
			}
			else{
				ldl.shoop.setTargetPosition(shoopLockPos);
				ldl.zhoop.setTargetPosition(zhoopLockPos);
				ldl.uwu.setTargetPosition(uwuLockPos);
			}
		}
		else if(gamepad2.y){ //Override mode
			ldl.shoop.setTargetPosition(shoopLockPos);
			ldl.zhoop.setTargetPosition(zhoopLockPos);
			ldl.uwu.setTargetPosition(uwuLockPos);
		}
    }
	double holdPower = 0;
    public void flerp(){
		if(!toggleMap2.dpad_left){
			ldl.crGrap.setPower(gamepad2.left_stick_x);
		}
		if(toggleMap2.dpad_left){
			ldl.crGrap.setPower(-1);
		}
    }
	public void resetEncoders(){
		if(gamepad2.y && gamepad2.b){
			if(!toggleMap2.b){
				useMap2.b = runtime.milliseconds(); //Using useMap weirdly here
			}
			toggleMap2.b = true; //Using toggleMap weirdly here
			telemetry.addData("WARNING", "RESETTING ENCODERS");
			if(cdCheck(useMap2.b, 2000)){
				ldl.resetSlides();
			}
		}
		else{
			toggleMap2.b = false;
		}
	}
    ////////////////////////////////
    // TOGGLES ////////// USE MAP //
    //[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[

    public void updateKeys(){
		if(gamepad1.left_bumper && cdCheck(useMap1.left_bumper, 200)){ 
            toggleMap1.left_bumper = toggle(toggleMap1.left_bumper);
            useMap1.left_bumper = runtime.milliseconds();
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


