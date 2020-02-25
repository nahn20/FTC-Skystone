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
@TeleOp(name="StatesTeleOp", group="main")
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
	// int[] shoopStackHeight = new int[]{450, 805, 1185, 1510, 1890, 2250, 2630, 3000, 3000, 3000};
	// int[] zhoopStackHeight = new int[]{450, 805, 1185, 1510, 1890, 2250, 2630, 3000, 3000, 3000};
	int[] shoopStackHeight = new int[]{650, 1005, 1385, 1710, 2090, 2450, 2830, 3030, 3500, 4350};
	int[] zhoopStackHeight = new int[]{650, 1005, 1385, 1710, 2090, 2450, 2830, 3030, 3500, 4350};
	////////////////////////////////////0    1    2     3     4     5     6     7     8     9 //789 untested
	int shoopLockPos = 0;
	int zhoopLockPos = 0;
	int uwuLockPos = 0;

    @Override
    public void runOpMode(){
        ldl.init(hardwareMap);
        ldl.imu();

        ldl.runWithoutEncoderDrive();
		//Check if slides hold position when transitioned
        double startTime = 0;
		//ldl.resetSlides(); //Delete me
        while(!opModeIsActive()){
            telemetry.addData("shoop position", ldl.shoop.getCurrentPosition());
            telemetry.addData("zhoop position", ldl.zhoop.getCurrentPosition());
			telemetry.addData("uwu position", ldl.uwu.getCurrentPosition());
			resetEncoders();
            telemetry.update();
        }
		if(ldl.uwu.getCurrentPosition() < ldl.uwuBoundsContained){
			uwuLockPos = 0;
		}
		else{
			uwuLockPos = ldl.uwuDeposit;
		}
		shoopLockPos = ldl.shoopMin;
		zhoopLockPos = ldl.zhoopMin;
        startTime = runtime.milliseconds();
        while(opModeIsActive()){
            updateKeys();
			drive();
			suc();
			slide();
			flerp();
			resetEncoders();
			updateStack();
			telemetry.update();
        }
		ldl.shoop.setTargetPosition(0);
		ldl.zhoop.setTargetPosition(0);
		ldl.uwu.setTargetPosition(0);
    }
    //Player 1
    public void drive() {
        double stick_x = -gamepad1.left_stick_x;
        double stick_y = gamepad1.left_stick_y;
        double pX = 0;
        double pY = 0;
        double pRot = 0;
		double rotMultiplier = 0.6;
        double theta = Math.atan2(stick_y, stick_x); //Arctan2 doesn't have bad range restriction
        if (gamepad1.dpad_up || gamepad1.dpad_right || gamepad1.dpad_left || gamepad1.dpad_down) {
            //CCCCCCC      HHH      HHH           AA           DDDDDDD \\
            //C            HHH      HHH          AAAA          DDD   DD\\
            //C            HHHHHHHHHHHH         AA  AA         DDD   DD\\
            //C            HHHHHHHHHHHH        AAAAAAAA        DDD   DD\\
            //C            HHH      HHH       AA      AA       DDD   DD\\
            //CCCCCCC      HHH      HHH      AA        AA      DDDDDDD \\
            double mag = 0.25;
            if (gamepad1.dpad_up) {
                pX = mag;
                pY = -mag;
            } else if (gamepad1.dpad_left) {
                pX = 2*mag;
                pY = 2*mag;
            } else if (gamepad1.dpad_down) {
                pX = -mag;
                pY = mag;
            } else if (gamepad1.dpad_right) {
                pX = -2*mag;
                pY = -2*mag;
			}
            pRot = -rotMultiplier*(gamepad1.right_trigger-gamepad1.left_trigger);
            ldl.mecanumDrive(pX, pY, pRot);
        } else {
            pRot = -0.6 * rotMultiplier*(gamepad1.right_trigger-gamepad1.left_trigger);
            if (gamepad1.left_stick_y == 0 && gamepad1.left_stick_x == 0) {
                pRot = -rotMultiplier*(gamepad1.right_trigger-gamepad1.left_trigger);
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
		if(gamepad2.left_trigger > 0.1){ //Manual override by player 2
			ldl.succ.setPower(-gamepad2.left_trigger);
			telemetry.addData("Intake", "Override, ON");
		}
		else if(gamepad2.right_trigger > 0.1){
			ldl.succ.setPower(gamepad2.right_trigger);
			telemetry.addData("Intake", "Override, REVERSE");
		}
		//Player 1 control
		else if(toggleMap1.left_bumper){
			ldl.succ.setPower(-1);
			telemetry.addData("Intake", "ON");
		}
		else if(toggleMap1.right_bumper){
			ldl.succ.setPower(1);
			telemetry.addData("Intake", "REVERSE");
		}
		else{
			ldl.succ.setPower(0);
			telemetry.addData("Intake", "OFF");
		}
    }
	int weirdRetractionVar = 0;
	int weirdRetractionStage = 0;
    public void slide(){
		int shoopPos = ldl.shoop.getCurrentPosition();
		int zhoopPos = ldl.zhoop.getCurrentPosition();
		int uwuPos = ldl.uwu.getCurrentPosition();
		boolean tooLow = (shoopPos < ldl.shoopBoundsClearance || zhoopPos < ldl.zhoopBoundsClearance);
		if(Math.abs(gamepad1.right_stick_y + gamepad2.right_stick_y) > 0 || Math.abs(gamepad1.right_stick_x + gamepad2.right_stick_x) > 0){
			toggleMap1.y = false;
			toggleMap1.a = false;
			shoopLockPos += Math.round(-(gamepad1.right_stick_y+gamepad2.right_stick_y)*ldl.shoopMax*0.005);
			zhoopLockPos += Math.round(-(gamepad1.right_stick_y+gamepad2.right_stick_y)*ldl.zhoopMax*0.005);
			if(!(gamepad1.right_stick_x + gamepad2.right_stick_x > 0 && shoopLockPos < ldl.shoopBoundsClearance && zhoopLockPos < ldl.shoopBoundsClearance) || gamepad2.y){
				uwuLockPos += Math.round((gamepad1.right_stick_x+gamepad2.right_stick_x)*ldl.uwuMax*0.007);
			}
		}
		if(gamepad1.y){ //Auto stack
			toggleMap1.y = true;
			toggleMap1.a = false;
			if(shoopStackHeight[stackHeight] < ldl.shoopClearance || zhoopStackHeight[stackHeight] < ldl.zhoopClearance){ //If the thing has to lift, extend, then lower
				shoopLockPos = ldl.shoopClearance;
				zhoopLockPos = ldl.zhoopClearance;
				uwuLockPos = ldl.uwuDeposit;
			}
			else{ //Regular lift
				shoopLockPos = shoopStackHeight[stackHeight];
				zhoopLockPos = zhoopStackHeight[stackHeight];
			}
		}
		if(toggleMap1.y){
			if(shoopStackHeight[stackHeight] < ldl.shoopClearance || zhoopStackHeight[stackHeight] < ldl.zhoopClearance){ //If the thing has to lift, extend, then lower
				if(uwuPos > ldl.uwuBarelyOut){
					shoopLockPos = shoopStackHeight[stackHeight];
					zhoopLockPos = zhoopStackHeight[stackHeight];
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
			if(uwuPos > ldl.uwuBarelyOut){
				shoopLockPos = shoopPos + 100;
				zhoopLockPos = zhoopPos + 100;
				if(shoopPos < ldl.shoopBoundsClearance){
					shoopLockPos = ldl.shoopClearance;
					zhoopLockPos = ldl.zhoopClearance;
				}
				weirdRetractionVar = shoopLockPos;
			}
			else{
				weirdRetractionStage = 1;
			}
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
			toggleMap1.y = false;
			toggleMap1.a = false;
			ldl.shoop.setTargetPosition(shoopLockPos);
			ldl.zhoop.setTargetPosition(zhoopLockPos);
			ldl.uwu.setTargetPosition(uwuLockPos);
		}
		if(gamepad2.a){
			ldl.shoop.setPower(1);
			ldl.zhoop.setPower(1);
			ldl.uwu.setPower(1);
		}
		else{
			ldl.shoop.setPower(0);
			ldl.zhoop.setPower(0);
			ldl.uwu.setPower(0);
		}
    }
	double holdPower = 0;
    public void flerp(){
		if(Math.abs(gamepad2.left_stick_x) > 0.05){
			ldl.crGrap.setPower(gamepad2.left_stick_x);
			toggleMap1.b = false;
			toggleMap1.x = false;
		}
		//Player 1 stuff
		else if(toggleMap1.b){
			ldl.crGrap.setPower(1);
			if(cdCheck(useMap1.b, ldl.grabReleaseTime)){ //Release time for grabber
				toggleMap1.b = false;
			}
		}
		else if(toggleMap1.x){
			ldl.crGrap.setPower(-0.5);
		}
		else{
			ldl.crGrap.setPower(0);
		}
    }
	public void resetEncoders(){
		//Telemetry - put into a button later
		telemetry.addData("moop average", (ldl.shoop.getCurrentPosition()+ldl.zhoop.getCurrentPosition())/2);
		telemetry.addData("shoop position", ldl.shoop.getCurrentPosition());
		telemetry.addData("zhoop position", ldl.zhoop.getCurrentPosition());
		telemetry.addData("uwu position", ldl.uwu.getCurrentPosition());
		if(gamepad2.y && gamepad2.b){
			if(!toggleMap2.b){
				useMap2.b = runtime.milliseconds(); //Using useMap weirdly here
			}
			toggleMap2.b = true; //Using toggleMap weirdly here
			if(cdCheck(useMap2.b, 2000)){
				telemetry.addData(">", "Encoders Successfully Reset");
				ldl.resetSlides();
			}
			else{
				telemetry.addData("WARNING", "RESETTING ENCODERS");
			}
		}
		else{
			toggleMap2.b = false;
		}
	}
	public void updateStack(){
		if(gamepad2.x){
			stackHeight = 0;
		}
		if(gamepad2.dpad_up && cdCheck(useMap2.dpad_up, 200)){
			useMap2.dpad_up = runtime.milliseconds();
			stackHeight++;
		}
		if(gamepad2.dpad_down && cdCheck(useMap2.dpad_down, 200)){
			useMap2.dpad_down = runtime.milliseconds();
			stackHeight--;
		}
		if(stackHeight >= 10){
			stackHeight = 0;
		}
		telemetry.addData("Stack Height", stackHeight);
	}
    ////////////////////////////////
    // TOGGLES ////////// USE MAP //
    //[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[

    public void updateKeys(){
		if(gamepad1.left_bumper && cdCheck(useMap1.left_bumper, 500)){ 
            toggleMap1.left_bumper = toggle(toggleMap1.left_bumper);
			useMap1.left_bumper = runtime.milliseconds();
			toggleMap1.right_bumper = false;
		}
		if(gamepad1.right_bumper && cdCheck(useMap1.right_bumper, 500)){ 
            toggleMap1.right_bumper = toggle(toggleMap1.right_bumper);
			useMap1.right_bumper = runtime.milliseconds();
			toggleMap1.left_bumper = false;
		}
		if(gamepad1.x && cdCheck(useMap1.x, ldl.grabReleaseTime)){ 
            toggleMap1.x = toggle(toggleMap1.x);
			useMap1.x = runtime.milliseconds();
		}
		if(gamepad1.b && cdCheck(useMap1.b, 500)){ 
            toggleMap1.b = toggle(toggleMap1.b);
			useMap1.b = runtime.milliseconds();
			toggleMap1.x = false;
		}
		if(gamepad2.left_bumper){
			toggleMap1.left_bumper = false;
			toggleMap1.right_bumper = false;
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


