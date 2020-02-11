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
@TeleOp(name="AbsoluteTeleOp", group="Pushboat")
public class AbsoluteTeleOp extends LinearOpMode {
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
        startTime = runtime.milliseconds();
        while(opModeIsActive()){
            updateKeys();
            //Player 1
            drive();
            suc();
            //Player 2
            slide();
            flerp();
			// telemetry.addData("frontLeft", ldl.frontLeft.getPower());
			// telemetry.addData("backLeft", ldl.backLeft.getPower());
			// telemetry.addData("backRight", ldl.backRight.getPower());
			// telemetry.addData("frontRight", ldl.frontRight.getPower());
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
    public void slide(){
		if(gamepad2.a){ //Low load position
			uwuLockPos = 0;
		}
		if(toggleMap2.a && ldl.uwu.getCurrentPosition() < ldl.uwuContained){
			shoopLockPos = 0;
			zhoopLockPos = 0;
			if(ldl.shoop.getCurrentPosition() < 20){
				toggleMap2.a = false;
			}
		}
		if(gamepad2.x){ //Medium load position
			shoopLockPos = ldl.shoopClearance;
			zhoopLockPos = ldl.zhoopClearance;
		}
		if(toggleMap2.x){
			uwuLockPos = ldl.uwuDeposit;
			if(Math.abs(uwuLockPos-ldl.uwuDeposit) < 20){
				toggleMap2.x = false;
			}
		}
		if(gamepad2.y){ //High load position
			shoopLockPos = ldl.shoopMax;
			zhoopLockPos = ldl.zhoopMax;
		}
		if(gamepad2.b){
			uwuLockPos = 0;
		}
		shoopLockPos += Math.round(-gamepad2.right_stick_y*ldl.shoopMax*0.005);
		zhoopLockPos += Math.round(-gamepad2.right_stick_y*ldl.zhoopMax*0.005);
		uwuLockPos += Math.round(gamepad2.right_stick_x*ldl.uwuMax*0.007);
		if(Math.abs(gamepad2.right_stick_x) > 0.3){
			toggleMap2.x = false;
		}

		shoopLockPos = Range.clip(shoopLockPos, 0, ldl.shoopMax);
		zhoopLockPos = Range.clip(zhoopLockPos, 0, ldl.zhoopMax);
		uwuLockPos = Range.clip(uwuLockPos, 0, ldl.uwuMax);

		ldl.shoop.setTargetPosition(shoopLockPos);
		ldl.zhoop.setTargetPosition(zhoopLockPos);
		boolean verticalClearance = ldl.shoop.getCurrentPosition() > ldl.shoopMinClearance && ldl.zhoop.getCurrentPosition() > ldl.zhoopMinClearance;
		if((verticalClearance || (!verticalClearance && uwuLockPos > ldl.uwuContained) || gamepad2.left_bumper)){
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
    ////////////////////////////////
    // TOGGLES ////////// USE MAP //
    //[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[

    public void updateKeys(){
		telemetry.addData("left_bumper", gamepad1.left_bumper);
		if(gamepad1.left_bumper && cdCheck(useMap1.left_bumper, 200)){ 
            toggleMap1.left_bumper = toggle(toggleMap1.left_bumper);
            useMap1.left_bumper = runtime.milliseconds();
        }
		if(gamepad2.x && cdCheck(useMap2.x, 200)){ 
            toggleMap2.x = toggle(toggleMap2.x);
			toggleMap2.a = false;
            useMap2.x = runtime.milliseconds();
        }
		if(gamepad2.a && cdCheck(useMap2.a, 200)){ 
            toggleMap2.a = toggle(toggleMap2.a);
			toggleMap2.x = false;
            useMap2.a = runtime.milliseconds();
        }
		if(gamepad2.dpad_left && cdCheck(useMap2.dpad_left, 200)){ 
            toggleMap2.dpad_left = toggle(toggleMap2.dpad_left);
            useMap2.dpad_left = runtime.milliseconds();
        }
		if(gamepad2.dpad_right || Math.abs(gamepad2.left_stick_x) > 0.1){
			toggleMap2.dpad_left = false;
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


