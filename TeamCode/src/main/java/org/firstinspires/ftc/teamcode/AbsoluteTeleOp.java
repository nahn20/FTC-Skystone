package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
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
            telemetry.update();
        }
		ldl.shoop.setPower(1);
		ldl.zhoop.setPower(1);
        startTime = runtime.milliseconds();
        while(opModeIsActive()){
            updateKeys();
            //Player 1
            drive();
            suc();
            //Player 2
            slide();
            flerp();
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
            if (gamepad1.dpad_up || (toggleMap1.right_bumper && theta > Math.PI / 4 && theta <= 3 * Math.PI / 4)) {
                pX = -1;
                pY = 1;
            } else if (gamepad1.dpad_left || (toggleMap1.right_bumper && (theta > 3 * Math.PI / 4 || theta <= -3 * Math.PI / 4))) {
                pX = -1;
                pY = -1;
            } else if (gamepad1.dpad_down || (toggleMap1.right_bumper && theta < -Math.PI / 4 && theta >= -3 * Math.PI / 4)) {
                pX = 1;
                pY = -1;
            } else if (gamepad1.dpad_right || (toggleMap1.right_bumper && theta > -Math.PI / 4 && theta <= Math.PI / 4 && !(stick_y == 0 && stick_x == 0))) {
                pX = 1;
                pY = 1;
            } else if (toggleMap1.right_bumper) {
                pX = 0;
                pY = 0;
            }
            pRot = -gamepad1.right_stick_x;
            ldl.frontLeft.setPower(pY - pRot);
            ldl.backLeft.setPower(pX + pRot);
            ldl.backRight.setPower(pY + pRot);
            ldl.frontRight.setPower(pX - pRot);
        } else {
            pRot = 0.6 * gamepad1.right_stick_x;
            if (gamepad1.left_stick_y == 0 && gamepad1.left_stick_x == 0) {
                pRot = gamepad1.right_stick_x;
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

            ldl.frontLeft.setPower(pY + pRot);
            ldl.backLeft.setPower(pX - pRot);
            ldl.backRight.setPower(pY - pRot);
            ldl.frontRight.setPower(pX + pRot);
        }
    }
    public void suc(){
        if(gamepad1.right_bumper){
            ldl.succ.setPower(1);
            ldl.succc.setPower(1);
        }
        else if(gamepad1.left_bumper){
            ldl.succ.setPower(-1);
            ldl.succc.setPower(-1);
        }
        else if(gamepad1.left_trigger > 0){
            ldl.succ.setPower(gamepad1.left_trigger);
            ldl.succc.setPower(gamepad1.left_trigger);
        }
        else if(gamepad1.right_trigger > 0){
            ldl.succ.setPower(gamepad1.right_trigger);
            ldl.succc.setPower(gamepad1.right_trigger);
        }
        else{
            ldl.succ.setPower(0);
            ldl.succc.setPower(0);
        }
    }
    //Player 2
	int shoopLockPos = 0;
	int zhoopLockPos = 0;
    public void slide(){
        int shoopMax = 2097; //Fill me in later dadddy OwO
		int zhoopMax = 1974;
        ldl.uwu.setPower(-gamepad2.left_stick_x);
		if(!toggleMap2.left_bumper){ //Manual
			ldl.shoop.setTargetPosition(Math.round(Math.abs(gamepad2.left_stick_y)*shoopMax));
			ldl.zhoop.setTargetPosition(Math.round(Math.abs(gamepad2.left_stick_y)*zhoopMax));
		}
		if(toggleMap2.left_bumper){ //Locked
			ldl.shoop.setTargetPosition(shoopLockPos);
			ldl.zhoop.setTargetPosition(zhoopLockPos);
		}
		if(gamepad2.left_bumper){ //Lock slides
			shoopLockPos = ldl.shoop.getCurrentPosition();
			zhoopLockPos = ldl.zhoop.getCurrentPosition();
        }
    }
    public void flerp(){
        if(toggleMap2.y){
            ldl.iwantdie.setPosition(1.0);
            ldl.shootme.setPosition(0.0);
        }
        else{
            ldl.iwantdie.setPosition(0.0);
            ldl.shootme.setPosition(1.0);
        }
    }
    ////////////////////////////////
    // TOGGLES ////////// USE MAP //
    //[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[

    public void updateKeys(){
        if(gamepad2.y && cdCheck(useMap2.y, 500)){ 
            toggleMap2.y = toggle(toggleMap2.y);
            useMap2.y = runtime.milliseconds();
        }
		if(gamepad2.left_bumper && cdCheck(useMap2.left_bumper, 200)){ 
            toggleMap2.left_bumper = toggle(toggleMap2.left_bumper);
            useMap2.left_bumper = runtime.milliseconds();
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


