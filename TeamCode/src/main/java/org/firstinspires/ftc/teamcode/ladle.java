package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

//Numeros uno and dos
public class ladle{
    //////////////////
    /* DECLARATIONS */
    //////////////////

    //DRIVE//
    public DcMotor frontLeft   = null;
    public DcMotor backLeft    = null;
    public DcMotor backRight   = null;
	public DcMotor frontRight  = null;


    //Big Boi Slides
    public DcMotor shoop = null; //Side verticaal slides
    public DcMotor zhoop = null;
    public DcMotor uwu = null; //Shooty outty slide out

	public int shoopMax = 4315;
	public int zhoopMax = 4133;
	public int uwuMax = 0;

	public int uwuDeposit = 0;
	public int uwuContained = 0; //Horizontal distance the slide can extend inside the chassis without smashing shit

	public int shoopLoad = 0; //Position for loading a block
	public int zhoopLoad = 0;

	public int shoopMinClearance = 2000;
	public int zhoopMinClearance = 2000;

	public int shoopClearance = shoopMinClearance+(0.1*shoopMax); //Height for clearing physical obstacles. Need to go to this height to extend
	public int zhoopClearance = zhoopMinClearance+(0.1*zhoopMax);
    //Intake wheels
    public DcMotor succ = null;

    //GRaap
	public CRServo crGrap = null;


    //IMU//
    BNO055IMU imu;

    HardwareMap hwMap           =  null;

    /* Constructor */
    public ladle(){
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap hwMap) {
        //////////////////////////////////
        /* RETRIEVING STUFF FROM PHONES */
        //////////////////////////////////

        //DRIVE//
        frontLeft   = hwMap.dcMotor.get("fl");
        backLeft    = hwMap.dcMotor.get("bl");
        backRight   = hwMap.dcMotor.get("br");
        frontRight  = hwMap.dcMotor.get("fr");
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //SLIIIIIIDE
        shoop = hwMap.dcMotor.get("shoop");
        zhoop = hwMap.dcMotor.get("zhoop");
        uwu = hwMap.dcMotor.get("uwu");
        shoop.setDirection(DcMotor.Direction.REVERSE);
        shoop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        zhoop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		uwu.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shoop.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        zhoop.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		uwu.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		shoop.setTargetPosition(0);
		zhoop.setTargetPosition(0);
		uwu.setTargetPosition(0);
        shoop.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        zhoop.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		uwu.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        //SUUCCCCCC
        succ = hwMap.dcMotor.get("succ");
        succ.setDirection(DcMotor.Direction.FORWARD);
        succ.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

		//Grap
		crGrap = hwMap.crservo.get("crGrap");

        //IMU//
        imu = hwMap.get(BNO055IMU.class, "imu");
        //////////////////
        /* STUFFY STUFF */
        //////////////////

    }
    public void mecanumDrive(double pX, double pY, double pRot){
        frontLeft.setPower(pY + pRot);
        backLeft.setPower(pX - pRot);
        backRight.setPower(pY - pRot);
        frontRight.setPower(pX + pRot);
    }
    public void powerDrive(double power){
        frontLeft.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
        frontRight.setPower(power);
    }
    public void resetDrive(){
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void runToPosDrive(){
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void runWithoutEncoderDrive(){
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void imu(){
        /* IMU STUFF */
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        imu.initialize(parameters);
    }
}