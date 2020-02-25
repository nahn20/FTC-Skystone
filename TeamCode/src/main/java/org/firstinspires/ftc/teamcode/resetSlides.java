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


@Autonomous(name="Reset Slides", group="main")
public class resetSlides extends LinearOpMode {
    ladle ldl = new ladle();


    @Override
    public void runOpMode() {
        ldl.init(hardwareMap);
        ldl.imu();
		ldl.resetSlides();
		while(opModeIsActive()){};
	}
}