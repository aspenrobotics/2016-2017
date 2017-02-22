package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Tests The Color Sensor Values
 */

@TeleOp(name = "Read Sensor Values", group = "Auto")
public class Color_Sensor_Tester extends OpMode{
    private DcMotor leftMotor, rightMotor, armMotor, elevatorMotor, rollerMotor;
    private ColorSensor topSensor;
    private OpticalDistanceSensor opticalSensor;
    private boolean readyToFire;
    private Servo beaconHit;

    public void init(){
        variableSettingsAndInitialization();
    }
    public void start(){

    }
    public void stop(){

    }
    public void loop(){
        double r = topSensor.red();
        double b = topSensor.blue();
        double g = topSensor.green();
        double a = topSensor.alpha();
        telemetry.addData("Red reads: " + r, topSensor);
        telemetry.addData("Blue reads: " + b, topSensor);
        telemetry.addData("Green reads: " + g, topSensor);
        telemetry.addData("Alpha reads: " + a, topSensor);
        telemetry.addData("Servo Position: " + beaconHit.getPosition(), beaconHit);
        if(gamepad1.x){
            topSensor.enableLed(true);
        }
        if(gamepad1.a){
            topSensor.enableLed(false);
        }

    }
    private void variableSettingsAndInitialization(){
        //////////////////////////////////////////////////////////////////////
        //Variable Settings and Modes;
        //////////////////////////////////////////////////////////////////////
        leftMotor = hardwareMap.dcMotor.get("right_drive");
        rightMotor = hardwareMap.dcMotor.get("left_drive");
        armMotor = hardwareMap.dcMotor.get("arm_motor");
        rollerMotor = hardwareMap.dcMotor.get("roller_motor");
        elevatorMotor = hardwareMap.dcMotor.get("elevator_motor");
        topSensor = hardwareMap.colorSensor.get("top_sensor");
        opticalSensor = hardwareMap.opticalDistanceSensor.get("optical_sensor");
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        armMotor.setDirection(DcMotor.Direction.REVERSE);
        topSensor.enableLed(false);
        readyToFire = true;
        //setting mode
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rollerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevatorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        beaconHit = hardwareMap.servo.get("main_servo");
    }


}
