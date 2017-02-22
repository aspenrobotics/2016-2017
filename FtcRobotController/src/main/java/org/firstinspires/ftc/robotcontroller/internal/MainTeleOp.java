package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


///////////////////////////////////////////////////////////
//               Main TeleOp Program                     //
///////////////////////////////////////////////////////////
@TeleOp(name = "Concept: MainTeleOp", group = "Tester")
@Disabled
public class MainTeleOp extends OpMode implements Runnable{
    private DcMotor leftMotor, rightMotor, armMotor, elevatorMotor, rollerMotor;
    private ColorSensor topSensor, bottomSensor;

    @Override
    public void run(){

    }
    @Override
    public void init() {
        // Save reference to Hardware map

        // Define and Initialize Motors
        leftMotor = hardwareMap.dcMotor.get("left_drive");
        rightMotor = hardwareMap.dcMotor.get("right_drive");
        armMotor = hardwareMap.dcMotor.get("arm_motor");
        rollerMotor = hardwareMap.dcMotor.get("roller_motor");
        elevatorMotor = hardwareMap.dcMotor.get("elevator_motor");
        topSensor = hardwareMap.colorSensor.get("top_sensor");
        bottomSensor = hardwareMap.colorSensor.get("bottom_sensor");
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);
        armMotor.setDirection(DcMotor.Direction.REVERSE);
        topSensor.enableLed(false);
        bottomSensor.enableLed(false);
        //Declares the thread for arm control


        // Set all motors to zero power
        leftMotor.setPower(0);
        rightMotor.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rollerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevatorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        // Define and initialize ALL installed servos.
    }
    @Override
    public void start(){
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        armMotor.setPower(0);
        topSensor.enableLed(false);
        bottomSensor.enableLed(false);

    }
    @Override
    public void stop(){
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        armMotor.setPower(0);
        topSensor.enableLed(false);
        bottomSensor.enableLed(false);
    }
    @Override
    public void loop(){
        float throttle = -gamepad1.left_stick_y;
        float direction = gamepad1.left_stick_x;
        float right = throttle + direction;
        float left = throttle - direction;
        //Sets all powers



        motorPower(left, right);
        catapultReturn();
        elevatorPower();
        try {
            catapultArmFullPower();
        } catch (Exception e) {
            e.printStackTrace();
        }

    }
    //Main Catapult Arm Power
    private void elevatorPower(){
        if(gamepad1.a){
            elevatorMotor.setPower(1);
            rollerMotor.setPower(1);
        }
        else{
            elevatorMotor.setPower(0);
            rollerMotor.setPower(0);
        }
    }
    private void catapultArmFullPower() throws Exception{
        if(gamepad1.right_bumper) {
            armMotor.setPower(1);
            Thread.sleep(600);
        }
    }
    private void catapultReturn(){
        if(gamepad1.left_bumper) {
            armMotor.setPower(-.3);
        }else{
            armMotor.setPower(0);
        }
    }
    //Controls Motor Power
    private void motorPower(float left, float right){
        leftMotor.setPower(left);
        rightMotor.setPower(right);
    }

}

