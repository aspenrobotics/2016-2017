package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Enumeration;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;


///////////////////////////////////////////////////////////
//               Main TeleOp Program                     //
///////////////////////////////////////////////////////////
@TeleOp(name = "Concept: MainTeleOpv2_Actual", group = "Tester")
@Disabled
public class MainTeleOpv2 extends OpMode implements Runnable{
    private DcMotor leftMotor, rightMotor, armMotor, elevatorMotor, rollerMotor, liftMotor;
    private ColorSensor topSensor;
    private Servo beaconHit;
    private enum CState{
        Normal, SlowSpeed
    }
    private enum ServoState{
        Inside, Outside
    }
    private CState cState;
    private ServoState servoState;

    @Override
    public void run(){

    }
    @Override
    public void init() {
        // Save reference to Hardware map
        beaconHit = hardwareMap.servo.get("main_servo");
        cState = CState.Normal;
        servoState = ServoState.Inside;
        // Define and Initialize Motors
        leftMotor = hardwareMap.dcMotor.get("left_drive");
        rightMotor = hardwareMap.dcMotor.get("right_drive");
        armMotor = hardwareMap.dcMotor.get("arm_motor");
        rollerMotor = hardwareMap.dcMotor.get("roller_motor");
        elevatorMotor = hardwareMap.dcMotor.get("elevator_motor");
        topSensor = hardwareMap.colorSensor.get("top_sensor");
        liftMotor = hardwareMap.dcMotor.get("lift_motor");
        //bottomSensor = hardwareMap.colorSensor.get("bottom_sensor");
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);
        armMotor.setDirection(DcMotor.Direction.REVERSE);
        liftMotor.setDirection(DcMotor.Direction.FORWARD);
        topSensor.enableLed(false);
        //bottomSensor.enableLed(false);
        //Declares the thread for arm control


        // Set all motors to zero power
        leftMotor.setPower(0);
        rightMotor.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rollerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevatorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ///////////////
        ///////////////
        ////////////
        ////////////
        /////////////
        ///////////////
        ////////////
        //Change this for max speed
        armMotor.setMaxSpeed(4000);



        // Define and initialize ALL installed servos.
    }
    @Override
    public void start(){
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        armMotor.setPower(0);
        topSensor.enableLed(false);
        //bottomSensor.enableLed(false);

    }
    @Override
    public void stop(){
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        armMotor.setPower(0);
        topSensor.enableLed(false);
        liftMotor.setPower(0);
        //bottomSensor.enableLed(false);
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
        catapultUnlock();
        elevatorPower();
        liftMotorControl();
        servoControl();
        try {
            switchOpMode();
        }catch (Exception e){
            e.printStackTrace();
        }

        try {
            catapultArmFullPower();
        } catch (Exception e) {
            e.printStackTrace();
        }
        telemetry.addData("Servo Position Is: " + beaconHit.getPosition(), beaconHit);
        telemetry.addData("Servo Direction Is: " + beaconHit.getDirection(), beaconHit);
    }
    //Main Catapult Arm Power
    private void switchOpMode() throws Exception{
        if(gamepad1.x){ //Is the purpose here to switch from ServoState.Inside to ServoState.Outside and vice-versa?
            if(servoState == ServoState.Inside){
                servoState = ServoState.Outside;
            }
            else if(servoState == ServoState.Outside){
                servoState = ServoState.Inside;
            }
            Thread.sleep(200);
        }
        if(gamepad1.start){ //See question Above
            if(cState == CState.Normal){
                cState = CState.SlowSpeed;
            }
            else if(cState == CState.SlowSpeed){
                cState = CState.Normal;
            }
            Thread.sleep(200);
        }


    }
    private void servoControl(){
        if(servoState == ServoState.Inside){
            beaconHit.setPosition(0);
        }
        if(servoState == ServoState.Outside){
            beaconHit.setPosition(180);
        }
    }
    private void elevatorPower(){
        if(gamepad1.a){
            elevatorMotor.setPower(1);
            rollerMotor.setPower(1);
        }
        else if(gamepad1.b) {
            elevatorMotor.setPower(-1);
            rollerMotor.setPower(-1);
        }
        else{
            elevatorMotor.setPower(0);
            rollerMotor.setPower(0);
        }
    }
    private void catapultArmFullPower() throws Exception {
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
    private void liftMotorControl(){
        if(gamepad1.right_trigger > .2){
            liftMotor.setPower(1);
        }
        if(gamepad1.left_trigger > .2){
            liftMotor.setPower(-1);
        }
        if(gamepad1.left_trigger < .2 && gamepad1.right_trigger < .2){
            liftMotor.setPower(0);
        }
    }
    //unlocks catapult
    private void catapultUnlock(){
        if(gamepad1.y){
            armMotor.setPower(.3);
        }
        else{
            armMotor.setPower(0);
        }
    }

    //Controls Motor Power
    private void motorPower(float left, float right){
        if(cState == CState.Normal) {
            leftMotor.setPower(left);
            rightMotor.setPower(right);
        }
        if(cState == CState.SlowSpeed){
            leftMotor.setPower(left*.5);
            rightMotor.setPower(right*.5);
        }
    }

}

