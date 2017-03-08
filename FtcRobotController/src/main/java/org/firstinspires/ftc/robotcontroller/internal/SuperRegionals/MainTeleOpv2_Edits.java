package org.firstinspires.ftc.robotcontroller.internal.SuperRegionals;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;


///////////////////////////////////////////////////////////
//               Main TeleOp Program                     //
///////////////////////////////////////////////////////////
@TeleOp(name = "Concept: MainTeleOpv2", group = "Tester")
//@Disabled
public class MainTeleOpv2_Edits extends OpMode implements Runnable{
    private DcMotor leftMotor, rightMotor, armMotor, elevatorMotor, rollerMotor, liftMotor, liftMotorTwo;
    private ColorSensor topSensor;
    private Servo beaconHit, servoBallControl;
    private enum CState{
        Normal, SlowSpeed
    }
    private enum ServoState{
        Inside, Outside
    }

    private CState cState;
    private ServoState servoState, ballServoState;

    @Override
    public void run(){

    }
    @Override
    public void init() {
        // Save reference to Hardware map
        beaconHit = hardwareMap.servo.get("main_servo");
        cState = CState.Normal;
        servoState = ServoState.Inside;
        ballServoState = ServoState.Inside;
        // Define and Initialize Motors
        leftMotor = hardwareMap.dcMotor.get("left_drive");
        rightMotor = hardwareMap.dcMotor.get("right_drive");
        armMotor = hardwareMap.dcMotor.get("arm_motor");
        rollerMotor = hardwareMap.dcMotor.get("roller_motor");
        elevatorMotor = hardwareMap.dcMotor.get("elevator_motor");
        topSensor = hardwareMap.colorSensor.get("top_sensor");
        liftMotor = hardwareMap.dcMotor.get("lift_motor");
        liftMotorTwo = hardwareMap.dcMotor.get("lift_motor_2");
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);
        armMotor.setDirection(DcMotor.Direction.REVERSE);
        liftMotor.setDirection(DcMotor.Direction.FORWARD);
        liftMotorTwo.setDirection(DcMotor.Direction.REVERSE);
        servoBallControl = hardwareMap.servo.get("ball_servo");
        topSensor.enableLed(false);
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
        liftMotorTwo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Change this for max speed
        armMotor.setMaxSpeed(1800);



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
        telemetry.addData("Encoder Position at 600ms: " + armMotor.getCurrentPosition(), armMotor);
    }
    //Main Catapult Arm Power
    private void switchOpMode() throws Exception{
        if(gamepad1.x || gamepad2.x){
            if(servoState == ServoState.Inside){
                servoState = ServoState.Outside;
            }
            else if(servoState == ServoState.Outside){
                servoState = ServoState.Inside;
            }
            Thread.sleep(200);
        }
        if(gamepad1.start){
            if(cState == CState.Normal){
                cState = CState.SlowSpeed;
            }
            else if(cState == CState.SlowSpeed){
                cState = CState.Normal;
            }
            Thread.sleep(200);
        }
        if(gamepad1.y || gamepad2.y){
            if(ballServoState == ServoState.Inside){
                ballServoState = ServoState.Outside;
            }
            else if(ballServoState == ServoState.Outside){
                ballServoState = ServoState.Inside;
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
        if(ballServoState == ServoState.Inside){
            servoBallControl.setPosition(70);
        }
        if(ballServoState == ServoState.Outside){
            servoBallControl.setPosition(0);
        }
    }
    private void elevatorPower(){
        if(gamepad1.a || gamepad2.a){
            elevatorMotor.setPower(1);
            rollerMotor.setPower(1);
        }
        else if(gamepad1.b || gamepad2.b) {
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
            armMotor.setMaxSpeed(1600);
            armMotor.setPower(1);
            Thread.sleep(600);
        }
    }
    private void catapultReturn(){
        if(gamepad1.left_bumper) {
            armMotor.setMaxSpeed(900);
            armMotor.setPower(-1);
        } else{
            armMotor.setPower(0);
        }
    }
    private void liftMotorControl(){
        if(gamepad1.right_trigger > .2 || gamepad2.right_trigger >.2){
            liftMotor.setPower(1);
            liftMotorTwo.setPower(1);
        }
        if(gamepad1.left_trigger > .2 || gamepad2.left_trigger >.2){
            liftMotor.setPower(-1);
            liftMotorTwo.setPower(-1);
        }
        if(gamepad1.left_trigger < .2 && gamepad1.right_trigger < .2){
            liftMotor.setPower(0);
            liftMotorTwo.setPower(0);
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


