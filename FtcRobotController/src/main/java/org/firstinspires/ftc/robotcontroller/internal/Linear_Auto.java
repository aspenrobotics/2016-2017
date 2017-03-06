package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cCompassSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@Autonomous(name = "Blue Linear", group = "Auto")
public class Linear_Auto extends LinearOpMode{

    private DcMotor leftMotor, rightMotor, armMotor, elevatorMotor, rollerMotor;
    private ColorSensor topSensor;
    private OpticalDistanceSensor opticalSensor;
    private boolean readyToFire;
    private Servo beaconHit;

    private enum RobotState{
        Init, FirstTwoShots, TurnToWall, DriveToWall, ParallelToWall
    }
    private RobotState rState;


    @Override
    public void runOpMode(){
        variableSettingsAndInitialization();
        //Above this is the init
        //Below this is the start
        waitForStart();
        ElapsedTime time = new ElapsedTime();
        time.reset();
        moveForwardTwoInches();
        time.reset();
        firstTwoShots(time);
        time.reset();
        turnToWall();
        time.reset();
        driveToWall();
        time.reset();
        parallel(time);
        time.reset();
        backIntoWall();
        time.reset();
        //driveToFirstWhiteLine();
        time.reset();
        hitABeacon(time);
        time.reset();
        backIntoWall();
        time.reset();
        hitABeacon(time);
        idle();








    }
    //Code for Hitting the Beacon
    private void hitABeacon(ElapsedTime time){
        leftMotor.setPower(.1);
        rightMotor.setPower(.1);
        leftMotor.setTargetPosition(-3000);
        rightMotor.setTargetPosition(-3000);
        while (topSensor.blue() < 4){}
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        resetEncoders(leftMotor);
        resetEncoders(rightMotor);
        leftMotor.setPower(.1);
        rightMotor.setPower(.1);
        leftMotor.setTargetPosition(-500);
        rightMotor.setTargetPosition(-500);
        while (leftMotor.isBusy() || rightMotor.isBusy()){}
        time.reset();
        while(time.time() < .6) {
            beaconHit.setPosition(1);
        }
        time.reset();
        while(time.time() < .6) {
            beaconHit.setPosition(0);
        }
        time.reset();
        while(time.time() < .6) {
            beaconHit.setPosition(1);
        }
        beaconHit.setPosition(0);
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        resetEncoders(leftMotor);
        resetEncoders(rightMotor);


    }

    //Back Into Wall
    private void backIntoWall(){
        leftMotor.setPower(.3);
        rightMotor.setPower(.3);
        leftMotor.setTargetPosition(-1000);
        rightMotor.setTargetPosition(-1000);
        while (leftMotor.isBusy() || rightMotor.isBusy()){}
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        resetEncoders(leftMotor);
        resetEncoders(rightMotor);
    }
    //Parallel
    private void parallel(ElapsedTime time){
        leftMotor.setPower(.3);
        rightMotor.setPower(.3);
        leftMotor.setTargetPosition(1000);
        rightMotor.setTargetPosition(1000);
        while (!((leftMotor.isBusy() || rightMotor.isBusy()) && (time.time() > .9))){}
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        resetEncoders(leftMotor);
        resetEncoders(rightMotor);

    }
    //Go to wall
    private void driveToWall(){
        leftMotor.setPower(1);
        rightMotor.setPower(1);
        leftMotor.setTargetPosition(6700);
        rightMotor.setTargetPosition(6700);
        while (leftMotor.isBusy() || rightMotor.isBusy()){}
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        resetEncoders(leftMotor);
        resetEncoders(rightMotor);



    }
    //Turns to wall
    private void turnToWall(){
        leftMotor.setPower(1);
        rightMotor.setPower(1);
        leftMotor.setTargetPosition(900);
        rightMotor.setTargetPosition(0);
        while (leftMotor.isBusy() || rightMotor.isBusy()){}
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        resetEncoders(leftMotor);
        resetEncoders(rightMotor);

    }
    //Move forward a bit
    private void moveForwardTwoInches(){
        leftMotor.setPower(1);
        rightMotor.setPower(1);
        leftMotor.setTargetPosition(425);
        rightMotor.setTargetPosition(425);
        while(leftMotor.isBusy() || rightMotor.isBusy()){}
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        resetEncoders(leftMotor);
        resetEncoders(rightMotor);
        rState = RobotState.FirstTwoShots;

    }
    //Fires The First Two Shots
    private void firstTwoShots(ElapsedTime time){
        time.reset();
        catapultArmFullPowerNoE();
        time.reset();
        elevatorMotor.setPower(1);
        while(time.time() < 2){}
        elevatorMotor.setPower(0);
        time.reset();
        armMotor.setMaxSpeed(300);
        armMotor.setPower(-1);
        while(time.time() < .5){}
        armMotor.setPower(0);
        armMotor.setMaxSpeed(900);
        time.reset();
        while (time.time() < .3){}
        readyToFire = true;
        catapultArmFullPowerNoE();
        rState = RobotState.TurnToWall;



    }
    //Full Power for 550 milliseconds
    private void catapultArmFullPowerNoE(){
        try{
            catapultArmFullPower();
        }
        catch (Exception e){
            e.printStackTrace();
        }
    }
    //Full Power Catapult With Exception
    private void catapultArmFullPower() throws Exception{
        if(readyToFire) {
            armMotor.setPower(1);
            Thread.sleep(550);
            readyToFire = false;
            armMotor.setPower(0);
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
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevatorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        resetEncoders(rightMotor);
        resetEncoders(leftMotor);
        rState = RobotState.Init;
        beaconHit = hardwareMap.servo.get("main_servo");
        armMotor.setMaxSpeed(900);
    }
    private boolean encoderReset(DcMotor motor){
        return (motor.getCurrentPosition() == 0);
    }
    private void resetEncodersWait(DcMotor motor) throws Exception{
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Thread.sleep(30);
        if(!encoderReset(motor)){
            Thread.sleep(30);
        }
    }
    private void resetEncoders(DcMotor motor){
        try {
            resetEncodersWait(motor);
        }
        catch(Exception e){
            e.printStackTrace();
        }
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
