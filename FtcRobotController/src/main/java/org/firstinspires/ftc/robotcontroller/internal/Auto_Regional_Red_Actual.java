package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@Autonomous(name = "Straight Forward", group = "Auto")
public class Auto_Regional_Red_Actual extends LinearOpMode{

    private DcMotor leftMotor, rightMotor, armMotor, elevatorMotor, rollerMotor;
    private ColorSensor topSensor;
    //private OpticalDistanceSensor opticalSensor;
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
        //time.reset();
        moveRobot(1, 1, 425, 425, false);//moveForwardTwoInches();
        firstTwoShots();
        time.reset();
        //turnToWall();
        //moveRobot(-1,1,500,500);
        time.reset();
        //driveToWall();
        time.reset();
        //smallTurn();
        time.reset();
        //parallel(time);
        time.reset();
        //hitABeacon(time);
        time.reset();
        //backIntoWall();
        time.reset();
        //hitABeacon(time);

        idle();








    }
    //Code for Hitting the Beacon
    private void hitABeacon(ElapsedTime time){
        moveRobot(.1,.1,5000,5000,true);
        leftMotor.setPower(.1);
        rightMotor.setPower(.1);
        leftMotor.setTargetPosition(5000);
        rightMotor.setTargetPosition(5000);
        while (topSensor.red() < 2){} // Empty Code Block?
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        resetEncoders(leftMotor);
        resetEncoders(rightMotor);
        leftMotor.setPower(.1);
        rightMotor.setPower(.1);
        leftMotor.setTargetPosition(-500);
        rightMotor.setTargetPosition(-500);
        while (leftMotor.isBusy() || rightMotor.isBusy()){} //Empty Code Block?
        time.reset();
        while(time.time() < 1.8) {
            if(time.time() >= 0.6 && time.time() < 1.2)
                beaconHit.setPosition(0);
            else
                beaconHit.setPosition(1);
        }
        beaconHit.setPosition(0);
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        resetEncoders(leftMotor);
        resetEncoders(rightMotor);


    }

    private void moveRobot(double leftPower, double rightPower, int leftCount, int rightCount, boolean color){
        resetEncoders(leftMotor);
        resetEncoders(rightMotor);
        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);
        leftMotor.setTargetPosition(leftCount);
        rightMotor.setTargetPosition(rightCount);
        if(color)
            while(topSensor.red() < 2){}
        else
            while (leftMotor.isBusy() || rightMotor.isBusy()){}
        leftMotor.setPower(0);
        rightMotor.setPower(0);

    }

    /*
    //Back Into Wall
    private void backIntoWall(){
        leftMotor.setPower(.6);
        rightMotor.setPower(.6);
        leftMotor.setTargetPosition(2200);
        rightMotor.setTargetPosition(2200);
        while (leftMotor.isBusy() || rightMotor.isBusy()){}
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        resetEncoders(leftMotor);
        resetEncoders(rightMotor);
    }
    //Parallel
    private void parallel(ElapsedTime time){
        leftMotor.setPower(.6);
        rightMotor.setPower(.6);
        leftMotor.setTargetPosition(-5500);
        rightMotor.setTargetPosition(-5500);
        while (!((leftMotor.isBusy() || rightMotor.isBusy()) && (time.time() > 4.5))){}
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        resetEncoders(leftMotor);
        resetEncoders(rightMotor);

    }
    //Small Turn
    private void smallTurn(){
        leftMotor.setPower(1);
        rightMotor.setPower(0);
        leftMotor.setTargetPosition(700);
        rightMotor.setTargetPosition(0);
        while (leftMotor.isBusy() || rightMotor.isBusy()){}
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        resetEncoders(leftMotor);
        resetEncoders(rightMotor);

    }
    //Go to wall
    private void driveToWall(){
        leftMotor.setPower(1);
        rightMotor.setPower(1);
        leftMotor.setTargetPosition(-5000);
        rightMotor.setTargetPosition(-5000);
        while (leftMotor.isBusy() || rightMotor.isBusy()){}
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        resetEncoders(leftMotor);
        resetEncoders(rightMotor);
    }
    //Turns to wall
    private void turnToWall(){
        leftMotor.setPower(1);
        rightMotor.setPower(0);
        leftMotor.setTargetPosition(3600);
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

    */
    //Fires The First Two Shots
    private void firstTwoShots(){
       // time.reset();
        catapultArmFullPowerNoE();
        ElapsedTime time = new ElapsedTime();
        elevatorMotor.setPower(1);
        while(time.time() < 2){} //Wait two seconds
        elevatorMotor.setPower(0);
        time.reset();
        armMotor.setMaxSpeed(350);
        armMotor.setPower(-1);
        while(time.time() < .7){}
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
            Thread.sleep(700);
            armMotor.setPower(0);
            readyToFire = false;
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
        //opticalSensor = hardwareMap.opticalDistanceSensor.get("optical_sensor");
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
