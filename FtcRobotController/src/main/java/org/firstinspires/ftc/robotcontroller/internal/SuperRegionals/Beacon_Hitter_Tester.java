package org.firstinspires.ftc.robotcontroller.internal.SuperRegionals;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
//This Autonomous is purely to test the beacons once the robot is lined up have lined up.
@Disabled
@Autonomous(name = "Beacon Test", group = "Auto")
public class Beacon_Hitter_Tester extends LinearOpMode{

    DcMotor leftMotor, rightMotor, armMotor, elevatorMotor;
    ColorSensor topSensor;
    boolean readyToFire;
    Servo beaconHit;


    @Override
    public void runOpMode(){

        variableSettingsAndInitialization(); //Variable Initialization

        waitForStart();
        moveRobot(.1, .1, -3000, -3000, true, 7); //Test for Beacons
        multipleBeaconHit();
        moveRobot(.1, .1, -1000, -1000, false, 5);
        moveRobot(.1, .1, -3000, -3000, true, 7); //Test for Beacons
        multipleBeaconHit();



    }
    private void moveRobot(double leftPower, double rightPower, int leftCount, int rightCount, boolean color, double maxTime){
        resetEncoders(leftMotor);
        resetEncoders(rightMotor);
        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);
        leftMotor.setTargetPosition(leftCount);
        rightMotor.setTargetPosition(rightCount);
        ElapsedTime time = new ElapsedTime();
        time.reset();
        if(color)
            while(topSensor.blue() < 5){
                if(maxTime < time.time()){break;}
            }
        else
            while (leftMotor.isBusy() || rightMotor.isBusy()){
                if(maxTime < time.time()){break;}
            }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }
    private void multipleBeaconHit(){
        hitBeacon(1, 1000); //Hits Beacons
        hitBeacon(.5, 500);
        hitBeacon(1, 500);
        hitBeacon(0, 1000);
    }
    //This needs an int from 0 -1
    private void hitBeacon(double position, int sleepTime) {
            beaconHit.setPosition(position);
            try {
                Thread.sleep(sleepTime);
            } catch (Exception e) {
                e.printStackTrace();
            }
    }
    private void firstTwoShots(){
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
        elevatorMotor = hardwareMap.dcMotor.get("elevator_motor");
        topSensor = hardwareMap.colorSensor.get("top_sensor");
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        armMotor.setDirection(DcMotor.Direction.REVERSE);
        topSensor.enableLed(false);
        readyToFire = true;
        //setting mode
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevatorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        resetEncoders(rightMotor);
        resetEncoders(leftMotor);
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
