package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cCompassSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.CompassSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;


@Autonomous(name = "Auto_Encoder", group = "Auto")
@Disabled
public class Auto_Encoder extends OpMode {

    private DcMotor leftMotor, rightMotor, armMotor, elevatorMotor, rollerMotor;
    private ColorSensor topSensor;
    private OpticalDistanceSensor opticalSensor;
    private boolean readyToFire = true;
    private ModernRoboticsI2cCompassSensor compassSensor;
    private double initialHeading, angle;

    private enum RobotState{
        Initialization ,FirstTwoShots, ContinueForward, Turn45, RollUpRamp
    }
    private RobotState robotState;


    public void init(){
        //declaring motor vars
        robotState = RobotState.Initialization;
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
        compassSensor = hardwareMap.get(ModernRoboticsI2cCompassSensor.class, "compass_sensor");
        compassSensor.setMode(CompassSensor.CompassMode.CALIBRATION_MODE);
        //setting mode
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rollerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevatorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void start(){
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        compassSensor.setMode(CompassSensor.CompassMode.MEASUREMENT_MODE);
        angle = 45;
        resetEncoders(rightMotor);
        resetEncoders(leftMotor);
    }
    public void stop(){

        leftMotor.setPower(0);
        rightMotor.setPower(0);
        resetEncoders(rightMotor);
        resetEncoders(leftMotor);
    }
    public void loop(){
        //Double shot
        startShot(this.time);
        //MoveForward
        continueForward(this.time);
        //Turn45
        /*turn45(this.time);
        //Backup
        RollUpRamp(this.time);
        */
        

        //The code above works for now
        //Moving forward
        telemetry.addData("This is the time value:  " + this.time, this.time);
        telemetry.addData("The motor Position: " + leftMotor.getCurrentPosition() + " " + rightMotor.getCurrentPosition(), leftMotor);
        /*telemetry.addData("This is the Heading: " + compassSensor.getDirection(), compassSensor.getDirection());
        telemetry.addData("This is the Initial Heading value: " + initialHeading, compassSensor);
        */

    }
    private void RollUpRamp(double time){
        if(time > 13 && time < 18 && robotState == RobotState.RollUpRamp){
            rightMotor.setPower(-1);
            leftMotor.setPower(-1);
        }
        if(time > 18 && robotState == RobotState.RollUpRamp){
            leftMotor.setPower(0);
            rightMotor.setPower(0);
        }
    }
    private void continueForward(double time){
        if(time > 7 && (robotState == RobotState.ContinueForward && time < 8.8)){
            rightMotor.setTargetPosition(6000);
            leftMotor.setTargetPosition(6000);
            rightMotor.setPower(1);
            leftMotor.setPower(1);
        }
        if(time > 8.8 && robotState == RobotState.ContinueForward){
            robotState = RobotState.Turn45;
        }
    }
    private void turn45(double time){
        if(time > 8.8 && time < 10.2 && robotState == RobotState.Turn45){
            rightMotor.setPower(-1);
        }
        if(time > 10.2 && robotState == RobotState.Turn45){
            rightMotor.setPower(0);
            robotState = RobotState.RollUpRamp;
        }
    }
    private void catapultArmFullPower() throws Exception{
        if(readyToFire) {
            armMotor.setPower(1);
            Thread.sleep(550);
            readyToFire = false;
        }
    }
    //Move to white line
    //Important for encoder stuff
    private boolean encoderReset(DcMotor motor){
        return (motor.getCurrentPosition() == 0 && motor.getCurrentPosition() ==0);
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
    //The Start of the Program Shoots Twice
    private void startShot(double time){
        if(robotState == RobotState.Initialization){
            leftMotor.setTargetPosition(200);
            rightMotor.setTargetPosition(200);
            leftMotor.setPower(.5);
            rightMotor.setPower(.5);
            robotState = RobotState.FirstTwoShots;
        }
        if(time < 1.4 && time > .7){
            armMotor.setPower(-.3);
        }

        if(time > 1.4 && time < 2.1){
            armMotor.setPower(0);

        }
        if(time > 2.2 && readyToFire){
            try {
                catapultArmFullPower();
            } catch (Exception e) {
                e.printStackTrace();
            }
            armMotor.setPower(0);
        }
        //second shot
        if(time > 3.5 && time < 5.5){
            elevatorMotor.setPower(1);
        }
        if(time < 5.9 && time > 5.5){
            elevatorMotor.setPower(0);
            armMotor.setPower(-.3);
        }
        if((time > 6.1 && time < 6.2)){
            readyToFire = true;
            resetEncoders(leftMotor);
            resetEncoders(rightMotor);
            robotState = RobotState.ContinueForward;
        }
    }

}
