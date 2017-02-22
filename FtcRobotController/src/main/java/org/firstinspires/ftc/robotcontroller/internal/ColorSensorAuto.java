package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;


///////////////////////////////////////////////////////////
//                 Test Color Sensor Program             //
///////////////////////////////////////////////////////////
@TeleOp(name = "ColorSensorAuto", group = "Autonomous")
@Disabled
public class ColorSensorAuto extends OpMode implements Runnable{
    //private DcMotor leftMotor, rightMotor;
    private ColorSensor topSensor, bottomSensor;
    private Servo mainServo;
    private double a, r,g, b;

    //The run method currently has no use
    @Override
    public void run(){}



    @Override
    public void init(){
        //Variable Declaration
        //leftMotor = hardwareMap.dcMotor.get("left_drive");
        //rightMotor = hardwareMap.dcMotor.get("right_drive");

        /*!!!!!!!!!!!!!!!!Make Sure To Use This When Setting Up Any Color Sensors!!!!!!!!!!!!*/
        topSensor = hardwareMap.colorSensor.get("top_sensor");
        bottomSensor = hardwareMap.colorSensor.get("bottom_sensor");
        topSensor.setI2cAddress(I2cAddr.create7bit(0x26));
        bottomSensor.setI2cAddress(I2cAddr.create7bit(0x2e));
        mainServo = hardwareMap.servo.get("push_servo");

        ///////////////////////////////////////////////////////////////////////////////////////

        //Setting Settings
        /*
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        */




    }
    @Override
    public void start(){
        /*
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        */
    }
    @Override
    public void stop() {
        topSensor.enableLed(false);
        bottomSensor.enableLed(false);
        /*
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        */

    }
    @Override
    public void loop(){


        telemetry.addData("The Top Alpha Reads: ", topSensor.alpha());
        telemetry.addData("The Bottom Sensor Reads: ", bottomSensor.alpha());
        lightControl();
    }
    private void lightControl(){
        if(gamepad1.a){
            topSensor.enableLed(false);
        }
        if(gamepad1.b){
            topSensor.enableLed(true);
        }
        if(gamepad1.y){
            mainServo.setPosition(0);
        }
        if(gamepad1.x){
            mainServo.setPosition(1);
        }
    }


}
