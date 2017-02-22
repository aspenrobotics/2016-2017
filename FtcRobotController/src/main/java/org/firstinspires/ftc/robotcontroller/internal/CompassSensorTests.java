package org.firstinspires.ftc.robotcontroller.internal;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cCompassSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CompassSensor;



///////////////////////////////////////////////////////////
//                 Test Color Sensor Program             //
///////////////////////////////////////////////////////////

@TeleOp(name = "Compass_Sensor Reader", group = "Autonomous")
@Disabled
public class CompassSensorTests extends OpMode{
    //private DcMotor leftMotor, rightMotor;
    private ModernRoboticsI2cCompassSensor compassSensor;
    private boolean calibrationStatus;

    @Override
    public void init(){
        //Variable Declaration
        //leftMotor = hardwareMap.dcMotor.get("left_drive");
        //rightMotor = hardwareMap.dcMotor.get("right_drive");

        /*!!!!!!!!!!!!!!!!Make Sure To Use This When Setting Up Any Color Sensors!!!!!!!!!!!!*/
        compassSensor = hardwareMap.get(ModernRoboticsI2cCompassSensor.class, "compass_sensor");
        compassSensor.setMode(CompassSensor.CompassMode.CALIBRATION_MODE);




    }
    @Override
    public void start(){
        compassSensor.setMode(CompassSensor.CompassMode.MEASUREMENT_MODE);

    }
    @Override
    public void stop() {


    }
    @Override
    public void loop(){

        telemetry.addData("Compass Sensor Reads " + compassSensor.getDirection(), compassSensor);

    }



}
