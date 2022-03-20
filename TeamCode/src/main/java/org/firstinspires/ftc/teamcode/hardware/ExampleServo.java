package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ExampleServo {

    Servo servo;

    public ExampleServo(HardwareMap hardwareMap, String configName ) {

        try {
            servo = hardwareMap.servo.get( configName );

        } catch(Exception a){
            servo = null;
        }
    }

    public void setPosition( double position ) {

        servo.setPosition( position );
    }
}
