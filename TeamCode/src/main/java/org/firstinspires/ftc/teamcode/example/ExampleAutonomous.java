package org.firstinspires.ftc.teamcode.example;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.hardware.ExampleDriveTrain;
import org.firstinspires.ftc.teamcode.hardware.ExampleServo;

@Autonomous(name = "ExampleAutonomous", group="Example")
public class ExampleAutonomous extends LinearOpMode {

    static final double CLAW_OPEN = 0.714;
    static final double CLAW_CLOSED = 0.970;

    public void runOpMode() {

        ExampleDriveTrain driveTrain = new ExampleDriveTrain( hardwareMap );
        ExampleServo claw = new ExampleServo( hardwareMap, "claw");
        ExampleServo arm = new ExampleServo( hardwareMap, "arm");

        waitForStart();

        while( opModeIsActive() ) {
        }
     }
}
