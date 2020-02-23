package RoboRaiders.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import RoboRaiders.AutonomousMethods.RRAutonomousMethods;
import RoboRaiders.AutonomousMethods.RoboRaidersPID;
import RoboRaiders.Robot.Robot;

@Autonomous(name= "PID Test Turn")
public class PIDTest extends RRAutonomousMethods {


    @Override
    public void runOpMode() {
        Robot robot = new Robot();
        RoboRaidersPID rrPID = new RoboRaidersPID(0.012,0,0.029);
        robot.initialize(hardwareMap);


        waitForStart();
        imuTurnPID(rrPID, robot, 90,  "right");
        robotSleep(1000);
        imuTurnPID(rrPID, robot, 90, "left");

    }
}


