/*
 * University of California, Berkeley
 * Pioneers in Engineering, Robotics Organizer.
 * PiER Framework v0.30 - 02/24/12
 * 
*/

using System;
using System.Collections;
using PiEAPI;
using Microsoft.SPOT;

namespace StudentPiER
{
    public class StudentCode : RobotCode
    {
        // Variables
        private I2CWiiMote irsensor;
        /// <summary>
        /// This is your robot
        /// </summary>
        private Robot robot;

        private AnalogSonar sonar;

        private ArrayList dist = new ArrayList();
        private ArrayList right = new ArrayList();
        private ArrayList leftA = new ArrayList();
        /// <summary>
        /// These variables are your motor controllers
        /// </summary>
        private I2CMotorController i2cR;
        private I2CMotorController i2cL;
        private I2CMotorController firingR;
        private I2CMotorController firingL;
        private I2CMotorController conveyor;
        private MicroMaestro door;
        private int[] coords = new int[8];
        Boolean left = true;
        Boolean top = false;
        Boolean positioned = false;
        Boolean collection = false;
        double deviation = 0.02;
        int irCount = 0;
        int prevIrCount = 0;
        Boolean centered = false;
        int maxFiring = 100; //*
        int minFiring = 30; //*
        Boolean tankDrive = false;
        Boolean autonomous;
        Boolean reverseDrive = false;
        int count = 0;
        int firingTime = 30;
        int holdingTime = 30; // servo time
        int pressCountY = 0;
        int pressCountX = 0;
        int constIRcount = 0;
        Boolean on = true;
        /// <summary>
        /// Main method which initializes the robot, and starts
        /// it running.
        /// </summary>       
        public static void Main()
        {
            // Initialize robot
            Robot robot = new Robot("1", "COM4");
            robot.Auton(true);
            Debug.Print("Code loaded successfully!");
            Master master = new Master(new StudentCode(robot));
            master.RunCode();

        }

        // Constructor
        public StudentCode(Robot robot)
        {
            this.robot = robot;
            i2cR = new I2CMotorController(robot, 0x0B); //0x0A is the right motor on drivetrain by default
            i2cL = new I2CMotorController(robot, 0x0C); //0x0B is the left motor on drivetrain by default
            firingL = new I2CMotorController(robot, 0x0D);
            firingR = new I2CMotorController(robot, 0x0E);
            conveyor = new I2CMotorController(robot, 0x0F);
            irsensor = new I2CWiiMote(robot, 25, 0, 200);
            sonar = new AnalogSonar(0);
            door = new MicroMaestro(robot, 12);
            door.minRotation[0] = -90;
            door.maxRotation[0] = 220;
        }

        // Gets the robot associated with this StudentCode
        public Robot getRobot()
        {
            return this.robot;
        }

        /// <summary>
        /// The robot will call this method every time it needs to run the user-controlled student code
        /// The StudentCode should basically treat this as a chance to read all the new PiEMOS analog/digital values
        /// and then use them to update the actuator states
        /// </summary>
        public void UserControlledCode()
        {
            
            autonomous = false;
            for (int a = 0; a < coords.Length; a = a + 2)
            {
                Debug.Print("coords: (" + coords[a] + ", " + coords[a + 1] + ")");
            }
            Debug.Print("");
            double distance = sonar.GetDistance();
            Debug.Print("distance: " + distance);
            Debug.Print("average distance: " + getAverageDistance());
            // always first set brake to 0 to move the motors
            i2cL.motorBrake = 0;
            i2cR.motorBrake = 0;
          
            // Observe two values from the PiEMOS interface (like left and right joysticks) and map them directly to the speeds of the motors
            // PiEMOS interface values will be between 0 and 255, but I'm centering motor speed = 0 at 128 (halfway between 0 and 255), so that I
            // can get negative and positive speeds.
            // Because I2CMotorController's motorSpeed only accepts between -100 and 100, I have to map the values to that range.
            // Ex. PiEMOS Interface value of 255 --> (255 - 128) * 100 / 128 = 99.23 (basically 100, the highest forward motor speed)
            // EX. PiEMOS Interface value of 0 --> (0 - 128) * 100 / 128 = -100 (the highest backward motor speed)
            // The nice thing is that this will automatically change the motor speed to things like joystick values when the joysticks are moved
            if (reverseDrive) // collection drive click x
            {
                float x = ((float)-(robot.UIAnalogVals[2] - 128) * 100 / (float)128);
                float y = ((float)-(robot.UIAnalogVals[3] - 128) * 100 / (float)128);
                float x1 = ((float)-(robot.UIAnalogVals[0] - 128) * 100 / (float)128);
                float y1 = ((float)-(robot.UIAnalogVals[1] - 128) * 100 / (float)128);
                i2cL.motorSpeed = -getAverageLeft(y - x / 3 * 2 - x1 / 3 + y1 * 3 / 10); //my joys
                i2cR.motorSpeed = -getAverageRight(y + x / 3 * 2 + x1 / 3 + y1 * 3 / 10);
                robot.radio.telemetry.analog[0] = (byte)(i2cR.motorSpeed + 100);
                robot.radio.telemetry.analog[1] = (byte)(i2cL.motorSpeed + 100);
                determineMotors();
            }
            else if (tankDrive)//hold left bumper
            {
                i2cR.motorSpeed = ((float)-(robot.UIAnalogVals[1] - 128) * 100 / (float)128);
                robot.radio.telemetry.analog[0] = (byte)(i2cR.motorSpeed + 100);
                i2cL.motorSpeed = ((float)-(robot.UIAnalogVals[3] - 128) * 100 / (float)128);
                robot.radio.telemetry.analog[1] = (byte)(i2cL.motorSpeed + 100);
                determineMotors();
            }
            else
            {
                float x = ((float)-(robot.UIAnalogVals[2] - 128) * 100 / (float)128);
                float y = ((float)-(robot.UIAnalogVals[3] - 128) * 100 / (float)128);
                float x1 = ((float)-(robot.UIAnalogVals[0] - 128) * 100 / (float)128);
                float y1 = ((float)-(robot.UIAnalogVals[1] - 128) * 100 / (float)128);
                i2cL.motorSpeed = getAverageLeft(y + x / 3 * 2 + x1 / 3 + y1 * 3 / 10);// joy stick
                i2cR.motorSpeed = getAverageRight(y - x / 3 * 2 - x1 / 3 + y1 * 3 / 10);// js switch the signs before x
                robot.radio.telemetry.analog[0] = (byte)(i2cR.motorSpeed + 100);
                robot.radio.telemetry.analog[1] = (byte)(i2cL.motorSpeed + 100);
                determineMotors();
            }


            // Observe a certain button being pressed on PiEMOS interface, if true (meaning "if pressed"), then brake
            //a
            if (robot.UIDigitalVals[0])
            {
                if (!centered)
                {
                    centered = centerTurn();
                }
                else
                {
                    moveForwardTo(20);
                }
            }
            //l1
            if (robot.UIDigitalVals[4])
            {
                tankDrive = true;
            }
            else
            {
                tankDrive = false;
            }
            //r1
            if (robot.UIDigitalVals[5])
            {
                fire();
            }
            else
            {
                door.targets[0] = 0;
                door.speeds[0] = 50;
                collection = true;
            }
            //b
            if (robot.UIDigitalVals[1])
            {
                // treat this as a gradual brake. every time this method is called (very often!), it will slowly add more braking
                // until it reaches the max braking of 10, at which it remains at 10.
                if (i2cL.motorBrake < 10)
                {
                    i2cL.motorBrake = i2cL.motorBrake + (float).1; //let's represent fractions with floats. always cast decimals to float
                    i2cR.motorBrake = i2cR.motorBrake + (float).1;
                }
                else
                {
                    i2cL.motorBrake = 10;
                    i2cR.motorBrake = 10;
                }
            }
            // at any point, if the button is released, turn braking off
            else
            {
                i2cL.motorBrake = 0;
                i2cR.motorBrake = 0;
            }
            if (robot.UIDigitalVals[2])
            {
                if (pressCountX == 0)
                {
                    reverseDrive = !reverseDrive;
                }
                pressCountX++;
            }
            else
            {
                pressCountX = 0;
            }
            //y
            if (robot.UIDigitalVals[3])
            {
                if (pressCountY == 0)
                {
                    on = !on;
                }
                pressCountY++;
            }
            else
            {
                pressCountY = 0;
            }
            // This is useful for debugging:
            /*
            Debug.Print("UI: " + robot.UIAnalogVals[0] + " " + robot.UIAnalogVals[1] + " " + robot.UIAnalogVals[2] + " " + robot.UIAnalogVals[3]);
            Debug.Print("canMove: " + robot.canMove);
            Debug.Print("i2cL:" + i2cL.motorSpeed);
            Debug.Print("i2cR:" + i2cR.motorSpeed);
            */

        }

        /// <summary>
        /// The robot will call this method every time it needs to run the autonomous student code
        /// The StudentCode should basically treat this as a chance to change motors and servos based on
        /// non user-controlled input like sensors. But you don't need sensors, as this example demonstrates.
        /// </summary>
        public void AutonomousCode()
        {
            autonomous = true;
            Debug.Print("Auton");
            coords = irsensor.Read();
            prevIrCount = 8 - scan();
            sort();
                if (prevIrCount < 6 && !centered)
                {
                    constIRcount = 0;
                    //might see opposite goal
                    searchTurn(left);
                    irCount = 8 - scan();
                    if (irCount < prevIrCount)
                    {
                        left = false;
                    }
                }
                if (prevIrCount >= 6 && !centered)
                {
                    constIRcount++;
                    if (constIRcount >= 3)
                    {
                        top = coords[5] > coords[3] && coords[5] > coords[7];
                    }
                    centered = centerTurn();
                }
                    if (centered && !positioned)
                    {
                        moveForwardTo(20);
                    }
                    if (positioned)
                    {
                        count++;
                        if (count < firingTime) {
                            fire();
                        }
                        else if (count > firingTime + holdingTime)
                        {
                            count = 0;
                        }
                        else if (count >= firingTime)
                        {
                            door.targets[0] = 20;// autocode
                            door.speeds[0] = 50;
                        }
                    }
            determineMotors();
        }

        public void sort()
        {
            int[] temp = new int[8];
            for (int a = 0; a < coords.Length; a = a + 2)
            {
                int sIndex = a;
                for (int b = a; b < coords.Length; b = b + 2)
                {
                    if (coords[b] < coords[sIndex])
                    {
                        sIndex = b;
                    }
                }
                temp[a] = coords[sIndex];
                temp[a + 1] = coords[sIndex + 1];
                coords[sIndex] = coords[a];
                coords[sIndex + 1] = coords[a + 1];
                coords[a] = temp[a];
                coords[a + 1] = temp[a + 1];
            }
        }

        public void determineMotors()
        {
            if (collection)
            {
                conveyor.motorBrake = 0;
                conveyor.motorSpeed = -100;
            }
            else
            {
                conveyor.motorBrake = 10;
                conveyor.motorSpeed = 0;
                firingL.motorSpeed = 0;
                firingR.motorSpeed = 0;
                firingL.motorBrake = 10;
                firingR.motorBrake = 10;
            }
        }

        public int scan()
        {
            int zeroCount = 0;
            for (int a = 0; a < coords.Length; a++)
            {
                if (coords[a] == 0)
                {
                    zeroCount++;
                }
            }

            return zeroCount;
        }

        public void searchTurn(Boolean left)
        {
            if (left)
            {
                turnLeft(100);
            }
            else
            {
                turnRight(100);
            }
        }

        public Boolean centerTurn()
        {
            if (coords[2] < (512 - 512 * deviation))
            {
                turnLeft(100);
                return false;
            }
            else if (coords[2] > (512 + 512 * deviation))
            {
                turnRight(100);
                return false;
            }
            else
            {
                return true;
            }
        }

        public void turnLeft(int val)
        {
            i2cL.motorSpeed = -val;
            i2cR.motorSpeed = val;
        }

        public void turnRight(int val)
        {
            i2cL.motorSpeed = val;
            i2cR.motorSpeed = -val;
        }

        public void moveForwardTo(int dist)
        {
            if (getAverageDistance() > dist + 5)
            {
                moveForward(30);
                if (getAverageDistance() - dist + 5 < 5)
                {
                    moveForward((int)(getAverageDistance() - dist + 5) * 6);
                }
            }
            else
            {
                positioned = true;
                moveForward(0);
                i2cL.motorBrake = 10;
                i2cR.motorBrake = 10;
            }

        }

        public void moveForward(int val)
        {
            i2cL.motorBrake = 0;
            i2cR.motorBrake = 0;
            i2cL.motorSpeed = val;
            i2cR.motorSpeed = val;
        }

        public void aim()
        {
            //assuming on the slope..:
            //height = 46 - getAverageDistance()*sin(2)
            //length = getAverageDistance()*cos(2)
            //Vvertical = Vi*sin(angle)         Vi is the initial velocity of the ball
            //Vhorizontal = Vi*cos(angle)
            firingL.motorBrake = 0;
            firingR.motorBrake = 0;
            if (autonomous)
            {
                if (top)
                {
                    firingL.motorSpeed = maxFiring;
                    firingR.motorSpeed = maxFiring;
                }
                else
                {
                    firingL.motorSpeed = minFiring;
                    firingR.motorSpeed = minFiring;
                }
            }
            else
            {
                if (robot.UIDigitalVals[4])
                {
                    firingL.motorSpeed = minFiring;
                    firingR.motorSpeed = minFiring;
                }
                else
                {
                    firingL.motorSpeed = maxFiring;
                    firingR.motorSpeed = maxFiring;
                }
            }
        }

        public void fire()
        {
            aim();
            door.speeds[0] = 50;
            door.targets[0] = 180;
            if (!autonomous)
            {
                collection = false;
            }
        }

        public double getAverageDistance()
        {
            double distance = sonar.GetDistance();
            double sum = 0;
            dist.Add(distance);
            if (dist.Count >= 3)
            {
                dist.RemoveAt(0);
            }
            for (int k = 0; k < dist.Count; k++)
            {
                sum += (double)dist[k];
            }
            if (dist.Count > 0)
            {
                return sum / dist.Count;
            }
            else
            {
                return 0;
            }

        }

        public float getAverageRight(float right1)
        {
            float sum = 0;
            right.Add(right1);
            if (right.Count >= 3)
            {
                right.RemoveAt(0);
            }
            for (int k = 0; k < dist.Count; k++)
            {
                if (k == 0)
                {
                    sum += (float)right[k] * 2;
                }
                else {
                    sum += (float)right[k];
                }
            }
            if (right.Count > 0)
            {
                return sum / right.Count;
            }
            else
            {
                return 0;
            }
        }

        public float getAverageLeft(float left1)
        {
            float sum = 0;
            leftA.Add(left1);
            if (leftA.Count >= 3)
            {
                leftA.RemoveAt(0);
            }
            for (int k = 0; k < dist.Count; k++)
            {
                if (k == 0)
                {
                    sum += (float)leftA[k] * 2;
                }
                else
                {
                    sum += (float)leftA[k];
                }
            }
            if (leftA.Count > 0)
            {
                return sum / leftA.Count;
            }
            else
            {
                return 0;
            }
        }

        //might need to get average of x and y for driving
    }
}
