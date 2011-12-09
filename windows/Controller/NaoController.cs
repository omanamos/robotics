using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows;

using Aldebaran.Proxies;
using Communication;
using DataStore;


namespace Controller
{
    public class NaoController
    {
        private const double FUDGE_1 = 0.40;
        private const double FUDGE_2 = 0.30;
        public const float PI = 3.1415f;
        private readonly string ip;
        private static readonly float SPEED = 0.2f;
        
        private MotionProxy proxy;
        private TextToSpeechProxy tts;
        private MainController main;

        private float prevAng;


        public NaoController(string ip, MainController main)
        {
            try
            {
                this.main = main;
                this.ip = ip;
                prevAng = PI;
                proxy = new MotionProxy(ip, 9559);
                if (this.ip != "127.0.0.1")
                {
                    tts = new TextToSpeechProxy(this.ip, 9559);
                }
                else
                {
                    tts = null;
                }
                //this.setStiffness(1.0f);
            }
            catch (Exception e)
            {
                Console.Out.WriteLine("Connect exception: " + e);
            }
        }

        public void exit()
        {
            
            //this.setStiffness(0.0f);
        }

        private void setStiffness(float stiffness)
        {
            float time = 1.0f;
            String[] joints = {"HeadYaw", "HeadPitch",
                                  "LShoulderPitch", "RShoulderPitch",
                                  "LShoulderRoll", "RShoulderRoll",
                                  "LElbowYaw", "RElbowYaw",
                                  "LElbowRoll", "RElbowRoll",
                                  "RHipYawPitch", "LHipYawPitch",
                                  "LHipPitch", "RHipPitch",
                                  "LKneePitch", "RKneePitch",
                                  "LAnklePitch", "RAnklePitch",
                                  "LHipRoll", "RHipRoll",
                                  "LAnkleRoll", "RAnkleRoll"};
            float[] stiff = {stiffness, stiffness, stiffness, stiffness, stiffness, stiffness,
                                stiffness, stiffness, stiffness, stiffness, stiffness, stiffness,
                                stiffness, stiffness, stiffness, stiffness, stiffness, stiffness,
                                stiffness, stiffness, stiffness, stiffness};
            float[] times = {time, time, time, time, time, time,
                                time, time, time, time, time, time,
                                time, time, time, time, time, time,
                                time, time, time, time};

            
            proxy.stiffnessInterpolation((Object)joints, (Object)stiff, (Object)times);
        }


        public void walkToObject(Communication.Point obj, Communication.Point nao, bool fudge1)
        {
            Communication.Point p = new Communication.Point();
            p.X = obj.X - nao.X;
            p.Y = obj.Y - nao.Y;
            p.Z = checkAngle(nao.Z);
            if (Math.Abs(p.Z) > PI / 2)
            {
                Console.WriteLine("Facing Kinect");
            }

            rotate(p);
            
            double newZ = Math.Atan2(p.Y, p.X);
            if (newZ > PI)
                newZ = PI;
            else if (newZ < -1 * PI)
                newZ = -1 * PI;
            p.X = Math.Round(p.X, 2);
            p.Y = Math.Round(p.Y, 2);
            newZ = Math.Round(newZ, 2);
            Console.WriteLine("Object: ({0}, {1}, {2}),\nNao: ({3}, {4}, {5}),\nNew Loc: ({6}, {7}, {8}) @{9}",
                obj.X, obj.Y, obj.Z, nao.X, nao.Y, nao.Z, p.X, p.Y, p.Z, newZ);
            fudge(p, fudge1);
            Console.WriteLine("Fudged: ({0}, {1}, {2})", p.X, p.Y, p.Z);
            proxy.post.walkTo((float)p.X, (float)p.Y, (float)newZ);

            prevAng = (float)newZ + (float)p.Z;
            if (prevAng > PI)
                prevAng = prevAng - 2 * PI;
            else if (prevAng < -1 * PI)
                prevAng = prevAng + 2 * PI;
            Console.WriteLine("PPPPPPPPPPPPPPPPPPPPredicted Angle: " + prevAng);
        }

        private void fudge(Communication.Point delta, bool fudge1)
        {
            double theta = Math.Atan2(delta.Y, delta.X);
            double hyp = Math.Sqrt(Math.Pow(delta.X, 2) + Math.Pow(delta.Y, 2)) - (fudge1 ? FUDGE_1 : FUDGE_2);
            delta.X = hyp * Math.Cos(theta);
            delta.Y = hyp * Math.Sign(theta);
            Console.WriteLine("Theta: " + theta);
        }

        private double checkAngle(double angleFromKinect)
        {
            // Make the angles 0 - 360
            angleFromKinect = angleFromKinect < 0 ? angleFromKinect + 2 * PI : angleFromKinect;
            double prevAng360 = prevAng < 0 ? prevAng + 2 * PI : prevAng;

            // Calculate opposite angle from angleFromKinect
            double possibleAng = angleFromKinect > PI ? angleFromKinect - PI : angleFromKinect + PI;

            if (Math.Abs(prevAng360 - angleFromKinect) > Math.Abs(prevAng360 - possibleAng))
            {
                Console.WriteLine("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX: Flipping angle");
                return possibleAng > PI ? possibleAng - 2 * PI : possibleAng;
            }
            else
                return angleFromKinect > PI ? angleFromKinect - 2 * PI : angleFromKinect;
        }

        // tranform the position of the object wrt to Nao's facing direction
        private void rotate(Communication.Point p)
        {
            float x = (float)p.X;
            float y = (float)p.Y;
            float rad = (float)p.Z;

            p.X = (float)(x * Math.Cos(rad) + y * Math.Sin(rad));
            p.Y = (float)(-x * Math.Sin(rad) + y * Math.Cos(rad));
        }

        public void speak(String context)
        {
            Console.WriteLine("NNNNNNNNNNNAAAAAAAAAAAAAAAOOOOOOOOOOOO: Nao is saying: " + context);
            if (tts != null)
            {
                tts.say(context);
            }
        }

        public void stop()
        {
            proxy.post.killWalk();
            proxy.post.walkTo(0.01f, 0.0f, 0.0f);
        }

        public bool isWalking()
        {
            bool rtn = proxy.walkIsActive();
            this.main.setWalking(rtn);
            return rtn;
        }

        public void raiseArms()
        {
            String[] joints = {"LShoulderPitch", "RShoulderPitch"};
            proxy.setAngles(joints, 0.0f, 0.5f);
        }
    }
}
