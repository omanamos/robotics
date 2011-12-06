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
        public const float PI = 3.1415f;
        private readonly string ip;
        private static readonly float SPEED = 0.2f;
        
        private MotionProxy proxy;
        private TextToSpeechProxy tts;

        private float predictedAng;


        public NaoController(string ip)
        {
            try
            {
                this.ip = ip;
                predictedAng = 0f;
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


        public void walkToObject(Communication.Point obj, Communication.Point nao)
        {
            /*        
            float x = (float) (obj.X - nao.X);
            float y = (float) (obj.Y - nao.Y);
            
            Console.WriteLine("nao x axis: " + nao.X);
            Console.WriteLine("nao y axis: " + nao.Y);

            proxy.post.walkTo(x, y, 0.0f);
            */

            Communication.Point p = new Communication.Point();
            p.X = obj.X - nao.X;
            p.Y = obj.Y - nao.Y;            
            p.Z = getRightAng(nao.Z);

            rotate(p);
            
            double newZ = Math.Atan2(p.Y, p.X);
            if (newZ > PI)
                newZ = PI;
            else if (newZ < -1 * PI)
                newZ = -1 * PI;
            p.X = Math.Round(p.X, 2);
            p.Y = Math.Round(p.Y, 2);
            newZ = Math.Round(newZ, 2);
            Console.WriteLine("Object: ({0}, {1}, {2}), Nao: ({3}, {4}, {5}), New Loc: ({6}, {7}, {8}) @{9}",
                obj.X, obj.Y, obj.Z, nao.X, nao.Y, nao.Z, p.X, p.Y, p.Z, newZ);
            proxy.post.walkTo((float)p.X, (float)p.Y, (float)newZ);

            predictedAng = (float)newZ + (float)p.Z;
            Console.WriteLine("Predicted Angle: " + predictedAng);
            if (predictedAng > PI)
                predictedAng = predictedAng - PI;
            else if (predictedAng < -1 * PI)
                predictedAng = predictedAng + PI;
        }

        private double getRightAng(double currentAng)
        {
            double possibleAng;
            if (currentAng >= PI / 2)
                possibleAng = currentAng - PI / 2;
            else
                possibleAng = currentAng + PI / 2;

            double currentAnginNao = converToNaoAng(currentAng);
            double possibleAnginNao = converToNaoAng(possibleAng);

            if (Math.Abs(predictedAng - currentAnginNao) > Math.Abs(predictedAng - possibleAnginNao))
                return possibleAnginNao;
            else
                return currentAnginNao;
        }


        private double converToNaoAng(double ang)
        {
            if (ang >= PI / 2)
                return PI - ang;
            else
                return ang;
        }

        // tranform the position of the object respects to Nao's facing direction
        private void rotate(Communication.Point p)
        {
            float x = (float)p.X;
            float y = (float)p.Y;
            float rad = (float)p.Z;

            p.X = (float)(x * Math.Cos(rad) + y * Math.Sin(rad));
            p.Y = (float)(-1 * x * Math.Sin(rad) + y * Math.Cos(rad));
        }

        public void speak(String context)
        {
            Console.WriteLine("Nao is saying: " + context);
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
            return proxy.walkIsActive();
        }

        public void raiseArms()
        {
            String[] joints = {"LShoulderPitch", "RShoulderPitch"};
            proxy.setAngles(joints, 0.0f, 0.5f);
        }
    }
}
