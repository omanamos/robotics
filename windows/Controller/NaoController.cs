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

        private float currentAng;

        public NaoController(string ip)
        {
            try
            {
                this.ip = ip;
                currentAng = 0f;
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
            this.setStiffness(0.0f);
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
            Communication.Point p = new Communication.Point();
            p.X = nao.X - obj.X;
            p.Y = nao.Y - obj.Y;
            rotate(p, currentAng);
            float newZ = (float)Math.Atan2(p.Y, p.X);
            if (newZ > PI)
                newZ = PI;
            else if (newZ < -1 * PI)
                newZ = -1 * PI;
            currentAng += newZ;
            if (currentAng > PI)
                currentAng = currentAng - 2 * PI;
            else if (currentAng < -1 * PI)
                currentAng = currentAng + 2 * PI;
            //proxy.walkTo((float)p.X, (float)p.Y, newZ);
            //proxy.walkTo(0.5f, 0.5f, /*1.5709f*/0);

            float x = (float) (obj.X - nao.X);
            float y = (float) (obj.Y - nao.Y);

            Console.WriteLine("nao x axis: " + nao.X);
            Console.WriteLine("nao y axis: " + nao.Y);

            proxy.walkTo(x, y, 0);
            //proxy.walkTo(0.5f, 0.5f, /*1.5709f*/0);

        }

        // tranform the position of the object respects to Nao's facing direction
        private void rotate(Communication.Point p, float rad)
        {
            float x = (float)p.X;
            float y = (float)p.Y;

            p.X = (float)(x * Math.Cos(rad) + y * Math.Sin(rad));
            p.Y = (float)(-1 * x * Math.Sin(rad) + y * Math.Cos(rad));
        }

        public void speak(String context)
        {
            if (tts != null)
            {
                tts.say(context);
            }
        }

        public void stop()
        {
            proxy.killAll();
        }
    }
}
