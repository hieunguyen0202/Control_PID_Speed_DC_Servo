using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using ZedGraph;

using static SORFWARE_BTL2.Form1;

namespace SORFWARE_BTL2
{   
    public partial class dothihienthi : Form
    {
        public dothihienthi()
        {
            InitializeComponent();
            Control.CheckForIllegalCrossThreadCalls = false; // cho phep nhan du lieu
        }
        float maxVel;
        float maxAcc;
        float initPos;
        float pos;
        float vel;
        float acc;
        float oldPos;
        float oldPosRef;
        float oldVel;

        float dBrk;
        float dAcc;
        float dVel;
        float dDec;
        float dTot;

        float tBrk;
        float tAcc;
        float tVel;
        float tDec;

        float velSt;

        double oldTime;
        double lastTime;
        double deltaTime;

        short signM;        // 1 = positive change, -1 = negative change
        bool shape;         // true = trapezoidal, false = triangular

        bool isFinished;


        private void dothihienthi_Load(object sender, EventArgs e)
        {
            init1();
            GraphPane mypanne1 = zedGraphVelocity.GraphPane;
            mypanne1.Title.Text = "Velocity";
            mypanne1.XAxis.Title.Text = "Time";
            mypanne1.YAxis.Title.Text = "Giá trị";
            RollingPointPairList list3 = new RollingPointPairList(60000);
            LineItem duongline3 = mypanne1.AddCurve("Đường vận tốc", list3, Color.BlueViolet, SymbolType.None);

            mypanne1.XAxis.Scale.Min = 0;
            mypanne1.XAxis.Scale.Max = 5;
            mypanne1.XAxis.Scale.MinorStep = 0.2;
            mypanne1.XAxis.Scale.MajorStep = 1;

            mypanne1.YAxis.Scale.Min = -600;
            mypanne1.YAxis.Scale.Max = 600;
            // mypanne.YAxis.Scale.MinorStep = 1;
            mypanne1.YAxis.Scale.MajorStep = 100;
            zedGraphVelocity.AxisChange(); //thay doi cac gia tri truc

            GraphPane mypanne2 = zedGraphAcc.GraphPane;
            mypanne2.Title.Text = "Gia tốc";
            mypanne2.XAxis.Title.Text = "Time";
            mypanne2.YAxis.Title.Text = "Giá trị";
            RollingPointPairList list4 = new RollingPointPairList(80000);

            LineItem duongline4 = mypanne2.AddCurve("Đường gia tốc", list4, Color.BlueViolet, SymbolType.None);
            mypanne2.XAxis.Scale.Min = 0;
            mypanne2.XAxis.Scale.Max = 5;
            mypanne2.XAxis.Scale.MinorStep = 0.2;
            mypanne2.XAxis.Scale.MajorStep = 1;

            mypanne2.YAxis.Scale.Min = -600;
            mypanne2.YAxis.Scale.Max = 600;
            // mypanne.YAxis.Scale.MinorStep = 1;
            mypanne2.YAxis.Scale.MajorStep = 100;
            zedGraphAcc.AxisChange(); //thay doi cac gia tri truc


            GraphPane mypanne3 = zedGraphPosRef.GraphPane;
            mypanne3.Title.Text = "Position";
            mypanne3.XAxis.Title.Text = "Time";
            mypanne3.YAxis.Title.Text = "Giá trị";
            RollingPointPairList list5 = new RollingPointPairList(80000);

            LineItem duongline5 = mypanne3.AddCurve("Set value", list5, Color.BlueViolet, SymbolType.None);
            mypanne3.XAxis.Scale.Min = 0;
            mypanne3.XAxis.Scale.Max = 5;
            mypanne3.XAxis.Scale.MinorStep = 0.2;
            mypanne3.XAxis.Scale.MajorStep = 1;

            mypanne3.YAxis.Scale.Min = 0;
            mypanne3.YAxis.Scale.Max = 900;
            // mypanne.YAxis.Scale.MinorStep = 1;
            mypanne3.YAxis.Scale.MajorStep = 100;
            zedGraphPosRef.AxisChange(); //thay doi cac gia tri truc

        }

        private void reset()
        {
            // Reset all state variables	

            pos = initPos;
            oldPos = initPos;
            oldPosRef = 0;
            vel = 0;
            acc = 0;
            oldVel = 0;

            dBrk = 0;
            dAcc = 0;
            dVel = 0;
            dDec = 0;
            dTot = 0;

            tBrk = 0;
            tAcc = 0;
            tVel = 0;
            tDec = 0;

            velSt = 0;
        }
        private void init1()
        {
            // Time variables
            oldTime = 0;
            lastTime = oldTime;
            deltaTime = 0;

            // State variables
            reset();

            // Misc
            signM = 1;      // 1 = positive change, -1 = negative change
            shape = true;   // true = trapezoidal, false = triangular
            isFinished = false;
        }

        private void MotionGenerator(float aMaxVel, float aMaxAcc, float aInitPos) 

        {
            maxVel = aMaxVel;
            maxAcc = aMaxAcc;
            initPos = aInitPos;
            //maxVel(aMaxVel), maxAcc(aMaxAcc), initPos(aInitPos)
            init1();
        }
        //float update(float posRef)
        //{

        //    if (oldPosRef != posRef)  // reference changed
        //    {
        //        isFinished = false;
        //        // Shift state variables
        //        oldPosRef = posRef;
        //        oldPos = pos;
        //        oldVel = vel;
        //        oldTime = lastTime;

        //        // Calculate braking time and distance (in case is needed)
        //        tBrk = Math.Abs(oldVel) / maxAcc;
        //        dBrk = tBrk * Math.Abs(oldVel) / 2;

        //        // Calculate Sign of motion
        //        signM = sign(posRef - (oldPos + sign(oldVel) * dBrk));

        //        if (signM != sign(oldVel))  // means brake is needed
        //        {
        //            tAcc = (maxVel / maxAcc);
        //            dAcc = tAcc * (maxVel / 2);
        //        }
        //        else
        //        {
        //            tBrk = 0;
        //            dBrk = 0;
        //            tAcc = (maxVel - Math.Abs(oldVel)) / maxAcc;
        //            dAcc = tAcc * (maxVel + Math.Abs(oldVel)) / 2;
        //        }

        //        // Calculate total distance to go after braking
        //        dTot = Math.Abs(posRef - oldPos + signM * dBrk);

        //        tDec = maxVel / maxAcc;
        //        dDec = tDec * (maxVel) / 2;
        //        dVel = dTot - (dAcc + dDec);
        //        tVel = dVel / maxVel;

        //        if (tVel > 0)    // trapezoidal shape
        //            shape = true;
        //        else             // triangular shape
        //        {
        //            shape = false;
        //            // Recalculate distances and periods
        //            if (signM != sign(oldVel))  // means brake is needed
        //            {
        //                velSt = (float)Math.Sqrt(maxAcc * (dTot));
        //                tAcc = (velSt / maxAcc);
        //                dAcc = tAcc * (velSt / 2);
        //            }
        //            else
        //            {
        //                tBrk = 0;
        //                dBrk = 0;
        //                dTot = (float)Math.Abs(posRef - oldPos);      // recalculate total distance
        //                velSt = (float)Math.Sqrt(0.5 * oldVel * oldVel + maxAcc * dTot);
        //                tAcc = (velSt - (float)Math.Abs(oldVel)) / maxAcc;
        //                dAcc = tAcc * (velSt + (float)Math.Abs(oldVel)) / 2;
        //            }
        //            tDec = velSt / maxAcc;
        //            dDec = tDec * (velSt) / 2;
        //        }

        //    }

        //    //unsigned long time = mill();		// current timeư
        //    uint time = (uint)GetTickCount();     // current time
        //                                                            // Calculate time since last set-point change
        //    deltaTime = (time - oldTime);
        //    // Calculate new set point
        //    calculateTrapezoidalProfile(posRef);
        //    // Update last time
        //    lastTime = time;

        //    //calculateTrapezoidalProfile(setpoint);
        //    return pos;
        //}



        private float update(float posRef, double lNextTime)
        {
            if (oldPosRef != posRef)  // reference changed
            {
                isFinished = false;
                // Shift state variables
                oldPosRef = posRef;
                oldPos = pos;
                oldVel = vel;
                oldTime = lastTime;

                // Calculate braking time and distance (in case is needed)
                tBrk = Math.Abs(oldVel) / maxAcc;
                dBrk = tBrk * Math.Abs(oldVel) / 2;

                // Calculate Sign of motion
                signM = sign(posRef - (oldPos + sign(oldVel) * dBrk));

                if (signM != sign(oldVel))  // means brake is needed
                {
                    tAcc = (maxVel / maxAcc);
                    dAcc = tAcc * (maxVel / 2);
                }
                else
                {
                    tBrk = 0;
                    dBrk = 0;
                    tAcc = (maxVel - Math.Abs(oldVel)) / maxAcc;
                    dAcc = tAcc * (maxVel + Math.Abs(oldVel)) / 2;
                }

                // Calculate total distance to go after braking
                dTot = Math.Abs(posRef - oldPos + signM * dBrk);

                tDec = maxVel / maxAcc;
                dDec = tDec * (maxVel) / 2;
                dVel = dTot - (dAcc + dDec);
                tVel = dVel / maxVel;

                if (tVel > 0)    // trapezoidal shape
                    shape = true;
                else             // triangular shape
                {
                    shape = false;
                    // Recalculate distances and periods
                    if (signM != sign(oldVel))  // means brake is needed
                    {
                        velSt = (float)Math.Sqrt(maxAcc * (dTot));
                        tAcc = (velSt / maxAcc);
                        dAcc = tAcc * (velSt / 2);
                    }
                    else
                    {
                        tBrk = 0;
                        dBrk = 0;
                        dTot = (float)Math.Abs(posRef - oldPos);      // recalculate total distance
                        velSt = (float)Math.Sqrt(0.5 * oldVel * oldVel + maxAcc * dTot);
                        tAcc = (velSt - (float)Math.Abs(oldVel)) / maxAcc;
                        dAcc = tAcc * (velSt + (float)Math.Abs(oldVel)) / 2;
                    }
                    tDec = velSt / maxAcc;
                    dDec = tDec * (velSt) / 2;
                }

            }

            //unsigned long time = mill();		// current timeư
            double time = lNextTime;        // current time
                                            // Calculate time since last set-point change
            deltaTime = (time - oldTime);
            // Calculate new set point
            calculateTrapezoidalProfile(posRef);
            // Update last time
            lastTime = time;

            //calculateTrapezoidalProfile(setpoint);
            return pos;
        }
        private void calculateTrapezoidalProfile(float posRef)
        {

            double chx = Convert.ToDouble(deltaTime);    // conversion from milliseconds to seconds	
            float t = (float)chx;
                

            if (shape)   // trapezoidal shape
            {
                if (t <= (tBrk + tAcc))
                {
                    pos = oldPos + oldVel * t + (float)(signM * 0.5 * maxAcc * t * t);
                    vel = oldVel + signM * maxAcc * t;
                    acc = signM * maxAcc;
                }
                else if (t > (tBrk + tAcc) && t < (tBrk + tAcc + tVel))
                {
                    pos = oldPos + signM * (-dBrk + dAcc + maxVel * (t - tBrk - tAcc));
                    vel = signM * maxVel;
                    acc = 0;
                }
                else if (t >= (tBrk + tAcc + tVel) && t < (tBrk + tAcc + tVel + tDec))
                {
                    pos = oldPos + (float)(signM * (-dBrk + dAcc + dVel + maxVel * (t - tBrk - tAcc - tVel)) - 0.5 * maxAcc * (t - tBrk - tAcc - tVel) * (t - tBrk - tAcc - tVel));
                    vel = signM * (maxVel - maxAcc * (t - tBrk - tAcc - tVel));
                    acc = -signM * maxAcc;
                }
                else
                {
                    pos = posRef;
                    vel = 0;
                    acc = 0;
                    isFinished = true;
                }
            }
            else            // triangular shape
            {
                if (t <= (tBrk + tAcc))
                {
                    pos = oldPos + oldVel * t + (float)(signM * 0.5 * maxAcc * t * t);
                    vel = oldVel + signM * maxAcc * t;
                    acc = signM * maxAcc;
                }
                else if (t > (tBrk + tAcc) && t < (tBrk + tAcc + tDec))
                {
                    pos = oldPos + (float)(signM * (-dBrk + dAcc + velSt * (t - tBrk - tAcc) - 0.5 * maxAcc * (t - tBrk - tAcc) * (t - tBrk - tAcc)));
                    vel = signM * (velSt - maxAcc * (t - tBrk - tAcc));
                    acc = -signM * maxAcc;
                }
                else
                {
                    pos = posRef;
                    vel = 0;
                    acc = 0;
                    isFinished = true;
                }

            }

        }

        private bool getFinished()
        {
            return isFinished;
        }

        private float getVelocity()
        {
            return vel;
        }

        private float getAcceleration()
        {
            return acc;
        }

        private void setMaxVelocity(float aMaxVel)
        {
            maxVel = aMaxVel;
        }

        private void setMaxAcceleration(float aMaxAcc)
        {
            maxAcc = aMaxAcc;
        }

        private void setInitPosition(float aInitPos)
        {
            initPos = aInitPos;
            pos = aInitPos;
            oldPos = aInitPos;
        }

        private short sign(float aVal)
        {
            if (aVal < 0)
                return -1;
            else if (aVal > 0)
                return 1;
            else
                return 0;
        }
        private void button1_Click(object sender, EventArgs e)
        {
            double dPosRef = Convert.ToDouble(txbPosMax.Text);
            double dVelMax = Convert.ToDouble(txbVelMax.Text);
            double dAccMax = Convert.ToDouble(txbAccMax.Text);

            MotionGenerator((float)dVelMax, (float)dAccMax, 0);

            float[] fCalPosition = new float[200];
            float[] fCalVel = new float[200];
            float[] fCalAcc = new float[200];
          //  double positionRef = dPosRef;
            float[] positionRefC = new float[200];
            double[] fTime = new double[200];
            for (int i = 0; i < 200; i++)
            {
                positionRefC[i] = (float)dPosRef;
                if (i == 0)
                {
                    fTime[i] = 0.025;

                }
                else fTime[i] = (fTime[i - 1] + 0.025);
            }    
            for (int i = 0; i < 200; i++)
            {
                fCalPosition[i] = update(positionRefC[i], fTime[i]);
                fCalVel[i] = getVelocity();
                fCalAcc[i] = getAcceleration();
            }
            if (getFinished()) { };
                reset();

            if (zedGraphVelocity.GraphPane.CurveList.Count <= 0)
                return;
            if (zedGraphAcc.GraphPane.CurveList.Count <= 0)
                return;
            if (zedGraphPosRef.GraphPane.CurveList.Count <= 0)
                return;
            LineItem duongline3 = zedGraphVelocity.GraphPane.CurveList[0] as LineItem;
            LineItem duongline4 = zedGraphAcc.GraphPane.CurveList[0] as LineItem;
            LineItem duongline5 = zedGraphPosRef.GraphPane.CurveList[0] as LineItem;
            if (duongline5 == null)
                return;
            if (duongline3 == null)
                return;
            if (duongline4 == null)
                return;
            
            IPointListEdit list5 = duongline5.Points as IPointListEdit;
            IPointListEdit list3 = duongline3.Points as IPointListEdit;
            IPointListEdit list4 = duongline4.Points as IPointListEdit;
           
            if (list5 == null)
                return;
            if (list3 == null)
                return;
            if (list4 == null)
                return;
           

            for (int i = 0; i < 200; i++)
            {
                list5.Add(fTime[i], dPosRef);
                zedGraphPosRef.AxisChange();
                zedGraphPosRef.Invalidate();
            }

            for (int i = 0 ; i < 200; i++ )
            {
                list3.Add(fTime[i], fCalVel[i]);
                zedGraphVelocity.AxisChange();
                zedGraphVelocity.Invalidate();
            }
            for (int i = 0; i < 200; i++)
            {
                list4.Add(fTime[i], fCalAcc[i]);
                zedGraphAcc.AxisChange();
                zedGraphAcc.Invalidate();
            }
        }
        byte[] bSTX = { 0x02 };
        byte[] bMOVL = { 0x4D, 0x4F, 0x56, 0x4C };
        byte[] bGPOS = { 0x47, 0x50, 0x4F, 0x53 };
        byte[] bSTT = { 0x47, 0x53, 0x54, 0x54 };
        byte[] bGVEL = { 0x47, 0x56, 0x45, 0x4C };
        //option
        byte[] bOPT = { 0x00, 0x00, 0x00 };
        //DATA
        byte[] bDATA = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

        byte[] bSYNC = { 0x16 };
        byte[] bACK = { 0x06 };
        byte[] bETX = { 0x03 };

        byte[] bSPID = { 0x53, 0x50, 0x49, 0x44 };
        byte[] bGPID = { 0x47, 0x50, 0x49, 0x44 };
        byte[] bCTUN = { 0x43, 0x54, 0x55, 0x4E };
        byte[] bCRUN = { 0x43, 0x52, 0x55, 0x4E };
        byte[] bCSET = { 0x43, 0x53, 0x45, 0x54 };
        byte[] bGRMS = { 0x47, 0x52, 0x4D, 0x53 };


        private void  btnSendSeting_Click(object sender, EventArgs e)
        {
            byte[] bProtocol = new byte[50];

            Buffer.BlockCopy(bSTX, 0, bProtocol, 0, 1);
            Buffer.BlockCopy(bGPID, 0, bProtocol, 1, 4);
            Buffer.BlockCopy(bOPT, 0, bProtocol, 5, 3);
            Buffer.BlockCopy(bDATA, 0, bProtocol, 8, 8);
            Buffer.BlockCopy(bSYNC, 0, bProtocol, 16, 1);
            Buffer.BlockCopy(bETX, 0, bProtocol, 17, 1);
           // Form1 fr = new Form1();
            //    Form1.bytetostring(bProtocol);
            //Form1.serialPort1.Write(bProtocol, 0, 18);
            //txtReadSerial.Text += "CSET CMD:\r\n";
        }
    }
}
