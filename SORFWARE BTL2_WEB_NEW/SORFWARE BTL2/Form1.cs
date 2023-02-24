using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.IO.Ports;
using ZedGraph;


namespace SORFWARE_BTL2
{
    public partial class Form1 : Form
    {
        string SDatas = String.Empty; // Khai báo chuỗi để lưu dữ liệu cảm biến gửi qua Serial
                                      // string SRealTime = String.Empty; // Khai báo chuỗi để lưu thời gian gửi qua Serial
                                      // int status = 0; // Khai báo biến để xử lý sự kiện vẽ đồ thị
        double[] realtime = new double[1000]; //Khai báo biến thời gian để vẽ đồ thị
        double[] datas = new double[50000]; //Khai báo biến dữ liệu cảm biến để vẽ đồ thị
        double[] fPositionDatas = new double[50000];

        void InitCom()
        {
            ccbComName.DataSource = SerialPort.GetPortNames(); // doc cong com tren may tinh
            ccbComJSON.DataSource = SerialPort.GetPortNames();
        }

        string text1;
        void bytetostring(byte[] data)
        {
            text1 = "\r\n";
            text1 += txtReadSerial.Text;
            txtReadSerial.Text = text1;
            text1 = "";
            string[] s0 = { "0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "A", "B", "C", "D", "E", "F" };
            for (int i = 0; i < 18; i++)
            {
                int a = data[i] / 16;
                int b = data[i] % 16;
                text1 += s0[a] + s0[b] + " ";

            }
            text1 += txtReadSerial.Text;
            txtReadSerial.Text = text1;
            //  txtReadSerial.Text += "\r\n";
        }
        public Form1()
        {
            InitializeComponent();
            Control.CheckForIllegalCrossThreadCalls = false; // cho phep nhan du lieu

        }

        private void Form1_Load(object sender, EventArgs e)
        {
            InitCom();
            GraphPane mypanne = zedGraphPID.GraphPane;
            mypanne.Title.Text = "PID";
            mypanne.XAxis.Title.Text = "Time";
            mypanne.YAxis.Title.Text = "Position";
            RollingPointPairList list1 = new RollingPointPairList(60000);
            LineItem duongline1 = mypanne.AddCurve("Đường pid", list1, Color.Green, SymbolType.None);
            duongline1.Line.Width = 4;
            RollingPointPairList list2 = new RollingPointPairList(60000);
            LineItem duongline2 = mypanne.AddCurve("Set value", list2, Color.Blue, SymbolType.None);
            duongline2.Line.Width = 3;
            mypanne.XAxis.Scale.Min = 0;
            mypanne.XAxis.Scale.Max = 2;
            mypanne.XAxis.Scale.MinorStep = 0.1;
            mypanne.XAxis.Scale.MajorStep = 0.5;

            mypanne.YAxis.Scale.Min = 0;
            mypanne.YAxis.Scale.Max = 150;
            mypanne.YAxis.Scale.MinorStep = 1;
            mypanne.YAxis.Scale.MajorStep = 5;
            zedGraphPID.AxisChange(); //thay doi cac gia tri truc


            init1();
            GraphPane mypanne1 = zedGraphVelocity.GraphPane;
            mypanne1.Title.Text = "Velocity";
            mypanne1.XAxis.Title.Text = "Time";
            mypanne1.YAxis.Title.Text = "Giá trị";
            RollingPointPairList list3 = new RollingPointPairList(60000);
            LineItem duongline3 = mypanne1.AddCurve("Đường vận tốc", list3, Color.BlueViolet, SymbolType.None);
            duongline3.Line.Width = 5;

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
            duongline4.Line.Width = 5;
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
            duongline5.Line.Width = 3;
            RollingPointPairList list6 = new RollingPointPairList(80000);

            LineItem duongline6 = mypanne3.AddCurve("POSITION", list6, Color.Red, SymbolType.None);
            duongline6.Line.Width = 5;
            RollingPointPairList list7 = new RollingPointPairList(80000);

            LineItem duongline7 = mypanne3.AddCurve("POS lý thuyết", list7, Color.Green, SymbolType.None);
            duongline7.Line.Width = 4;
            mypanne3.XAxis.Scale.Min = 0;
            mypanne3.XAxis.Scale.Max = 5;
            mypanne3.XAxis.Scale.MinorStep = 0.2;
            mypanne3.XAxis.Scale.MajorStep = 1;

            mypanne3.YAxis.Scale.Min = 0;
            mypanne3.YAxis.Scale.Max = 900;
            // mypanne.YAxis.Scale.MinorStep = 1;
            mypanne3.YAxis.Scale.MajorStep = 100;
            zedGraphPosRef.AxisChange(); //thay doi cac gia tri truc
            // draw graph speed
            GraphPane myPaneYk = zedGraph_Value.GraphPane;
            myPaneYk.Title.Text = "Speed (RPM)";
            myPaneYk.XAxis.Title.Text = "Time";
            myPaneYk.YAxis.Title.Text = "Giá trị";
            RollingPointPairList list_value = new RollingPointPairList(80000);

            LineItem duongline_value = myPaneYk.AddCurve("Set point", list_value, Color.BlueViolet, SymbolType.None);
            duongline_value.Line.Width = 3;
            RollingPointPairList list_current = new RollingPointPairList(80000);

            LineItem duongline_current = myPaneYk.AddCurve("Current", list_current, Color.OrangeRed, SymbolType.None);
            duongline_current.Line.Width = 3;
        }

        double intsetpoint, intcurrent, inte;
        private void Draw(string setpoint, string current, string e)
        {
            // Make sure that the curvelist has at least one curve  
            double.TryParse(setpoint, out intsetpoint);
            double.TryParse(current, out intcurrent);
            //   double.TryParse(udk, out intudk);
            double.TryParse(e, out inte);//add
            if (zedGraph_Value.GraphPane.CurveList.Count <= 0)
                return;

            // Get the first CurveItem in the graph
            LineItem curve = zedGraph_Value.GraphPane.CurveList[0] as LineItem;
            LineItem curve1 = zedGraph_Value.GraphPane.CurveList[1] as LineItem;
            //LineItem curve2 = zedGraph_Udk.GraphPane.CurveList[0] as LineItem; //add
            if (curve == null)
                return;
            if (curve1 == null)
                return;

            // Get the PointPairList
            IPointListEdit list001 = curve.Points as IPointListEdit;
            IPointListEdit list100 = curve1.Points as IPointListEdit;
            //IPointListEdit list2 = curve2.Points as IPointListEdit; //add
            // If this is null, it means the reference at curve.Points does not
            // support IPointListEdit, so we won't be able to modify it
            if (list001 == null)
                return;
            if (list100 == null)
                return;

            // Time is measured in seconds
            double time = (Environment.TickCount - TickStart);

            list001.Add(time, intsetpoint);
            list100.Add(time, intcurrent / 100);
            // list2.Add(time, intudk / 100);
            // Keep the X scale at a rolling 30 second interval, with one
            // major step between the max X value and the end of the axis
            Scale xScale = zedGraph_Value.GraphPane.XAxis.Scale;
            if (time > xScale.Max - xScale.MajorStep)
            {
                xScale.Max = time + xScale.MajorStep;
                xScale.Min = 0;
            }
            //
            //Scale xScale1 = zedGraph_Udk.GraphPane.XAxis.Scale;
            //if (time > xScale1.Max - xScale1.MajorStep)
            //{
            //    xScale1.Max = time + xScale1.MajorStep;
            //    xScale1.Min = 0;
            //}

            // Make sure the Y axis is rescaled to accommodate actual data
            zedGraph_Value.AxisChange();
            // zedGraph_Udk.AxisChange();
            // Force a redraw
            zedGraph_Value.Invalidate();
            //  zedGraph_Udk.Invalidate();
            //zedGraph_Speed.Refresh(); //add
            //zedGraph_Udk.Refresh(); //add
            //Add

        }

        private void ClearGraphSPEED()
        {
            zedGraph_Value.GraphPane.CurveList.Clear(); // Xóa đường
            zedGraph_Value.GraphPane.GraphObjList.Clear(); // Xóa đối tượng

            zedGraph_Value.AxisChange();
            zedGraph_Value.Invalidate();
            GraphPane myPaneYk = zedGraph_Value.GraphPane;
            myPaneYk.Title.Text = "Speed (RPM)";
            myPaneYk.XAxis.Title.Text = "Time";
            myPaneYk.YAxis.Title.Text = "Giá trị";
            RollingPointPairList list_value = new RollingPointPairList(80000);

            LineItem duongline_value = myPaneYk.AddCurve("Set point", list_value, Color.BlueViolet, SymbolType.None);
            duongline_value.Line.Width = 3;
            RollingPointPairList list_current = new RollingPointPairList(80000);

            LineItem duongline_current = myPaneYk.AddCurve("Current", list_current, Color.OrangeRed, SymbolType.None);
            duongline_current.Line.Width = 3;
        }
        private void ClearZedGraph()
        {

            zedGraphVelocity.GraphPane.CurveList.Clear(); // Xóa đường
            zedGraphVelocity.GraphPane.GraphObjList.Clear(); // Xóa đối tượng

            zedGraphVelocity.AxisChange();
            zedGraphVelocity.Invalidate();
            GraphPane mypanne1 = zedGraphVelocity.GraphPane;
            mypanne1.Title.Text = "Velocity";
            mypanne1.XAxis.Title.Text = "Time";
            mypanne1.YAxis.Title.Text = "Giá trị";
            RollingPointPairList list3 = new RollingPointPairList(60000);
            LineItem duongline3 = mypanne1.AddCurve("Đường vận tốc", list3, Color.BlueViolet, SymbolType.None);
            duongline3.Line.Width = 5;

            mypanne1.XAxis.Scale.Min = 0;
            mypanne1.XAxis.Scale.Max = 5;
            mypanne1.XAxis.Scale.MinorStep = 0.2;
            mypanne1.XAxis.Scale.MajorStep = 1;

            mypanne1.YAxis.Scale.Min = -600;
            mypanne1.YAxis.Scale.Max = 600;
            // mypanne.YAxis.Scale.MinorStep = 1;
            mypanne1.YAxis.Scale.MajorStep = 100;
            zedGraphVelocity.AxisChange(); //thay doi cac gia tri truc

            zedGraphAcc.GraphPane.CurveList.Clear(); // Xóa đường
            zedGraphAcc.GraphPane.GraphObjList.Clear(); // Xóa đối tượng

            zedGraphAcc.AxisChange();
            zedGraphAcc.Invalidate();
            GraphPane mypanne2 = zedGraphAcc.GraphPane;
            mypanne2.Title.Text = "Gia tốc";
            mypanne2.XAxis.Title.Text = "Time";
            mypanne2.YAxis.Title.Text = "Giá trị";
            RollingPointPairList list4 = new RollingPointPairList(80000);

            LineItem duongline4 = mypanne2.AddCurve("Đường gia tốc", list4, Color.BlueViolet, SymbolType.None);
            duongline4.Line.Width = 5;
            mypanne2.XAxis.Scale.Min = 0;
            mypanne2.XAxis.Scale.Max = 5;
            mypanne2.XAxis.Scale.MinorStep = 0.2;
            mypanne2.XAxis.Scale.MajorStep = 1;

            mypanne2.YAxis.Scale.Min = -600;
            mypanne2.YAxis.Scale.Max = 600;
            // mypanne.YAxis.Scale.MinorStep = 1;
            mypanne2.YAxis.Scale.MajorStep = 100;
            zedGraphAcc.AxisChange(); //thay doi cac gia tri truc


            zedGraphPosRef.GraphPane.CurveList.Clear(); // Xóa đường
            zedGraphPosRef.GraphPane.GraphObjList.Clear(); // Xóa đối tượng

            zedGraphPosRef.AxisChange();
            zedGraphPosRef.Invalidate();
            GraphPane mypanne3 = zedGraphPosRef.GraphPane;
            mypanne3.Title.Text = "Position";
            mypanne3.XAxis.Title.Text = "Time";
            mypanne3.YAxis.Title.Text = "Giá trị";
            RollingPointPairList list5 = new RollingPointPairList(80000);

            LineItem duongline5 = mypanne3.AddCurve("Set value", list5, Color.BlueViolet, SymbolType.None);
            duongline5.Line.Width = 3;
            RollingPointPairList list6 = new RollingPointPairList(80000);

            LineItem duongline6 = mypanne3.AddCurve("POSITION", list6, Color.Red, SymbolType.None);
            duongline6.Line.Width = 5;
            RollingPointPairList list7 = new RollingPointPairList(80000);

            LineItem duongline7 = mypanne3.AddCurve("POS lý thuyết", list7, Color.Green, SymbolType.None);
            duongline7.Line.Width = 4;
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


        int nLenGraphPID;
        int nLenGraphPositionData = 0;
        //  int tong = 0;
        public void draw()
        {
            if (zedGraphPID.GraphPane.CurveList.Count <= 0)
                return;
            LineItem duongline1 = zedGraphPID.GraphPane.CurveList[0] as LineItem;
            LineItem duongline2 = zedGraphPID.GraphPane.CurveList[1] as LineItem;
            if (duongline1 == null)
                return;
            if (duongline2 == null)
                return;
            IPointListEdit list1 = duongline1.Points as IPointListEdit;
            IPointListEdit list2 = duongline2.Points as IPointListEdit;
            if (list1 == null)
                return;
            if (list2 == null)
                return;
            for (int i = 0; i < 200; i++)
            {
                if (i == 0)
                {
                    realtime[i] = 0.01;
                }
                else realtime[i] = realtime[i - 1] + 0.01;
            }

            double setpoint = (Convert.ToDouble(txbSetpoint.Text) + 1);


            for (int i = 0; i < nLenGraphPID; i++)
            {
                list1.Add(realtime[i], datas[i]); // Thêm điểm trên đồ thị
                list2.Add(realtime[i], setpoint);




                zedGraphPID.AxisChange();
                zedGraphPID.Invalidate();
            }



        }

        private void Form1_KeyDown(object sender, KeyEventArgs e)
        {
            switch (e.KeyCode)
            {
                case Keys.Escape:
                    Close();
                    break;
            }
        }

        private void btnOpen_Click(object sender, EventArgs e)
        {
            if (btnOpen.Text == "Open")
            {
                if (ccbBaudrate.Text != "4800" && ccbBaudrate.Text != "9600" && ccbBaudrate.Text != "19200" && ccbBaudrate.Text != "115200")
                {
                    MessageBox.Show("Chưa chọn Baudrate");
                }
                else
                {
                    serialPort1.PortName = ccbComName.Text;
                    serialPort1.Open();
                    btnOpen.Text = "Close";
                    Status_Label.Text = "Connected";
                    serialPort1.BaudRate = int.Parse(ccbBaudrate.Text);
                    ccbBaudrate.Enabled = false;
                    ccbComName.Enabled = false;
                    //   radioButton1.Enabled = true;
                    //   radioButton2.Enabled = true;
                    MessageBox.Show("Đã kết nối", "STATUS", MessageBoxButtons.OK, MessageBoxIcon.Exclamation);
                }
            }
            else if (btnOpen.Text == "Close")
            {
                if (serialPort1.IsOpen == true)
                {
                    // timer.Stop();
                    btnOpen.Text = "Open";
                    serialPort1.DiscardInBuffer();
                    serialPort1.DiscardOutBuffer();

                    serialPort1.Close();
                    Status_Label.Text = "Disconnected";
                    //  radioButton2.Enabled = false;
                    //  radioButton1.Enabled = false;
                    ccbBaudrate.Enabled = true;
                    ccbComName.Enabled = true;
                    txtReadSerial.ResetText();
                    MessageBox.Show("Đã ngắt kết nối", "STATUS", MessageBoxButtons.OK, MessageBoxIcon.Information);
                }
            }
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
        string text2;
        string Setpoint, Value, udk, ep;
        int dem = 0;
        // byte[] truyengiup = new byte[18] {0x02, 0x53, 0x45, 0x52, 0x56, 0x00, 0x00, 0x00, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x06, 0x03 }
      //  byte[] truyengiup = new byte[18] { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
        private void serialPort1_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            if (aset != 1 ) {

                byte[] dataReceive = new byte[18];
                
                
                    text2 = "Receive:\r\n";
                    text2 += txtReadSerial.Text;
                    txtReadSerial.Text = text2;
                    text2 = "";
                
                    for (int i = 0; i < 18; i++)
                    {
                        dataReceive[i] = (byte)serialPort1.ReadByte();
                    }
                    bytetostring(dataReceive);

                // txtReadSerial.Text += "Receive:\r\n";

                //   txtReadSerial.Text += "\r\n"; 

                //   txtReadSerial.SelectionStart = txtReadSerial.TextLength;
                //  txtReadSerial.ScrollToCaret();

                byte[] bProtocolBuffer = new byte[18];
                byte[] bProtocolData = new byte[8];
                byte[] bProtocolOption = new byte[3];
                ushort nIndex = 0;
                ushort nPosition = 0;


                //chuoi nhan PID
                string plus = "";
                
                for (int i = 0; i < 18; i++)
                {
                    bProtocolBuffer[i] = dataReceive[i];
                }
                for (int i = 1; i <= 4; i++)
                {

                    char ch = Convert.ToChar(bProtocolBuffer[i]);

                    plus += ch.ToString();

                }
                txbTest.Text = plus.ToString();
                string chuoighep = Convert.ToString(txbTest.Text);
                //  textBox1.Text = chuoighep.ToString();

                for (int i = 5; i <= 7; i++)
                {
                    bProtocolOption[i - 5] = bProtocolBuffer[i];
                }
                for (int i = 8; i <= 15; i++)
                {
                    bProtocolData[i - 8] = bProtocolBuffer[i];
                }

               
                int kq = string.Compare(chuoighep, "GPID");
                string hienthi = "";
                if (kq == 0)
                {
                    //  MessageBox.Show("dung", "note");
                    nLenGraphPID = (bProtocolData[0] << 8) + bProtocolData[1];
                    nIndex = Convert.ToUInt16((bProtocolData[2] << 8) + bProtocolData[3]);
                    nPosition = Convert.ToUInt16((bProtocolData[6] << 8) + bProtocolData[7]);
                    datas[nIndex] = nPosition;
                   
                    if (nIndex != (bProtocolData[0] << 8) + bProtocolData[1])
                    {
                        draw();
                        int pSect = (bProtocolData[4] << 8) + bProtocolData[5];
                        ushort pRMS = Convert.ToUInt16(pSect);
                        double gtriset = (Convert.ToDouble(txbSetpoint.Text) + 1);
                        double ss = gtriset + (double)pRMS;
                        string m_gtriset = Convert.ToString(gtriset);
                        string m_ss = Convert.ToString(ss);
                        // txbRecieError.Text = pRMS.ToString();
                        hienthi += "THÔNG SỐ CỦA BỘ ĐIỀU KHIỂN PID\n";
                        hienthi += "\r\n";
                        hienthi += "Với các thông số Kp = " + txbKp.Text + " , Ki = " + txbKi.Text + " , Kd = " + txbKd.Text + "-> Ta tính được\n";
                        hienthi += "\r\n";
                        hienthi += "Giá trị đặt = " + m_gtriset + "\n";
                        hienthi += "\r\n";
                        hienthi += "Giá trị thực = " + m_ss + "\n";
                        hienthi += "\r\n";
                        hienthi += "Sai số = ";
                        hienthi += pRMS.ToString();
                        txbRecieError.Text = hienthi.ToString();

                    }

                }
               int kq1 = string.Compare(chuoighep, "GRMS");
                // ushort nnIndex = 0;
                // ushort nnPosition = 0;
                if (kq1 == 0)
                {
                    //  MessageBox.Show("dung", "note");
                    nLenGraphPositionData = (bProtocolData[0] << 8) + bProtocolData[1];
                    nIndex = Convert.ToUInt16((bProtocolData[2] << 8) + bProtocolData[3]);
                    nPosition = Convert.ToUInt16((bProtocolData[6] << 8) + bProtocolData[7]);
                    fPositionDatas[nIndex] = nPosition;

                    if (nLenGraphPositionData == nIndex)
                    {
                        MessageBox.Show("YOU WANT TO DRAW POSITION");
                        // DrawOperationGraph();
                        // giatri = 2;


                        for (int i = 0; i < 1000; i++)
                        {
                            if (i == 0)
                            {
                                realtime[i] = 0.01;
                            }
                            else realtime[i] = realtime[i - 1] + 0.01;
                        }
                        if (zedGraphPosRef.GraphPane.CurveList.Count <= 1)
                            return;

                        LineItem duongline6 = zedGraphPosRef.GraphPane.CurveList[1] as LineItem;

                        if (duongline6 == null)
                            return;

                        IPointListEdit list6 = duongline6.Points as IPointListEdit;


                        if (list6 == null)
                            return;


                        for (int i = 0; i < nLenGraphPositionData; i++)
                        {
                            list6.Add(realtime[i], fPositionDatas[i]); // Thêm điểm trên đồ thị

                            zedGraphPosRef.AxisChange();

                            zedGraphPosRef.Invalidate();
                            //   MessageBox.Show("gia trij dung");
                        }


                        int nRms = (bProtocolData[4] << 8) + bProtocolData[5];
                        ushort RMS = Convert.ToUInt16(nRms);
                        txbRms.Text = RMS.ToString();
                    }

                }
             
              
            }
            

        }

        private void label2_Click(object sender, EventArgs e)
        {

        }


        byte chuyedoi;

        private void FloatToByteArray(float dNumber, byte[] bOut, byte chuyedoi)
        {
            byte nCountTithes = 0;
            bOut[0] = (byte)dNumber;
            float nTithes = (float)(dNumber - bOut[0]);
            do
            {
                nTithes *= 10;
                byte v = nCountTithes++;
                if (Convert.ToBoolean(v >> 3))

                    break;

            } while (nTithes <= 1);
            chuyedoi = nCountTithes;
            bOut[1] = (byte)nTithes;

            //dNumber = 0.0f;
            //bOut = BitConverter.GetBytes(dNumber);
        }


        int count_Kd;
        void floattobytearrayforkd(float a, byte[] bOut)
        {
            int id = 2;
            count_Kd = 0;
            bOut[0] = (byte)a;
            float i = a - (float)bOut[0];
            do
            {
                i *= 10;
                count_Kd++;

            } while (id-- > 1);
            bOut[1] = (byte)i;
        }

        void FloatToByteArrayWithNipes(double fNumber, byte[] bOut, byte chuyedoi)
        {
            byte nCountTithes = 5;
            bOut[0] = (byte)fNumber;
            float nTithes = (float)(fNumber - bOut[0]);
            float cal = float.Parse(Convert.ToString(Math.Pow(10, nCountTithes)));
            nTithes = nTithes * cal;
            while (nTithes > 255)
            {
                nTithes /= 10;
                nCountTithes--;

            }
            chuyedoi = nCountTithes;
            bOut[1] = (byte)nTithes;



        }

        byte[] bSTX = { 0x02 };
        byte[] bMOVL = { 0x4D, 0x4F, 0x56, 0x4C };
        byte[] bGPOS = { 0x47, 0x50, 0x4F, 0x53 };
        byte[] bSTT = { 0x47, 0x53, 0x54, 0x54 };
        byte[] bGVEL = { 0x47, 0x56, 0x45, 0x4C };
        //option
        byte[] bOPT = { 0x00, 0x00, 0x00 };
        //DATA
        byte[] bDATA = new byte[8] { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

        byte[] bSYNC = { 0x16 };
        byte[] bACK = { 0x06 };
        byte[] bETX = { 0x03 };
        byte[] bSECT = { 0x53, 0x45, 0x43, 0x54 };
        byte[] bSPID = { 0x53, 0x50, 0x49, 0x44 };
        byte[] bGPID = { 0x47, 0x50, 0x49, 0x44 };
        byte[] bCTUN = { 0x43, 0x54, 0x55, 0x4E };
        byte[] bCRUN = { 0x43, 0x52, 0x55, 0x4E };
        byte[] bCSET = { 0x43, 0x53, 0x45, 0x54 };
        byte[] bSPED = { 0x53, 0x50, 0x45, 0x44 };
        byte[] bSERV = { 0x53, 0x45, 0x52, 0x56 };
        byte[] bSTOP = { 0x53, 0x54, 0x4F, 0x50 };
        byte[] bGRMS = { 0x47, 0x52, 0x4D, 0x53 };
        
        int nhan = 0;
        private void btnSend_Click(object sender, EventArgs e)
        {
            double dKp = Convert.ToDouble(txbKp.Text);
            double dKi = Convert.ToDouble(txbKi.Text);
            double dKd = Convert.ToDouble(txbKd.Text);

            byte[] bProtocol = new byte[50];

            byte[] bKp = new byte[2];
            byte[] bKi = new byte[2];
            byte[] bKd = new byte[3];

            //  char ch1 = ( char)dKp;
            float keycode1 = (float)(dKp);
            //char ch2 = (char)dKi;
            float keycode2 = (float)(dKi);
            float keycode3 = (float)(dKd);

            FloatToByteArray(keycode1, bKp, chuyedoi);
            bDATA[0] = bKp[0];
            bDATA[1] = bKp[1];
            FloatToByteArray(keycode2, bKi, chuyedoi);
            bDATA[2] = bKi[0];
            bDATA[3] = bKi[1];
            floattobytearrayforkd(keycode3, bKd);
            bDATA[4] = bKd[0];
            bDATA[5] = bKd[1];
            bDATA[6] = 0x02;


            Buffer.BlockCopy(bSTX, 0, bProtocol, 0, 1);
            Buffer.BlockCopy(bSPID, 0, bProtocol, 1, 4);
            Buffer.BlockCopy(bOPT, 0, bProtocol, 5, 3);
            Buffer.BlockCopy(bDATA, 0, bProtocol, 8, 8);
            Buffer.BlockCopy(bSYNC, 0, bProtocol, 16, 1);
            Buffer.BlockCopy(bETX, 0, bProtocol, 17, 1);
            bytetostring(bProtocol);
           // if (dem == 1)
           // {
                serialPort1.Write(bProtocol, 0, 18);
                txtReadSerial.Text += "SPID CMD:\r\n";
              //  dem = 0;
           // }
            //byte[] nhanxoa = new byte[18];
              nhan = 1;
            
            run11 = 0;

        }

        private void btnTuning_Click(object sender, EventArgs e)
        {
            byte[] bProtocol = new byte[50];

            Buffer.BlockCopy(bSTX, 0, bProtocol, 0, 1);
            Buffer.BlockCopy(bCTUN, 0, bProtocol, 1, 4);
            Buffer.BlockCopy(bOPT, 0, bProtocol, 5, 3);
            Buffer.BlockCopy(bDATA, 0, bProtocol, 8, 8);
            Buffer.BlockCopy(bSYNC, 0, bProtocol, 16, 1);
            Buffer.BlockCopy(bETX, 0, bProtocol, 17, 1);
            bytetostring(bProtocol);
            serialPort1.Write(bProtocol, 0, 18);
            txtReadSerial.Text += "CTUN CMD:\r\n";

        }

        private void btnGetdata_Click(object sender, EventArgs e)
        {
            byte[] bProtocol = new byte[50];

            Buffer.BlockCopy(bSTX, 0, bProtocol, 0, 1);
            Buffer.BlockCopy(bGPID, 0, bProtocol, 1, 4);
            Buffer.BlockCopy(bOPT, 0, bProtocol, 5, 3);
            Buffer.BlockCopy(bDATA, 0, bProtocol, 8, 8);
            Buffer.BlockCopy(bSYNC, 0, bProtocol, 16, 1);
            Buffer.BlockCopy(bETX, 0, bProtocol, 17, 1);
            bytetostring(bProtocol);
            serialPort1.Write(bProtocol, 0, 18);
            txtReadSerial.Text += "GPID CMD:\r\n";
        }

        private void label4_Click(object sender, EventArgs e)
        {

        }

        private void textBox2_TextChanged(object sender, EventArgs e)
        {

        }



        private void tabPage1_Click(object sender, EventArgs e)
        {

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

        private float getPosition()
        {
            return pos;
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
        int n = 0;
        int giatri = 0;

        private void btnDraw_Click(object sender, EventArgs e)
        {
            double dPosRef = Convert.ToDouble(txbPosMax.Text);
            double dVelMax = Convert.ToDouble(txbVelMax.Text);
            double dAccMax = Convert.ToDouble(txbAccMax.Text);
            //  btnReset.Enabled = false;
            MotionGenerator((float)dVelMax, (float)dAccMax, 0);

            float[] fCalPosition = new float[200];
            float[] fCalVel = new float[200];
            float[] fCalAcc = new float[200];
            float[] fCalPo = new float[200];
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
            if (n > 0) {
                if (btnDraw.Enabled = true)
                {
                    DialogResult traloi;
                    traloi = MessageBox.Show("Bạn có chắc muốn xóa?", "Xóa dữ liệu", MessageBoxButtons.OKCancel, MessageBoxIcon.Warning);
                    if (traloi == DialogResult.OK)
                    {
                        ClearZedGraph();

                    }

                }

            }
            n++;
            for (int i = 0; i < 200; i++)
            {
                fCalPosition[i] = update(positionRefC[i], fTime[i]);
                fCalVel[i] = getVelocity();
                fCalAcc[i] = getAcceleration();
                fCalPo[i] = getPosition();
            }
            if (getFinished()) { };
            reset();

            if (zedGraphVelocity.GraphPane.CurveList.Count <= 0)
                return;
            if (zedGraphAcc.GraphPane.CurveList.Count <= 0)
                return;
            if (zedGraphPosRef.GraphPane.CurveList.Count <= 1)
                return;
            LineItem duongline3 = zedGraphVelocity.GraphPane.CurveList[0] as LineItem;
            LineItem duongline4 = zedGraphAcc.GraphPane.CurveList[0] as LineItem;
            LineItem duongline5 = zedGraphPosRef.GraphPane.CurveList[0] as LineItem;
            // LineItem duongline6 = zedGraphPosRef.GraphPane.CurveList[1] as LineItem;
            LineItem duongline7 = zedGraphPosRef.GraphPane.CurveList[2] as LineItem;
            if (duongline5 == null)
                return;
            if (duongline3 == null)
                return;
            if (duongline4 == null)
                return;
            //if (duongline6 == null)
            //    return;
            if (duongline7 == null)
                return;

            IPointListEdit list5 = duongline5.Points as IPointListEdit;
            IPointListEdit list3 = duongline3.Points as IPointListEdit;
            IPointListEdit list4 = duongline4.Points as IPointListEdit;
            IPointListEdit list7 = duongline7.Points as IPointListEdit;

            //IPointListEdit list6 = duongline6.Points as IPointListEdit;

            if (list5 == null)
                return;
            if (list3 == null)
                return;
            if (list4 == null)
                return;
            if (list7 == null)
                return;

            //if (list6 == null)
            //    return;


            for (int i = 0; i < 200; i++)
            {
                list7.Add(fTime[i], fCalPo[i]);
                zedGraphPosRef.AxisChange();

                zedGraphPosRef.Invalidate();

            }
            for (int i = 0; i < 200; i++)
            {
                list5.Add(fTime[i], dPosRef);
                zedGraphPosRef.AxisChange();

                zedGraphPosRef.Invalidate();

            }


            for (int i = 0; i < 200; i++)
            {
                list3.Add(fTime[i], fCalVel[i]);
                zedGraphVelocity.AxisChange();

                zedGraphVelocity.Invalidate();

            }
            //zedGraphVelocity.GraphPane.CurveList.Clear();
            for (int i = 0; i < 200; i++)
            {
                list4.Add(fTime[i], fCalAcc[i]);
                zedGraphAcc.AxisChange();
                //   zedGraphAcc.GraphPane.CurveList.Clear();
                zedGraphAcc.Invalidate();

            }


        }



        private void btnSendSeting_Click(object sender, EventArgs e)
        {
            byte[] bProtocol = new byte[50];
            double dPosRef = Convert.ToDouble(txbPosMax.Text);
            double dVelMax = Convert.ToDouble(txbVelMax.Text);
            double dAccMax = Convert.ToDouble(txbAccMax.Text);

            ushort AccM = Convert.ToUInt16(dAccMax);
            ushort VelM = Convert.ToUInt16(dVelMax);
            ushort PosM = Convert.ToUInt16(dPosRef);
            bDATA[2] = (byte)((AccM & 0xFF00) >> 8);
            bDATA[3] = (byte)(AccM & 0xFF);
            bDATA[4] = (byte)((VelM & 0xFF00) >> 8);
            bDATA[5] = (byte)(VelM & 0xFF);
            bDATA[6] = (byte)((PosM & 0xFF00) >> 8);
            bDATA[7] = (byte)(PosM & 0xFF);


            Buffer.BlockCopy(bSTX, 0, bProtocol, 0, 1);
            Buffer.BlockCopy(bCSET, 0, bProtocol, 1, 4);
            Buffer.BlockCopy(bOPT, 0, bProtocol, 5, 3);
            Buffer.BlockCopy(bDATA, 0, bProtocol, 8, 8);
            Buffer.BlockCopy(bSYNC, 0, bProtocol, 16, 1);
            Buffer.BlockCopy(bETX, 0, bProtocol, 17, 1);
            bytetostring(bProtocol);
            serialPort1.Write(bProtocol, 0, 18);
            txtReadSerial.Text += "CSET CMD:\r\n";
        }

        private void btnRun_Click(object sender, EventArgs e)
        {

            byte[] bProtocol = new byte[50];

            Buffer.BlockCopy(bSTX, 0, bProtocol, 0, 1);
            Buffer.BlockCopy(bCRUN, 0, bProtocol, 1, 4);
            Buffer.BlockCopy(bOPT, 0, bProtocol, 5, 3);
            Buffer.BlockCopy(bDATA, 0, bProtocol, 8, 8);
            Buffer.BlockCopy(bSYNC, 0, bProtocol, 16, 1);
            Buffer.BlockCopy(bETX, 0, bProtocol, 17, 1);
            bytetostring(bProtocol);
            serialPort1.Write(bProtocol, 0, 18);
            txtReadSerial.Text += "CRUN CMD:\r\n";
        }

        private void button4_Click(object sender, EventArgs e)
        {
            byte[] bProtocol = new byte[50];

            Buffer.BlockCopy(bSTX, 0, bProtocol, 0, 1);
            Buffer.BlockCopy(bGRMS, 0, bProtocol, 1, 4);
            Buffer.BlockCopy(bOPT, 0, bProtocol, 5, 3);
            Buffer.BlockCopy(bDATA, 0, bProtocol, 8, 8);
            Buffer.BlockCopy(bSYNC, 0, bProtocol, 16, 1);
            Buffer.BlockCopy(bETX, 0, bProtocol, 17, 1);
            bytetostring(bProtocol);
            serialPort1.Write(bProtocol, 0, 18);
            txtReadSerial.Text += "GRMS CMD:\r\n";
        }



        private void btnReset_Click_1(object sender, EventArgs e)
        {
            zedGraphPID.GraphPane.CurveList.Clear(); // Xóa đường
            zedGraphPID.GraphPane.GraphObjList.Clear(); // Xóa đối tượng

            zedGraphPID.AxisChange();
            zedGraphPID.Invalidate();

            GraphPane mypanne = zedGraphPID.GraphPane;
            mypanne.Title.Text = "PID";
            mypanne.XAxis.Title.Text = "Time";
            mypanne.YAxis.Title.Text = "Position";
            RollingPointPairList list1 = new RollingPointPairList(60000);
            LineItem duongline1 = mypanne.AddCurve("Đường pid", list1, Color.Green, SymbolType.None);
            duongline1.Line.Width = 4;
            RollingPointPairList list2 = new RollingPointPairList(60000);
            LineItem duongline2 = mypanne.AddCurve("Set value", list2, Color.Blue, SymbolType.None);
            duongline2.Line.Width = 3;
            mypanne.XAxis.Scale.Min = 0;
            mypanne.XAxis.Scale.Max = 2;
            mypanne.XAxis.Scale.MinorStep = 0.1;
            mypanne.XAxis.Scale.MajorStep = 0.5;

            mypanne.YAxis.Scale.Min = 0;
            mypanne.YAxis.Scale.Max = 150;
            mypanne.YAxis.Scale.MinorStep = 1;
            mypanne.YAxis.Scale.MajorStep = 5;
            zedGraphPID.AxisChange(); //thay doi cac gia tri truc
        }

        private void textBox1_TextChanged(object sender, EventArgs e)
        {

        }

        private void btnSelect_Click(object sender, EventArgs e)
        {
            double Set = Convert.ToDouble(txbSetpoint.Text);

            double dSet = (Set * 360) / (4 * 210) + 1;
            byte[] bProtocol = new byte[50];

            byte[] bSet = new byte[2];

            float keycode001 = (float)(dSet);

            FloatToByteArray(keycode001, bSet, chuyedoi);
            bDATA[0] = bSet[0];
            bDATA[1] = bSet[1];
            bDATA[6] = 0x02;
            Buffer.BlockCopy(bSTX, 0, bProtocol, 0, 1);
            Buffer.BlockCopy(bSECT, 0, bProtocol, 1, 4);
            Buffer.BlockCopy(bOPT, 0, bProtocol, 5, 3);
            Buffer.BlockCopy(bDATA, 0, bProtocol, 8, 8);
            Buffer.BlockCopy(bSYNC, 0, bProtocol, 16, 1);
            Buffer.BlockCopy(bETX, 0, bProtocol, 17, 1);
            bytetostring(bProtocol);
            serialPort1.Write(bProtocol, 0, 18);
            txtReadSerial.Text += "SECT CMD:\r\n";
        }
        int aset = 0;
        byte[] clearPID = new byte[18];
        private void button1_Click(object sender, EventArgs e)
        {
            if (button1.Text == "CONVERT")
            {
                byte[] bProtocol = new byte[50];
                Buffer.BlockCopy(bSTX, 0, bProtocol, 0, 1);
                Buffer.BlockCopy(bSPED, 0, bProtocol, 1, 4);
                Buffer.BlockCopy(bOPT, 0, bProtocol, 5, 3);
                Buffer.BlockCopy(bDATA, 0, bProtocol, 8, 8);
                Buffer.BlockCopy(bSYNC, 0, bProtocol, 16, 1);
                Buffer.BlockCopy(bETX, 0, bProtocol, 17, 1);
                bytetostring(bProtocol);
                serialPort1.Write(bProtocol, 0, 18);
                txtReadSerial.Text += "SPED CMD:\r\n";

                button1.Text = "BACK";

            }
            else if (button1.Text == "BACK")
            {
                
                button1.Text = "CONVERT";

                byte[] bProtocol = new byte[50];
                Buffer.BlockCopy(bSTX, 0, bProtocol, 0, 1);
                Buffer.BlockCopy(bSTOP, 0, bProtocol, 1, 4);
                Buffer.BlockCopy(bOPT, 0, bProtocol, 5, 3);
                Buffer.BlockCopy(bDATA, 0, bProtocol, 8, 8);
                Buffer.BlockCopy(bSYNC, 0, bProtocol, 16, 1);
                Buffer.BlockCopy(bETX, 0, bProtocol, 17, 1);
                bytetostring(bProtocol);
                serialPort1.Write(bProtocol, 0, 18);
                txtReadSerial.Text += "STOP CMD:\r\n";
                ClearGraphSPEED();
            }

        }
        byte[] txbuff = new byte[64];

        private void timer1_Tick(object sender, EventArgs e)
        {
            st_count++;
            if (Status_Label.Text == "Connected" && st_count > 1)
            {
                Draw(Setpoint, Value, ep);

            }

        }


       

        int mode = 0;
        int run11 = 0;
        private void btnWebser_Click(object sender, EventArgs e)
        {
            byte[] bProtocol = new byte[50];
            Buffer.BlockCopy(bSTX, 0, bProtocol, 0, 1);
            Buffer.BlockCopy(bSERV, 0, bProtocol, 1, 4);
            Buffer.BlockCopy(bOPT, 0, bProtocol, 5, 3);
            Buffer.BlockCopy(bDATA, 0, bProtocol, 8, 8);
            Buffer.BlockCopy(bSYNC, 0, bProtocol, 16, 1);
            Buffer.BlockCopy(bETX, 0, bProtocol, 17, 1);
            bytetostring(bProtocol);
            serialPort1.Write(bProtocol, 0, 18);
            txtReadSerial.Text += "SERV CMD:\r\n";
            //  run11 = 1;

        }

        private void btnStopServer_Click(object sender, EventArgs e)
        {

            byte[] bProtocol = new byte[50];

            Buffer.BlockCopy(bSTX, 0, bProtocol, 0, 1);
            Buffer.BlockCopy(bSTOP, 0, bProtocol, 1, 4);
            Buffer.BlockCopy(bOPT, 0, bProtocol, 5, 3);
            Buffer.BlockCopy(bDATA, 0, bProtocol, 8, 8);
            Buffer.BlockCopy(bSYNC, 0, bProtocol, 16, 1);
            Buffer.BlockCopy(bETX, 0, bProtocol, 17, 1);
            bytetostring(bProtocol);
            serialPort1.Write(bProtocol, 0, 18);
           txtReadSerial.Text += "STOP CMD:\r\n";
           // run11 = 0;
        }

        private void btnOpenWeb_Click(object sender, EventArgs e)
        {
            if (btnOpenWeb.Text == "UPLOAD")
            {
                if (ccbBaurateJSON.Text != "4800" && ccbBaurateJSON.Text != "9600" && ccbBaurateJSON.Text != "19200" && ccbBaurateJSON.Text != "115200")
                {
                    MessageBox.Show("Chưa chọn Baudrate");
                }
                else
                {
                    serialPort2.PortName = ccbComJSON.Text;
                    serialPort2.Open();
                    btnOpenWeb.Text = "Close";

                    serialPort2.BaudRate = int.Parse(ccbBaurateJSON.Text);
                    ccbBaurateJSON.Enabled = false;
                    ccbComJSON.Enabled = false;

                    MessageBox.Show("Đã kết nối", "STATUS", MessageBoxButtons.OK, MessageBoxIcon.Exclamation);
                }
            }
            else if (btnOpenWeb.Text == "Close")
            {
                if (serialPort2.IsOpen == true)
                {

                    btnOpenWeb.Text = "UPLOAD";
                    serialPort2.DiscardInBuffer();
                    serialPort2.DiscardOutBuffer();

                    serialPort2.Close();
     
                    ccbBaurateJSON.Enabled = true;
                    ccbComJSON.Enabled = true;
   
                    MessageBox.Show("Đã ngắt kết nối", "STATUS", MessageBoxButtons.OK, MessageBoxIcon.Information);
                }
            }
        }

        private void serialPort2_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            int tong = 0;
           
                string rcv_data = serialPort2.ReadLine();
                string[] sub_data = rcv_data.Split(',');
                if (sub_data[0] == "S")
                {
                    // MessageBox.Show("ok nè ");
                    Setpoint = (sub_data[1]);
                    Value = sub_data[2];
                    udk = sub_data[3];
                    ep = sub_data[4];
                    double.TryParse(Setpoint, out intsetpoint);
                    double.TryParse(Value, out intcurrent);
                    //   double.TryParse(udk, out intudk);
                    double.TryParse(ep, out inte);//add
                    Speed_textBox.Text = (intcurrent / 100).ToString();
                    Setpoint_R_textBox.Text = intsetpoint.ToString();
                    // Udk_textBox.Text = (intudk / 100).ToString();
                    saiso.Text = (inte / 1000).ToString();
                }
                tong++;
                if (tong == 10)
                {
                    serialPort2.DiscardInBuffer();
                    tong = 0;
                }
            
        }

        int st_count;
        int TickStart;
        byte[] clear = new byte[64] ;
        private void btn_Setpara_Click(object sender, EventArgs e)
        {

            
            if (btn_Setpara.Text == "SET PARA")
            {
                txbuff[0] = 0x01;
                txbuff[1] = 0x01;
                byte[] setpoint = BitConverter.GetBytes(Int16.Parse(Setpoint_S_textBox.Text));
                txbuff[2] = setpoint[0];
                txbuff[3] = setpoint[1];

                btn_Setpara.Text = "Stop";
                btn_Update.Enabled = true;
                byte[] Kp = BitConverter.GetBytes(double.Parse(Kp_textBox.Text));
                Array.Copy(Kp, 0, txbuff, 4, 8);

                byte[] Ki = BitConverter.GetBytes(double.Parse(Ki_textBox.Text));
                Array.Copy(Ki, 0, txbuff, 12, 8);

                byte[] Kd = BitConverter.GetBytes(double.Parse(Kd_textBox.Text));
                Array.Copy(Kd, 0, txbuff, 20, 8);
                if (serialPort2.IsOpen) serialPort2.Write(txbuff, 0, 28);
            }
            else if (btn_Setpara.Text == "Stop")
            {
                btn_Update.Enabled = false;
                txbuff[0] = 0;
                txbuff[1] = 0;
                if (serialPort2.IsOpen) serialPort2.Write(txbuff, 0, 2);
                btn_Setpara.Text = "SET PARA";
                timer1.Stop();
            }

        }

        private void btn_Update_Click(object sender, EventArgs e)
        {
            st_count++;
            txbuff[0] = 1;
            txbuff[1] = 1;
            byte[] setpoint = BitConverter.GetBytes(Int16.Parse(Setpoint_S_textBox.Text));
            txbuff[2] = setpoint[0];
            txbuff[3] = setpoint[1];
   
            if (st_count == 1)
            { TickStart = Environment.TickCount; }

            byte[] Kp = BitConverter.GetBytes(double.Parse(Kp_textBox.Text));
            Array.Copy(Kp, 0, txbuff, 4, 8);

            byte[] Ki = BitConverter.GetBytes(double.Parse(Ki_textBox.Text));
            Array.Copy(Ki, 0, txbuff, 12, 8);

            byte[] Kd = BitConverter.GetBytes(double.Parse(Kd_textBox.Text));
            Array.Copy(Kd, 0, txbuff, 20, 8);
            if (serialPort2.IsOpen) serialPort2.Write(txbuff, 0, 28);

            timer1.Start();
        }
    }
}
