using System;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Shapes;
using System.IO.Ports;
using System.Timers;        // gunakan System.Timers
using Timer = System.Timers.Timer;

namespace hurufproject
{
    public partial class MainWindow : Window
    {
        // ========= Timer dan SerialPort ==========
        private Timer motionTimer;
        private SerialPort port;

        // ========= Data Robot ==========        
        double m_a1, m_a2, m_a3;
        int m_n, m_t;

        // Joint space
        double m_teta1_0, m_teta2_0, m_teta3_0;
        double m_teta1_1, m_teta2_1, m_teta3_1;

        double[] teta1 = new double[5000];
        double[] teta2 = new double[5000];
        double[] teta3 = new double[5000];

        // Workspace
        double m_qx_0, m_qy_0, m_orientasi_0;
        double m_qx_1, m_qy_1, m_orientasi_1;

        double[] qx = new double[5000];
        double[] qy = new double[5000];
        double[] orientasi = new double[5000];

        int jenis = 0;     // 0=Time/Path, 1=Time/Track
        int space = 0;     // 0 joint, 1 workspace, 2 project
        int index = 0;


        // =====================================================================
        // ======================= CONSTRUCTOR =================================
        // =====================================================================
        public MainWindow()
        {
            InitializeComponent();

            port = new SerialPort();
            motionTimer = new Timer();
            motionTimer.Elapsed += MotionTick;

            LoadPorts();

            SliderGripper.ValueChanged += SliderGripper_ValueChanged;
            BtnConnect.Click += BtnConnect_Click;
            BtnDisconnect.Click += BtnDisconnect_Click;
            BtnOrigin2.Click += BtnOrigin2_Click;

            BtnCalcJoint.Click += BtnCalcJoint_Click;
            BtnRunJoint.Click += BtnRunJoint_Click;

            BtnCalcWork.Click += BtnCalcWork_Click;
            BtnRunWork.Click += BtnRunWork_Click;

            BtnCalcProject.Click += BtnCalcProject_Click;
            BtnStartPos.Click += BtnStartPos_Click;
            BtnRunProject.Click += BtnRunProject_Click;

            BtnExit.Click += (s, e) => Close();
        }

        // =====================================================================
        // ======================= DRAW ARM ====================================
        // =====================================================================
        void ArmDraw(double deg1, double deg2, double deg3)
        {
            m_armCanvas.Children.Clear();

            double t1 = deg1 * Math.PI / 180;
            double t2 = deg2 * Math.PI / 180;
            double t3 = deg3 * Math.PI / 180;

            double kx = m_a1 * Math.Cos(t1);
            double ky = m_a1 * Math.Sin(t1);

            double px = kx + m_a2 * Math.Cos(t1 + t2);
            double py = ky + m_a2 * Math.Sin(t1 + t2);

            double ex = px + m_a3 * Math.Cos(t1 + t2 + t3);
            double ey = py + m_a3 * Math.Sin(t1 + t2 + t3);

            DrawLine(0, 0, kx, ky, Brushes.Red, 4);
            DrawLine(kx, ky, px, py, Brushes.Green, 4);
            DrawLine(px, py, ex, ey, Brushes.Blue, 4);

            IndicatorQx.Text = ex.ToString("0.0");
            IndicatorQy.Text = ey.ToString("0.0");
            IndicatorT1.Text = deg1.ToString("0.0");
            IndicatorT2.Text = deg2.ToString("0.0");
            IndicatorT3.Text = deg3.ToString("0.0");
            IndicatorPhi.Text = (deg1 + deg2 + deg3).ToString("0.0");
        }

        void DrawLine(double x1, double y1, double x2, double y2, Brush color, double thick)
        {
            var line = new Line
            {
                Stroke = color,
                StrokeThickness = thick,
                X1 = x1 + 200,
                Y1 = 300 - y1,
                X2 = x2 + 200,
                Y2 = 300 - y2
            };
            m_armCanvas.Children.Add(line);
        }

        // =====================================================================
        // =========================== IK Solver ===============================
        // =====================================================================
        void IK(double qx, double qy, double phiDeg, out double t1, out double t2, out double t3)
        {
            double phi = phiDeg * Math.PI / 180;
            double wx = qx - m_a3 * Math.Cos(phi);
            double wy = qy - m_a3 * Math.Sin(phi);

            double D = (wx * wx + wy * wy - m_a1 * m_a1 - m_a2 * m_a2)
                       / (2 * m_a1 * m_a2);
            D = Math.Max(-1, Math.Min(1, D));

            double theta2 = Math.Acos(D);
            double k1 = m_a1 + m_a2 * Math.Cos(theta2);
            double k2 = m_a2 * Math.Sin(theta2);

            double theta1 = Math.Atan2(wy, wx) - Math.Atan2(k2, k1);
            double theta3 = phi - (theta1 + theta2);

            t1 = theta1 * 180 / Math.PI;
            t2 = -theta2 * 180 / Math.PI;
            t3 = theta3 * 180 / Math.PI;
        }

        // =====================================================================
        // =========================== TIMER ==================================
        // =====================================================================
        void MotionTick(object? sender, ElapsedEventArgs e)
        {
            Dispatcher.Invoke(() =>
            {
                if (space != 2 && index > m_n) { motionTimer.Stop(); return; }
                if (space == 2 && index > m_n * 4) { motionTimer.Stop(); return; }

                if (space == 0)
                {
                    ArmDraw(teta1[index], teta2[index], teta3[index]);
                }
                else
                {
                    IK(qx[index], qy[index], orientasi[index], out double a, out double b, out double c);
                    ArmDraw(a, b, c);
                }

                index++;
            });
        }


        // =====================================================================
        // ======================== READ INPUT DATA ============================
        // =====================================================================
        bool ReadInputs()
        {
            try
            {
                m_a1 = double.Parse(TxtA1.Text);
                m_a2 = double.Parse(TxtA2.Text);
                m_a3 = double.Parse(TxtA3.Text);
                m_n = int.Parse(TxtN.Text);
                m_t = int.Parse(TxtT.Text);

                m_teta1_0 = double.Parse(TxtTeta1_0.Text);
                m_teta2_0 = double.Parse(TxtTeta2_0.Text);
                m_teta3_0 = double.Parse(TxtTeta3_0.Text);

                m_teta1_1 = double.Parse(TxtTeta1_1.Text);
                m_teta2_1 = double.Parse(TxtTeta2_1.Text);
                m_teta3_1 = double.Parse(TxtTeta3_1.Text);

                m_qx_0 = double.Parse(TxtQx_0.Text);
                m_qy_0 = double.Parse(TxtQy_0.Text);
                m_orientasi_0 = double.Parse(TxtOrientasi_0.Text);

                m_qx_1 = double.Parse(TxtQx_1.Text);
                m_qy_1 = double.Parse(TxtQy_1.Text);
                m_orientasi_1 = double.Parse(TxtOrientasi_1.Text);

                jenis = RadioTimePath.IsChecked == true ? 0 : 1;

                return true;
            }
            catch
            {
                MessageBox.Show("Input tidak valid!");
                return false;
            }
        }


        // =====================================================================
        // ======================== JOINT SPACE ================================
        // =====================================================================
        private void BtnCalcJoint_Click(object sender, RoutedEventArgs e)
        {
            if (!ReadInputs()) return;

            ListJointSpace.Items.Clear();

            for (int i = 0; i <= m_n; i++)
            {
                teta1[i] = m_teta1_0 + (m_teta1_1 - m_teta1_0) * i / m_n;
                teta2[i] = m_teta2_0 + (m_teta2_1 - m_teta2_0) * i / m_n;
                teta3[i] = m_teta3_0 + (m_teta3_1 - m_teta3_0) * i / m_n;

                ListJointSpace.Items.Add(
                    $"{i,3} {teta1[i],6:0.0} {teta2[i],6:0.0} {teta3[i],6:0.0}"
                );
            }
        }

        private void BtnRunJoint_Click(object sender, RoutedEventArgs e)
        {
            index = 0;
            space = 0;
            motionTimer.Interval = (jenis == 0 ? m_t : m_t / m_n);
            motionTimer.Start();
        }


        // =====================================================================
        // ======================== WORK SPACE ================================
        // =====================================================================
        private void BtnCalcWork_Click(object sender, RoutedEventArgs e)
        {
            if (!ReadInputs()) return;

            ListWorkSpace.Items.Clear();

            for (int i = 0; i <= m_n; i++)
            {
                qx[i] = m_qx_0 + (m_qx_1 - m_qx_0) * i / m_n;
                qy[i] = m_qy_0 + (m_qy_1 - m_qy_0) * i / m_n;
                orientasi[i] = m_orientasi_0 + (m_orientasi_1 - m_orientasi_0) * i / m_n;

                ListWorkSpace.Items.Add(
                    $"{i,3} {qx[i],6:0.0} {qy[i],6:0.0} {orientasi[i],6:0.0}"
                );
            }
        }

        private void BtnRunWork_Click(object sender, RoutedEventArgs e)
        {
            index = 0;
            space = 1;
            motionTimer.Interval = (jenis == 0 ? m_t : m_t / m_n);
            motionTimer.Start();
        }


        // =====================================================================
        // ========================== PROJECT HURUF ============================
        // =====================================================================
        private void BtnCalcProject_Click(object sender, RoutedEventArgs e)
        {
            // Sama seperti BUTTON8 di MFC
            int[] tx = { 46, 60, 75, 90, 100 };
            int[] ty = { 240, 190, 220, 190, 240 };
            int[] to = { 90, 90, 90, 90, 90 };

            ListWorkSpace.Items.Clear();

            qx[0] = tx[0];
            qy[0] = ty[0];
            orientasi[0] = to[0];

            int target = 1;

            for (int i = 0; i < m_n * 4; i++)
            {
                if (i % m_n == 0 && i != 0) target++;

                qx[i + 1] = tx[target - 1] +
                    (tx[target] - tx[target - 1]) * ((i % m_n) + 1) / m_n;

                qy[i + 1] = ty[target - 1] +
                    (ty[target] - ty[target - 1]) * ((i % m_n) + 1) / m_n;

                orientasi[i + 1] = to[target - 1];

                ListWorkSpace.Items.Add(
                    $"{i,3} {qx[i],6:0.0} {qy[i],6:0.0} {orientasi[i],6:0.0}"
                );
            }

            MessageBox.Show("Trajectory Project Huruf selesai.");
        }

        private void BtnStartPos_Click(object sender, RoutedEventArgs e)
        {
            if (!port.IsOpen) return;
            port.WriteLine("#0 P1427 #2 P1985 #3 P1120 T3000");
        }

        private void BtnOrigin2_Click(object sender, RoutedEventArgs e)
        {
            if (!port.IsOpen) return;
            port.WriteLine("#0 P1525 #2 P1500 #3 P1500 T3000");
        }

        private void BtnRunProject_Click(object sender, RoutedEventArgs e)
        {
            index = 0;
            space = 2;
            motionTimer.Interval = (jenis == 0 ? m_t : m_t / (m_n * 4));
            motionTimer.Start();
        }


        // =====================================================================
        // =========================== COM PORT ================================
        // =====================================================================
        void LoadPorts()
        {
            foreach (var p in SerialPort.GetPortNames())
                CmbPorts.Items.Add(p);
        }

        private void BtnConnect_Click(object sender, RoutedEventArgs e)
        {
            if (CmbPorts.SelectedItem == null)
            {
                MessageBox.Show("Pilih COM port");
                return;
            }

            port.PortName = CmbPorts.SelectedItem.ToString();
            port.BaudRate = 115200;

            try
            {
                port.Open();
                MessageBox.Show("Connected!");
            }
            catch
            {
                MessageBox.Show("Gagal membuka port");
            }
        }

        private void BtnDisconnect_Click(object sender, RoutedEventArgs e)
        {
            if (port.IsOpen)
            {
                port.Close();
                MessageBox.Show("Disconnected");
            }
        }

        private void SliderGripper_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (!port.IsOpen) return;

            int pos = (int)Math.Round(
                2450 + (550 - 2450) * (e.NewValue / 180)
            );

            port.WriteLine($"#4 P{pos} S1000");
        }
    }
}
