using System;
using System.IO.Ports;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Threading;

namespace _2doflagi
{
    // Pastikan ini adalah partial class
    public partial class MainWindow : Window
    {
        // VARIABEL NON-NULLABLE (AUTOMATICALLY INITIALIZED BY XAML/InitializeComponent)
        // KITA TIDAK PERLU MENDEKLARASIKAN ULANG (MENGHILANGKAN AMBIGUITY ERROR)
        // cmbPorts, btnConnect, sliderServo0, m_a1, dll. akan otomatis ada.

        // VARIABEL CUSTOM (PERLU DEKLARASI)
        private SerialPort? serialPort = null;
        private DispatcherTimer? refreshTimer = null;

        public MainWindow()
        {
            // InitializeComponent() adalah yang menghubungkan XAML dengan C# 
            // dan menginisialisasi semua kontrol yang diberi x:Name.
            InitializeComponent();
            InitializeSerialPort();
        }

        private void InitializeSerialPort()
        {
            // Menggunakan Null-Forgiving Operator (!) pada inisialisasi serialPort
            // untuk menghilangkan warning bahwa ia mungkin null.
            serialPort = new SerialPort();
        }

        // TAMBAHAN: Method ini dipanggil saat Jendela dan semua elemen UI telah dimuat.
        private void Window_Loaded(object sender, RoutedEventArgs e)
        {
            RefreshPortList();

            // Inisialisasi Timer setelah cmbPorts dimuat
            refreshTimer = new DispatcherTimer();
            refreshTimer.Interval = TimeSpan.FromSeconds(2);
            refreshTimer.Tick += (s, ev) => RefreshPortList();
            refreshTimer.Start();
        }


        // --- Serial Port Management ---

        private void RefreshPortList()
        {
            // cmbPorts sekarang otomatis ada setelah InitializeComponent()
            if (cmbPorts == null) return;

            string? currentSelection = cmbPorts.SelectedItem?.ToString();
            cmbPorts.Items.Clear();
            string[] ports = SerialPort.GetPortNames();

            foreach (string port in ports)
            {
                cmbPorts.Items.Add(port);
            }

            if (!string.IsNullOrEmpty(currentSelection) && cmbPorts.Items.Contains(currentSelection))
            {
                cmbPorts.SelectedItem = currentSelection;
            }
            else if (cmbPorts.Items.Count > 0)
            {
                cmbPorts.SelectedIndex = 0;
            }
        }

        private void BtnConnect_Click(object sender, RoutedEventArgs e)
        {
            try
            {
                if (serialPort == null) return;

                if (serialPort.IsOpen)
                {
                    serialPort.Close();
                    btnConnect.Content = "Connect";
                }
                else
                {
                    if (cmbPorts.SelectedItem == null)
                    {
                        MessageBox.Show("Please select a COM port");
                        return;
                    }

                    // Menggunakan operator '!' (null-forgiving) karena kita sudah memastikan tidak null di atas.
                    serialPort.PortName = cmbPorts.SelectedItem.ToString()!;
                    serialPort.BaudRate = 115200;
                    serialPort.Parity = Parity.None;
                    serialPort.DataBits = 8;
                    serialPort.StopBits = StopBits.One;
                    serialPort.Handshake = Handshake.None;
                    serialPort.Open();
                    btnConnect.Content = "Disconnect";

                    // Kirim posisi awal servo 
                    SendServoCommand(0, 1570, 100);
                    SendServoCommand(2, 1483, 100);
                    SendServoCommand(3, 1550, 100);
                }
            }
            catch (Exception ex)
            {
                MessageBox.Show($"Error during connection: {ex.Message}");
            }
        }

        private void BtnClose_Click(object sender, RoutedEventArgs e)
        {
            if (serialPort != null && serialPort.IsOpen)
            {
                serialPort.Close();
                btnConnect.Content = "Connect"; // btnConnect sudah tersedia dari XAML
                MessageBox.Show("Port serial tertutup.");
            }
        }

        private void SendServoCommand(int servoId, int pulse, int speed)
        {
            if (serialPort != null && serialPort.IsOpen)
            {
                string message = $"#{servoId} P{pulse} S{speed}\r\n";
                try
                {
                    serialPort.Write(message);
                }
                catch
                {
                    // Catch tanpa variabel 'ex' untuk menghindari warning "declared but never used"
                }
            }
        }

        // --- Servo Control, Interpolation, and Origin ---

        private void BtnOrigin_Click(object sender, RoutedEventArgs e)
        {
            if (serialPort != null && serialPort.IsOpen)
            {
                SendServoCommand(0, 1570, 500);
                SendServoCommand(2, 1483, 500);
                SendServoCommand(3, 1550, 500);
                SendServoCommand(4, 1583, 500);

                // Kontrol di XAML diakses langsung
                sliderServo0.Value = 90;
                sliderServo2.Value = 0;
                sliderServo3.Value = 0;
                sliderJepit.Value = 140;
            }
            else
            {
                MessageBox.Show("Serial port belum terbuka. Silakan Connect dulu.");
            }
        }

        private void SliderControl_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            // Menggunakan C# 8 'is not' pattern match
            if (sender is not Slider slider) return;

            // Logika untuk Servo 0, 2, 3
            if (slider.Tag != null && int.TryParse(slider.Tag.ToString(), out int servoId))
            {
                double angle = e.NewValue;
                double pulse = 0;

                if (servoId == 0)
                {
                    pulse = Interpolate(angle, 0, 2430, 180, 620);
                    txtSudut0.Text = angle.ToString("F0");
                    txtInterp0.Text = pulse.ToString("F0");
                }
                else if (servoId == 2)
                {
                    pulse = Interpolate(angle, -90, 2450, 90, 660);
                    txtSudut2.Text = angle.ToString("F0");
                    txtInterp2.Text = pulse.ToString("F0");
                }
                else if (servoId == 3)
                {
                    pulse = Interpolate(angle, -90, 2430, 90, 650);
                    txtSudut3.Text = angle.ToString("F0");
                    txtInterp3.Text = pulse.ToString("F0");
                }

                SendServoCommand(servoId, (int)pulse, 100);

                if (servoId == 0 || servoId == 2)
                {
                    CalculateForwardKinematics();
                }
            }
            // Logika untuk Jepit (Claw) 
            else if (slider.Name == "sliderJepit")
            {
                double mm = e.NewValue;
                double pulse = Interpolate(mm, 0, 2122, 200, 1044);
                txtJepitMM.Text = mm.ToString("F0");
                txtInterpJepit.Text = pulse.ToString("F0");

                SendServoCommand(4, (int)pulse, 100);
            }
        }

        private double Interpolate(double x, double x1, double y1, double x2, double y2)
        {
            return y1 + (x - x1) * (y2 - y1) / (x2 - x1);
        }

        // --- Kinematics Functions ---

        private void CalculateForwardKinematics()
        {
            try
            {
                // Menggunakan operator '!' karena setelah InitializeComponent, 
                // kontrol UI yang diberi x:Name diasumsikan non-null.
                if (!double.TryParse(m_a1.Text, out double a1) || !double.TryParse(m_a2.Text, out double a2))
                {
                    m_px.Text = "N/A"; m_py.Text = "N/A";
                    return;
                }

                double theta1_deg = sliderServo0.Value;
                double theta2_deg = sliderServo2.Value;

                double theta1 = theta1_deg * Math.PI / 180.0;
                double theta2 = theta2_deg * Math.PI / 180.0;

                double Px = a1 * Math.Cos(theta1) + a2 * Math.Cos(theta1 + theta2);
                double Py = a1 * Math.Sin(theta1) + a2 * Math.Sin(theta1 + theta2);

                m_px.Text = Px.ToString("F3");
                m_py.Text = Py.ToString("F3");
            }
            catch
            {
                // Catch tanpa variabel 'ex'
            }
        }

        private void cal_inv1_Click(object sender, RoutedEventArgs e)
        {
            CalculateInverseKinematics(true);
        }

        private void cal_inv2_Click(object sender, RoutedEventArgs e)
        {
            CalculateInverseKinematics(false);
        }

        private void CalculateInverseKinematics(bool elbowUp)
        {
            try
            {
                double a1 = double.Parse(m_a1.Text);
                double a2 = double.Parse(m_a2.Text);
                double Px = double.Parse(m_px.Text);
                double Py = double.Parse(m_py.Text);

                double D = (Px * Px + Py * Py - a1 * a1 - a2 * a2) / (2 * a1 * a2);

                if (D > 1 || D < -1)
                {
                    MessageBox.Show("Target di luar jangkauan robot.");
                    return;
                }

                double teta2 = elbowUp ? Math.Acos(D) : -Math.Acos(D);

                double teta1 = Math.Atan2(Py, Px) - Math.Atan2(a2 * Math.Sin(teta2), a1 + a2 * Math.Cos(teta2));

                double deg_teta1 = teta1 * 180.0 / Math.PI;
                double deg_teta2 = teta2 * 180.0 / Math.PI;

                if (elbowUp)
                {
                    m_satu.Text = deg_teta1.ToString("F2");
                    m_dua.Text = deg_teta2.ToString("F2");
                }
                else
                {
                    m_satu1.Text = deg_teta1.ToString("F2");
                    m_dua2.Text = deg_teta2.ToString("F2");
                }
            }
            catch (Exception ex)
            {
                MessageBox.Show($"Error IK: Pastikan semua parameter diisi dengan angka. Pesan: {ex.Message}");
            }
        }

        private void run_inv1_Click(object sender, RoutedEventArgs e)
        {
            RunInverseKinematics(m_satu.Text, m_dua.Text);
        }

        private void run_inv2_Click(object sender, RoutedEventArgs e)
        {
            RunInverseKinematics(m_satu1.Text, m_dua2.Text);
        }

        private void RunInverseKinematics(string teta1Text, string teta2Text)
        {
            try
            {
                if (serialPort == null || !serialPort.IsOpen)
                {
                    MessageBox.Show("Serial port belum terbuka.");
                    return;
                }

                double deg_teta1 = double.Parse(teta1Text);
                double deg_teta2 = double.Parse(teta2Text);

                double pulse0 = Interpolate(deg_teta1, 0, 2430, 180, 620);
                double pulse2 = Interpolate(deg_teta2, -90, 2450, 90, 660);

                SendServoCommand(0, (int)pulse0, 500);
                SendServoCommand(2, (int)pulse2, 500);

                sliderServo0.Value = deg_teta1;
                sliderServo2.Value = deg_teta2;
            }
            catch (Exception ex)
            {
                MessageBox.Show($"Error Run IK: {ex.Message}");
            }
        }

        // --- Claw/Jepit Control ---

        private void BtnClaw_Open_Click(object sender, RoutedEventArgs e)
        {
            SendServoCommand(4, 1044, 200);
        }

        private void BtnClaw_Close_Click(object sender, RoutedEventArgs e)
        {
            SendServoCommand(4, 2122, 200);
        }

        // --- Window Control ---

        private void BtnExit_Click(object sender, RoutedEventArgs e)
        {
            if (serialPort != null && serialPort.IsOpen)
            {
                serialPort.Close();
            }
            refreshTimer?.Stop();

            Application.Current.Shutdown();
        }

        private void BtnCancel_Click(object sender, RoutedEventArgs e)
        {
            MessageBox.Show("Operasi dibatalkan.");
        }
    }
}