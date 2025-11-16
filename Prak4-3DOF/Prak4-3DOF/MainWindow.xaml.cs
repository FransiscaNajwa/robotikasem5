using System;
using System.IO.Ports;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Media;

namespace Prak4_3DOF
{
    public partial class MainWindow: Window
    {
        private SerialPort? serialPort1;
        private bool servo4IsOpen = false;

        // Struktur Data untuk Kalibrasi Servo (Angle, Pulse)
        private readonly double[] CAL_0 = { 0, 180, 660, 2260 };
        private readonly double[] CAL_1 = { 0, 90, 1000, 2000 };
        private readonly double[] CAL_2 = { -90, 90, 550, 2440 };
        private readonly double[] CAL_3 = { -90, 90, 540, 2380 };

        public MainWindow()
        {
            InitializeComponent();
            InitializeSerialPorts();

            // Setup aman (mode normal)
            WindowStyle = WindowStyle.SingleBorderWindow;
            WindowState = WindowState.Normal;

            // PENTING: Menambahkan handler KeyDown di sini karena ia tidak bergantung pada elemen XAML
            KeyDown += (s, e) =>
            {
                if (e.Key == Key.Escape)
                {
                    WindowStyle = WindowStyle.SingleBorderWindow;
                    WindowState = WindowState.Normal;
                }
            };

            // Logika startup utama akan dipicu oleh event Loaded di XAML
        }

        private void InitializeSerialPorts()
        {
            serialPort1 = new SerialPort();
        }

        // PERBAIKAN: Fungsi ini sekarang dipanggil dari ArmLength_WindowLoaded
        private void InitializeSliders()
        {
            // Atur Slider Min/Max ke nilai SUDUT (0-180, -90-90)
            // Ini akan aman karena dipanggil setelah semua elemen UI terjamin ada.
            sliderSudut0.Minimum = CAL_0[0]; sliderSudut0.Maximum = CAL_0[1]; sliderSudut0.Value = 90;
            sliderSudut1.Minimum = CAL_1[0]; sliderSudut1.Maximum = CAL_1[1]; sliderSudut1.Value = 45;
            sliderSudut2.Minimum = CAL_2[0]; sliderSudut2.Maximum = CAL_2[1]; sliderSudut2.Value = 0;
            sliderSudut3.Minimum = CAL_3[0]; sliderSudut3.Maximum = CAL_3[1]; sliderSudut3.Value = 0;
        }

        // PERBAIKAN VITAL: Menghapus semua logika startup dari Constructor dan memindahkannya ke sini.
        // Metode ini dihubungkan ke Loaded="ArmLength_WindowLoaded" di XAML Anda.
        private void ArmLength_WindowLoaded(object sender, RoutedEventArgs e)
        {
            try
            {
                // Inisialisasi yang membutuhkan akses UI (controls)
                RefreshPortList();
                InitializeSliders();
                CalculateForwardKinematics();

                // Opsional: Atur kembali ke Fullscreen setelah dimuat, jika diinginkan.
                // WindowStyle = WindowStyle.None; 
                // WindowState = WindowState.Maximized; 
            }
            catch (Exception ex)
            {
                // Menangkap error jika inisialisasi masih gagal
                MessageBox.Show($"FATAL STARTUP ERROR: {ex.Message}", "Application Failed", MessageBoxButton.OK, MessageBoxImage.Error);
            }
        }

        // ... (Semua fungsi lain di bawah ini tetap sama) ...

        private void RefreshPortList()
        {
            string[] ports = SerialPort.GetPortNames();
            cmbPorts.Items.Clear();
            foreach (string port in ports) cmbPorts.Items.Add(port);
            if (cmbPorts.Items.Count > 0) cmbPorts.SelectedIndex = 0;
        }

        // Helper: Interpolasi Linier (MAPPING)
        private double Map(double value, double inMin, double inMax, double outMin, double outMax)
        {
            return (value - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
        }

        private void SendServoCommand(SerialPort? port, int id, double angle)
        {
            if (port == null || !port.IsOpen) return;

            // Tentukan Kalibrasi
            double[] cal = id switch
            {
                0 => CAL_0,
                1 => CAL_1,
                2 => CAL_2,
                3 => CAL_3,
                _ => Array.Empty<double>()
            };
            if (cal.Length == 0) return;

            // Konversi Sudut ke Pulse Width
            int pulse = (int)Map(angle, cal[0], cal[1], cal[2], cal[3]);

            try
            {
                string command = $"#{id} P{pulse} S300 \r\n";
                port.Write(command);
            }
            catch (Exception ex)
            {
                MessageBox.Show($"Send Error on Servo {id}: {ex.Message}");
            }
        }

        // ========================= CONNECTION (Connect Only) =========================
        private void BtnConnect_Click(object sender, RoutedEventArgs e)
        {
            ToggleConnection(serialPort1, cmbPorts, btnConnect);
        }

        private void ToggleConnection(SerialPort? port, ComboBox comboBox, Button button)
        {
            if (port == null) return;

            try
            {
                if (port.IsOpen)
                {
                    port.Close();
                    button.Content = "CONNECT";
                    button.Background = Brushes.LightGreen;
                    lblConnectionStatus.Content = "Status: DISCONNECTED";
                    lblConnectionStatus.Foreground = Brushes.Red;
                }
                else
                {
                    if (comboBox.SelectedItem == null) { MessageBox.Show("Please select a COM port"); return; }
                    port.PortName = comboBox.SelectedItem.ToString()!;
                    port.BaudRate = 115200;
                    port.Open();

                    button.Content = "DISCONNECT";
                    button.Background = Brushes.LightCoral;
                    lblConnectionStatus.Content = $"Status: CONNECTED to {port.PortName}";
                    lblConnectionStatus.Foreground = Brushes.Green;
                }
            }
            catch (Exception ex) { MessageBox.Show($"Connection Error: {ex.Message}"); }
        }

        private void btnRefresh_Click(object sender, RoutedEventArgs e)
        {
            RefreshPortList();
        }

        // ========================= SLIDER HANDLERS =========================

        private void UpdateServoState(Slider slider, int servoId)
        {
            double angle = slider.Value;

            // 1. Update UI Sudut
            if (FindName($"txtSudut{servoId}") is TextBox txtAngle)
            {
                txtAngle.Text = angle.ToString("F1");
            }

            // 2. Update UI Pulse Width
            double[] cal = servoId switch { 0 => CAL_0, 1 => CAL_1, 2 => CAL_2, 3 => CAL_3, _ => Array.Empty<double>() };
            if (cal.Length > 0)
            {
                int pulse = (int)Map(angle, cal[0], cal[1], cal[2], cal[3]);
                if (FindName($"txtInterpolasi{servoId}") is TextBox txtInterp)
                {
                    txtInterp.Text = pulse.ToString();
                }
            }

            // 3. Kirim Perintah (Hanya jika terhubung)
            SendServoCommand(serialPort1, servoId, angle);

            // 4. Hitung FK (Hanya Servo 0, 2, 3 yang terlibat di FK)
            if (servoId == 0 || servoId == 2 || servoId == 3)
            {
                CalculateForwardKinematics();
            }
        }

        // Handlers individual yang dipanggil oleh XAML
        private void sliderSudut0_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e) { UpdateServoState((Slider)sender, 0); }
        private void sliderSudut1_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e) { UpdateServoState((Slider)sender, 1); }
        private void sliderSudut2_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e) { UpdateServoState((Slider)sender, 2); }
        private void sliderSudut3_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e) { UpdateServoState((Slider)sender, 3); }

        // MENGHILANGKAN ERROR KOMPILASI: Handler Min/Max yang Kosong
        private void txtMin0_TextChanged(object sender, TextChangedEventArgs e) { /* Kosongkan */ }
        private void txtMax0_TextChanged(object sender, TextChangedEventArgs e) { /* Kosongkan */ }
        private void txtMin1_TextChanged(object sender, TextChangedEventArgs e) { /* Kosongkan */ }
        private void txtMax1_TextChanged(object sender, TextChangedEventArgs e) { /* Kosongkan */ }
        private void txtMin2_TextChanged(object sender, TextChangedEventArgs e) { /* Kosongkan */ }
        private void txtMax2_TextChanged(object sender, TextChangedEventArgs e) { /* Kosongkan */ }
        private void txtMin3_TextChanged(object sender, TextChangedEventArgs e) { /* Kosongkan */ }
        private void txtMax3_TextChanged(object sender, TextChangedEventArgs e) { /* Kosongkan */ }

        // ========================= SERVO 4 (GRIPPER) =========================
        private void btnServo4Toggle_Click(object sender, RoutedEventArgs e)
        {
            if (serialPort1 != null && serialPort1.IsOpen)
            {
                try
                {
                    int pulseWidth = servo4IsOpen ? 670 : 1620; // 670 = TERTUTUP, 1620 = TERBUKA
                    string command = $"#4 P{pulseWidth} S300 \r\n";
                    serialPort1.Write(command);

                    servo4IsOpen = !servo4IsOpen;
                    btnServo4Toggle.Content = servo4IsOpen ? "TUTUP" : "BUKA";
                    lblServo4Status.Content = servo4IsOpen ? "Status: TERBUKA" : "Status: TERTUTUP";
                }
                catch (Exception ex) { MessageBox.Show($"Servo 4 Error: {ex.Message}"); }
            }
            else { MessageBox.Show("Please connect to COM port first!"); }
        }

        // ========================= FORWARD KINEMATICS (3 DOF) =========================
        private void ArmLength_TextChanged(object sender, TextChangedEventArgs e)
        {
            CalculateForwardKinematics();
        }

        // Fungsi ini sekarang menjadi handler Loaded yang aman
        private void CalculateForwardKinematics()
        {
            // Ambil Panjang Lengan
            if (!double.TryParse(txtA1.Text, out double a1) ||
                !double.TryParse(txtA2.Text, out double a2) ||
                !double.TryParse(txtA3.Text, out double a3)) return;

            // Ambil Sudut dari Slider (Sudah dalam Derajat)
            double theta1_deg = sliderSudut0.Value;
            double theta2_deg = sliderSudut2.Value;
            double theta3_deg = sliderSudut3.Value;

            // Konversi ke Radian
            double DegToRad(double deg) => deg * Math.PI / 180.0;

            double t1 = DegToRad(theta1_deg);
            double t2 = DegToRad(theta2_deg);
            double t3 = DegToRad(theta3_deg);

            // Perhitungan FK yang BENAR (akumulasi sudut)
            double Qx = a1 * Math.Cos(t1) + a2 * Math.Cos(t1 + t2) + a3 * Math.Cos(t1 + t2 + t3);
            double Qy = a1 * Math.Sin(t1) + a2 * Math.Sin(t1 + t2) + a3 * Math.Sin(t1 + t2 + t3);

            // Orientasi End Effector
            double orientasi_deg = theta1_deg + theta2_deg + theta3_deg;

            // Tampilkan Hasil
            txtQx.Text = Qx.ToString("F2");
            txtQy.Text = Qy.ToString("F2");
            txtOrientasi.Text = orientasi_deg.ToString("F2");
        }

        // ========================= INVERSE KINEMATICS (3 DOF) =========================

        // Handlers Calculate yang dipanggil oleh XAML
        private void btnCalculate1_Click(object sender, RoutedEventArgs e) { CalculateAndDisplayIK(m_satu, m_dua, m_tiga, false); }
        private void btnCalculate2_Click(object sender, RoutedEventArgs e) { CalculateAndDisplayIK(m_satu1, m_dua2, m_tiga2, true); }

        private void CalculateAndDisplayIK(TextBox txtTeta1, TextBox txtTeta2, TextBox txtTeta3, bool elbowDownSolution)
        {
            if (!double.TryParse(txtTargetQx.Text, out double Qx) ||
                !double.TryParse(txtTargetQy.Text, out double Qy) ||
                !double.TryParse(txtTargetOrientasi.Text, out double orientasi_deg) ||
                !double.TryParse(txtA1.Text, out double a1) ||
                !double.TryParse(txtA2.Text, out double a2) ||
                !double.TryParse(txtA3.Text, out double a3))
            {
                MessageBox.Show("Nilai input Kinematika Invers tidak valid.");
                return;
            }

            double phi_rad = orientasi_deg * Math.PI / 180.0;
            double Px = Qx - a3 * Math.Cos(phi_rad);
            double Py = Qy - a3 * Math.Sin(phi_rad);
            double D = (Px * Px + Py * Py - a1 * a1 - a2 * a2) / (2 * a1 * a2);

            if (D < -1.0 || D > 1.0)
            {
                MessageBox.Show("Target koordinat (Qx, Qy) tidak dapat dijangkau.");
                txtTeta1.Text = "N/A"; txtTeta2.Text = "N/A"; txtTeta3.Text = "N/A";
                return;
            }

            double theta2_rad = elbowDownSolution ? -Math.Acos(D) : Math.Acos(D);
            double theta1_rad = Math.Atan2(Py, Px) - Math.Atan2(a2 * Math.Sin(theta2_rad), a1 + a2 * Math.Cos(theta2_rad));
            double theta3_rad = phi_rad - theta1_rad - theta2_rad;

            double theta1_deg = theta1_rad * 180.0 / Math.PI;
            double theta2_deg = theta2_rad * 180.0 / Math.PI;
            double theta3_deg = theta3_rad * 180.0 / Math.PI;

            txtTeta1.Text = theta1_deg.ToString("F2");
            txtTeta2.Text = theta2_deg.ToString("F2");
            txtTeta3.Text = theta3_deg.ToString("F2");
        }
    }
}