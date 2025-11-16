using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.IO.Ports;
using System.Windows.Threading;

namespace P1_PRNO
{
    public partial class MainWindow : Window
    {
        // Deklarasi variabel anggota kelas
        private SerialPort serialPort;
        private DispatcherTimer refreshTimer;

        // Konstanta Pulse SSC Standar (Normal Mapping)
        private const double SSC_MIN_PULSE = 500;  // 0 derajat / -90 derajat
        private const double SSC_MAX_PULSE = 2500; // 180 derajat / +90 derajat

        public MainWindow()
        {
            InitializeComponent();
            InitializeSerialPort();
            RefreshPortList();

            // Timer untuk refresh daftar COM port tiap 2 detik
            refreshTimer = new DispatcherTimer();
            refreshTimer.Interval = TimeSpan.FromSeconds(2);
            refreshTimer.Tick += (s, e) => RefreshPortList();
            refreshTimer.Start();
        }

        private void InitializeSerialPort()
        {
            serialPort = new SerialPort();
        }

        private void RefreshPortList()
        {
            string currentSelection = cmbPorts.SelectedItem?.ToString();
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

                    serialPort.PortName = cmbPorts.SelectedItem.ToString();
                    serialPort.BaudRate = 115200;
                    serialPort.Parity = Parity.None;
                    serialPort.DataBits = 8;
                    serialPort.StopBits = StopBits.One;
                    serialPort.Handshake = Handshake.None;
                    serialPort.Open();
                    btnConnect.Content = "Disconnect";

                    // --- INISIALISASI GERAK OTOMATIS DINONAKTIFKAN ---
                }
            }
            catch (Exception ex)
            {
                MessageBox.Show($"Error: {ex.Message}");
            }
        }

        private void BtnClose_Click(object sender, RoutedEventArgs e)
        {
            if (serialPort.IsOpen)
            {
                serialPort.Close();
                btnConnect.Content = "Connect";
            }
        }

        private void BtnOrigin_Click(object sender, RoutedEventArgs e)
        {
            // FUNGSI INI DINONAKTIFKAN SEPENUHNYA UNTUK KEAMANAN
            MessageBox.Show("Fungsi 'Origin Position' telah dinonaktifkan. Silakan atur posisi secara manual.");
        }

        // fungsi linear interpolation (y = y1 + (x - x1) * (y2 - y1) / (x2 - x1))
        private double Interpolate(double x, double x1, double y1, double x2, double y2)
        {
            return y1 + (x - x1) * (y2 - y1) / (x2 - x1);
        }

        // Fungsi CLAMP untuk membatasi nilai di antara min dan max
        private double Clamp(double value, double min, double max)
        {
            if (value < min) return min;
            if (value > max) return max;
            return value;
        }

        private void SliderControl_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (sender is Slider slider && slider.Tag != null && int.TryParse(slider.Tag.ToString(), out int servoId))
            {
                double angle = e.NewValue;
                double pulse = 0;
                bool isKinematicsRelevant = false;

                if (servoId == 0 || servoId == 1) // Servo 0 & Servo 1 (0–180 derajat)
                {
                    // SSC Mapping NORMAL: 0 deg -> 500 us, 180 deg -> 2500 us
                    pulse = Interpolate(angle, 0, SSC_MIN_PULSE, 180, SSC_MAX_PULSE);

                    if (servoId == 0)
                    {
                        txtSudut0.Text = angle.ToString("F0");
                        txtInterp0.Text = pulse.ToString("F0");
                        isKinematicsRelevant = true;
                    }
                    else // servoId == 1 (Rata Air)
                    {
                        txtSudut1.Text = angle.ToString("F0");
                        txtInterp1.Text = pulse.ToString("F0");
                    }
                }
                else if (servoId == 2 || servoId == 3) // Servo 2 & Servo 3 (-90–+90 derajat)
                {
                    // SSC Mapping NORMAL: -90 deg -> 500 us, +90 deg -> 2500 us
                    pulse = Interpolate(angle, -90, SSC_MIN_PULSE, 90, SSC_MAX_PULSE);

                    if (servoId == 2)
                    {
                        txtSudut2.Text = angle.ToString("F0");
                        txtInterp2.Text = pulse.ToString("F0");
                        isKinematicsRelevant = true;
                    }
                    else // servoId == 3
                    {
                        txtSudut3.Text = angle.ToString("F0");
                        txtInterp3.Text = pulse.ToString("F0");
                    }
                }

                // Kirim perintah ke servo
                if (serialPort != null && serialPort.IsOpen)
                {
                    string message = $"#{servoId} P{(int)pulse} S500\r\n";
                    serialPort.Write(message);
                }

                // Hanya hitung FK jika Servo 0 atau 2 (yang relevan) bergerak
                if (isKinematicsRelevant)
                {
                    CalculateForwardKinematics();
                }
            }
        }

        private void CalculateForwardKinematics()
        {
            try
            {
                double a1 = double.Parse(m_a1.Text);
                double a2 = double.Parse(m_a2.Text);

                // Ambil sudut dari slider (dalam radian)
                double theta1 = sliderServo0.Value * Math.PI / 180.0;
                double theta2 = sliderServo2.Value * Math.PI / 180.0;

                // Rumus FK 2-DOF planar
                double Px = a1 * Math.Cos(theta1) + a2 * Math.Cos(theta1 + theta2);
                double Py = a1 * Math.Sin(theta1) + a2 * Math.Sin(theta1 + theta2);

                // FK hanya menulis output ke TextBox khusus FK
                m_fkPx.Text = Px.ToString("F3");
                m_fkPy.Text = Py.ToString("F3");
            }
            catch (Exception ex)
            {
                // Menangani error parsing input a1/a2 jika kosong atau tidak valid
            }
        }

        // === Tombol Calculate Solusi 1 (Elbow Up) ===
        private void cal_inv1_Click(object sender, RoutedEventArgs e)
        {
            try
            {
                double a1 = double.Parse(m_a1.Text);
                double a2 = double.Parse(m_a2.Text);

                // AMBIL INPUT Px dan Py BARU dari TextBox IK
                double Px = double.Parse(m_targetPx.Text);
                double Py = double.Parse(m_targetPy.Text);

                // Cek jangkauan
                double distanceSq = Px * Px + Py * Py;
                double sumSq = (a1 + a2) * (a1 + a2);
                double diffSq = (a1 - a2) * (a1 - a2);

                if (distanceSq > sumSq || distanceSq < diffSq)
                {
                    MessageBox.Show("Target di luar jangkauan robot.");
                    return;
                }

                // Rumus inverse kinematics (solusi elbow-up)
                double cos_teta2 = (distanceSq - a1 * a1 - a2 * a2) / (2 * a1 * a2);
                cos_teta2 = Math.Max(-1.0, Math.Min(1.0, cos_teta2));
                double teta2 = Math.Acos(cos_teta2);
                double teta1 = Math.Atan2(Py, Px) - Math.Atan2(a2 * Math.Sin(teta2), a1 + a2 * Math.Cos(teta2));

                // Konversi ke derajat
                double deg_teta1 = teta1 * 180.0 / Math.PI;
                double deg_teta2 = teta2 * 180.0 / Math.PI;

                // --- CLAMPING: Pastikan hasil sesuai range slider ---
                double clamped_teta1 = Clamp(deg_teta1, 0, 180);
                double clamped_teta2 = Clamp(deg_teta2, -90, 90);

                // Tampilkan hasil yang sudah di-clamp dan DIBULATKAN KE INTEGER (F0)
                m_satu.Text = clamped_teta1.ToString("F0");
                m_dua.Text = clamped_teta2.ToString("F0");
            }
            catch (Exception ex)
            {
                MessageBox.Show("Error IK Solusi 1: " + ex.Message);
            }
        }

        // === Tombol Run Solusi 1 (DINONAKTIFKAN) ===
        private void run_inv1_Click(object sender, RoutedEventArgs e)
        {
            // Tombol RUN IK dinonaktifkan untuk keamanan.
            MessageBox.Show("Tombol 'Run' IK dinonaktifkan. Gunakan slider servo untuk menggerakkan robot.");
        }

        // === Tombol Calculate Solusi 2 (Elbow Down) ===
        private void cal_inv2_Click(object sender, RoutedEventArgs e)
        {
            try
            {
                double a1 = double.Parse(m_a1.Text);
                double a2 = double.Parse(m_a2.Text);

                // AMBIL INPUT Px dan Py BARU dari TextBox IK
                double Px = double.Parse(m_targetPx.Text);
                double Py = double.Parse(m_targetPy.Text);

                // Cek jangkauan
                double distanceSq = Px * Px + Py * Py;
                double sumSq = (a1 + a2) * (a1 + a2);
                double diffSq = (a1 - a2) * (a1 - a2);

                if (distanceSq > sumSq || distanceSq < diffSq)
                {
                    MessageBox.Show("Target di luar jangkauan robot.");
                    return;
                }

                // Rumus inverse kinematics (solusi elbow-down)
                double cos_teta2 = (distanceSq - a1 * a1 - a2 * a2) / (2 * a1 * a2);
                cos_teta2 = Math.Max(-1.0, Math.Min(1.0, cos_teta2));
                double teta2 = -Math.Acos(cos_teta2); // Solusi negatif untuk elbow-down
                double teta1 = Math.Atan2(Py, Px) - Math.Atan2(a2 * Math.Sin(teta2), a1 + a2 * Math.Cos(teta2));

                // Konversi ke derajat
                double deg_teta1 = teta1 * 180.0 / Math.PI;
                double deg_teta2 = teta2 * 180.0 / Math.PI;

                // --- CLAMPING: Pastikan hasil sesuai range slider ---
                double clamped_teta1 = Clamp(deg_teta1, 0, 180);
                double clamped_teta2 = Clamp(deg_teta2, -90, 90);

                // Tampilkan hasil yang sudah di-clamp dan DIBULATKAN KE INTEGER (F0)
                m_satu1.Text = clamped_teta1.ToString("F0");
                m_dua2.Text = clamped_teta2.ToString("F0");
            }
            catch (Exception ex)
            {
                MessageBox.Show("Error IK Solusi 2: " + ex.Message);
            }
        }

        // === Tombol Run Solusi 2 (DINONAKTIFKAN) ===
        private void run_inv2_Click(object sender, RoutedEventArgs e)
        {
            // Tombol RUN IK dinonaktifkan untuk keamanan.
            MessageBox.Show("Tombol 'Run' IK dinonaktifkan. Gunakan slider servo untuk menggerakkan robot.");
        }

        private void BtnClaw_Open_Click(object sender, RoutedEventArgs e)
        {
            if (serialPort != null && serialPort.IsOpen)
            {
                string message = "#4 P770 S200\r\n";
                serialPort.Write(message);
            }
        }

        private void BtnClaw_Close_Click(object sender, RoutedEventArgs e)
        {
            if (serialPort != null && serialPort.IsOpen)
            {
                string message = "#4 P2200 S200\r\n";
                serialPort.Write(message);
            }
        }
    }
}