using System;
using System.IO.Ports;
using System.Net.Sockets;
using System.Text;
using System.Windows;
using System.Windows.Controls;

namespace ArduinoGUI_Interface
{
    public partial class MainWindow : Window
    {
        // variables
        private SerialPort? serialPort;
        private string? arduinoIP;
        private readonly int arduinoPort = 8888;

        public MainWindow()
        {
            InitializeComponent();
            Loaded += MainWindow_Loaded;
        }
        private void MainWindow_Loaded(object sender, RoutedEventArgs e)
        {
            RefreshComPorts();

            wifi_status_label.Text = "";
            serial_status_label.Text = "";
        }

        private void RefreshComPorts()
        {
            comboBoxSerialPorts.ItemsSource = SerialPort.GetPortNames();
        }

        private void connect_usb_button_Click(object sender, RoutedEventArgs e)
        {
            string? portName = comboBoxSerialPorts.SelectedItem?.ToString();
            if (string.IsNullOrEmpty(portName)) return;

            try
            {
                serialPort = new SerialPort(portName, 9600);
                serialPort.DataReceived += SerialPort_DataReceived;
                serialPort.Open();
                serial_status_label.Text = "Serial Connected!";
            }
            catch (Exception ex)
            {
                serial_status_label.Text = $"Error: {ex.Message}";
            }
        }

        private void SerialPort_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            string data = serialPort?.ReadLine() ?? string.Empty;
            Dispatcher.Invoke(() =>
            {
                // textBoxOutput.AppendText("Serial: " + data + "\n");
            });
        }

        private void connect_wifi_button_Copy_Click(object sender, RoutedEventArgs e)
        {
            arduinoIP = wifi_ip_textbox.Text.Trim();
            if (string.IsNullOrEmpty(arduinoIP))
            {
                wifi_status_label.Text = "Enter IP address.";
                return;
            }

            try
            {
                using TcpClient client = new TcpClient(arduinoIP, arduinoPort);
                NetworkStream stream = client.GetStream();
                byte[] data = Encoding.ASCII.GetBytes("PING\n");
                stream.Write(data, 0, data.Length);

                byte[] buffer = new byte[256];
                int bytes = stream.Read(buffer, 0, buffer.Length);
                string response = Encoding.ASCII.GetString(buffer, 0, bytes);

                wifi_status_label.Text = "WiFi Connected: " + response;
            }
            catch (Exception ex)
            {
                wifi_status_label.Text = "WiFi Error: " + ex.Message;
            }
        }

        private void turnOnLedButton_Click(object sender, RoutedEventArgs e)
        {
            SendCommandToArduino("led on");
        }

        private void turnOffLedButton_Click(object sender, RoutedEventArgs e)
        {
            SendCommandToArduino("led off");
        }

        private void SendCommandToArduino(string command)
        {
            // Prefer Serial if connected
            if (serialPort?.IsOpen == true)
            {
                try
                {
                    serialPort.WriteLine(command);
                    textBoxOutput.AppendText($"[Serial] Sent: {command}\n");
                }
                catch (Exception ex)
                {
                    textBoxOutput.AppendText($"[Serial Error] {ex.Message}\n");
                }
                return;
            }

            // Use WiFi if Serial is not available
            if (!string.IsNullOrEmpty(arduinoIP))
            {
                try
                {
                    using TcpClient client = new TcpClient(arduinoIP, arduinoPort);
                    NetworkStream stream = client.GetStream();
                    byte[] data = Encoding.ASCII.GetBytes(command + "\n");
                    stream.Write(data, 0, data.Length);

                    byte[] buffer = new byte[256];
                    int bytes = stream.Read(buffer, 0, buffer.Length);
                    string response = Encoding.ASCII.GetString(buffer, 0, bytes);

                    textBoxOutput.AppendText($"[WiFi] Sent: {command} | Response: {response}\n");
                }
                catch (Exception ex)
                {
                    textBoxOutput.AppendText($"[WiFi Error] {ex.Message}\n");
                }
                return;
            }

            textBoxOutput.AppendText("❌ No communication channel available (Serial or WiFi).\n");
        }
    }
}