using System;
using System.IO.Ports;
using System.Net.Sockets;
using System.Text;
using System.Windows;
using System.Windows.Input;
using System.Windows.Threading;

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
            textBoxOutput.Text = "Output:\n";
            wifi_ip_textbox.Text = "192.168.4.1";
            textBoxSerialMonitor.Text = "Serial Monitor:\n";
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
                serialPort = new SerialPort(portName, 9600)
                {
                    DtrEnable = true, // Required for Arduino Uno R4 to reset properly
                    RtsEnable = true,
                    NewLine = "\n",   // Matches Arduino's println
                    Encoding = Encoding.ASCII
                };
                serialPort.DataReceived += SerialPort_DataReceived;
                serialPort.Open();
                serial_status_label.Text = $"Serial Connected on {comboBoxSerialPorts.Text}";
            }
            catch (Exception ex)
            {
                serial_status_label.Text = $"Error: {ex.Message}";
            }
        }

        private void SerialPort_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            try
            {
                string data = serialPort?.ReadLine() ?? "";

                Dispatcher.Invoke(() =>
                {
                    if (!string.IsNullOrWhiteSpace(data))
                    {
                        textBoxSerialMonitor.AppendText(data + "\n");
                        textBoxSerialMonitor.ScrollToEnd();
                    }
                });
            }
            catch (Exception ex)
            {
                Dispatcher.Invoke(() =>
                {
                    textBoxSerialMonitor.AppendText($"[Read Error] {ex.Message}\n");
                });
            }
        }

        private async void connect_wifi_button_Copy_Click(object sender, RoutedEventArgs e)
        {
            arduinoIP = wifi_ip_textbox.Text.Trim();
            if (string.IsNullOrEmpty(arduinoIP))
            {
                wifi_status_label.Text = "Enter IP address.";
                return;
            }

            wifi_status_label.Text = "⏳ Waiting for Arduino WiFi setup...";
            await Task.Delay(4000); // Let Arduino timeout on Serial and start WiFi

            try
            {
                using TcpClient client = new TcpClient(arduinoIP, arduinoPort);
                NetworkStream stream = client.GetStream();
                byte[] data = Encoding.ASCII.GetBytes("ping\n");
                stream.Write(data, 0, data.Length);

                byte[] buffer = new byte[256];
                int bytes = stream.Read(buffer, 0, buffer.Length);
                string response = Encoding.ASCII.GetString(buffer, 0, bytes);

                wifi_status_label.Text = "✅ WiFi Connected" + response;
            }
            catch (Exception ex)
            {
                wifi_status_label.Text = "❌ WiFi Error: " + ex.Message;
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

        private async void SendCommandToArduino(string command)
        {
            // Prefer Serial if connected
            if (serialPort?.IsOpen == true)
            {
                try
                {
                    serialPort.WriteLine(command);
                    textBoxOutput.AppendText($"[Serial] Sent: {command}\n");
                    textBoxOutput.ScrollToEnd();
                }
                catch (Exception ex)
                {
                    textBoxOutput.AppendText($"[Serial Error] {ex.Message}\n");
                    textBoxOutput.ScrollToEnd();
                }
                return;
            }

            // Use WiFi if Serial is not available
            if (!string.IsNullOrEmpty(arduinoIP))
            {
                try
                {
                    using TcpClient client = new TcpClient();
                    await client.ConnectAsync(arduinoIP, arduinoPort);

                    using NetworkStream stream = client.GetStream();
                    byte[] data = Encoding.ASCII.GetBytes(command + "\n");
                    await stream.WriteAsync(data, 0, data.Length);
                    await stream.FlushAsync();

                    // Wait up to 500ms for response
                    var buffer = new byte[256];
                    var timeout = DateTime.Now.AddMilliseconds(500);
                    string response = "";

                    while (!stream.DataAvailable && DateTime.Now < timeout)
                    {
                        await Task.Delay(10);
                    }

                    if (stream.DataAvailable)
                    {
                        int bytes = await stream.ReadAsync(buffer, 0, buffer.Length);
                        response = Encoding.ASCII.GetString(buffer, 0, bytes).Trim();
                    }
                    else
                    {
                        response = "[No response received]";
                    }

                    textBoxOutput.AppendText($"[WiFi] Sent: {command.Trim()} | Response: {response}\n");
                    textBoxOutput.ScrollToEnd();
                }
                catch (Exception ex)
                {
                    textBoxOutput.AppendText($"[WiFi Error] {ex.Message}\n");
                    textBoxOutput.ScrollToEnd();
                }
                return;
            }

            textBoxOutput.AppendText("❌ No communication channel available (Serial or WiFi).\n");
            textBoxOutput.ScrollToEnd();
        }


        private void sendCustomCommandTextbox_KeyDown(object sender, System.Windows.Input.KeyEventArgs e)
        {
            if (e.Key == Key.Enter)
            {
                string command = sendCustomCommandTextbox.Text.Trim();
                command = command + "\n"; // Ensure newline for Arduino compatibility

                if (!string.IsNullOrEmpty(command))
                {
                    SendCommandToArduino(command);
                    sendCustomCommandTextbox.Clear();
                    e.Handled = true;
                }
            }
        }

        private void refresh_com_ports_button_Click(object sender, RoutedEventArgs e)
        {
            RefreshComPorts();
        }
    }
}