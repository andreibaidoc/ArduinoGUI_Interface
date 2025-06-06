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
        }

        private void RefreshComPorts()
        {
            serial_options_combobox.ItemsSource = SerialPort.GetPortNames();
        }

    }
}