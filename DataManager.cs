using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Windows;

public class DataSample
{
    public ulong Timestamp { get; set; }
    public float Temperature { get; set; }
    public float Humidity { get; set; }
    public float Pressure { get; set; }
    public float AccelX { get; set; }
    public float AccelY { get; set; }
    public float AccelZ { get; set; }
}

public class DataManager
{
    public List<DataSample> Samples { get; private set; } = new();

    public void ParseCsv(string csv)
    {
        Samples.Clear();
        var lines = csv.Split('\n');

        foreach (var line in lines.Skip(1)) // Skip header
        {
            var parts = line.Split(',');
            if (parts.Length != 7) continue;

            if (ulong.TryParse(parts[0], out var time) &&
                float.TryParse(parts[1], out var temp) &&
                float.TryParse(parts[2], out var hum) &&
                float.TryParse(parts[3], out var press) &&
                float.TryParse(parts[4], out var ax) &&
                float.TryParse(parts[5], out var ay) &&
                float.TryParse(parts[6], out var az))
            {
                Samples.Add(new DataSample
                {
                    Timestamp = time,
                    Temperature = temp,
                    Humidity = hum,
                    Pressure = press,
                    AccelX = ax,
                    AccelY = ay,
                    AccelZ = az
                });
            }
        }
    }

    public void ExportToCsvWithDialog()
    {
        var dialog = new Microsoft.Win32.SaveFileDialog
        {
            FileName = "SensorData",
            DefaultExt = ".csv",
            Filter = "CSV files (*.csv)|*.csv"
        };

        if (dialog.ShowDialog() == true)
        {
            var sb = new StringBuilder();
            sb.AppendLine("Timestamp,Temperature,Humidity,Pressure,AccelX,AccelY,AccelZ");

            foreach (var sample in Samples)
            {
                sb.AppendLine($"{sample.Timestamp},{sample.Temperature},{sample.Humidity},{sample.Pressure},{sample.AccelX},{sample.AccelY},{sample.AccelZ}");
            }

            File.WriteAllText(dialog.FileName, sb.ToString());
            MessageBox.Show("Data exported successfully.");
        }
    }

    public int Count => Samples.Count;
}
