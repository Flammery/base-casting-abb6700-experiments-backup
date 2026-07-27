using ABB.Robotics.RobotStudio.Environment;
using System;
using System.IO;
using System.Threading.Tasks;
using System.Web.Script.Serialization;

namespace ABB6700.RobotStudioExport
{
    public static class AddInMainClass
    {
        private static readonly JavaScriptSerializer Json = new JavaScriptSerializer();
        private static readonly string BridgeDirectory = Path.Combine(
            System.Environment.GetFolderPath(System.Environment.SpecialFolder.LocalApplicationData),
            "ABB6700RobotStudioBridge");
        private static readonly string PendingPath = Path.Combine(BridgeDirectory, "pending.json");
        private static bool _processing;
        private static DateTime _notBefore;

        public static void AddInMain()
        {
            Directory.CreateDirectory(BridgeDirectory);
            try
            {
                string staleWorkDirectory = Path.Combine(BridgeDirectory, "work");
                if (Directory.Exists(staleWorkDirectory))
                    Directory.Delete(staleWorkDirectory, true);
            }
            catch
            {
                // A previous RobotStudio process may still be releasing a file;
                // this cache is retried on the next launch and is never an output.
            }
            AppendLog("AddInMain loaded in RobotStudio " + System.Diagnostics.Process.GetCurrentProcess().MainModule.FileVersionInfo.FileVersion);
            _notBefore = DateTime.UtcNow.AddSeconds(5);
            UIEnvironment.Idle += OnIdle;
        }

        private static void OnIdle(object sender, EventArgs args)
        {
            if (DateTime.UtcNow < _notBefore)
                return;

            try
            {
                string synchronizedStation = StationGenerator.TrySynchronizeActiveStation();
                if (!String.IsNullOrWhiteSpace(synchronizedStation))
                    AppendLog("RAPID synchronized for active station: " + synchronizedStation);
            }
            catch (Exception exception)
            {
                _notBefore = DateTime.UtcNow.AddSeconds(5);
                AppendLog("ACTIVE_STATION_ERROR " + exception);
                try
                {
                    File.WriteAllText(
                        Path.Combine(BridgeDirectory, "last_error.txt"),
                        DateTime.Now.ToString("s") + System.Environment.NewLine + exception);
                }
                catch
                {
                }
            }

            if (_processing || !File.Exists(PendingPath))
                return;

            _processing = true;
            string manifestPath = null;
            try
            {
                AppendLog("Claiming pending request.");
                PendingRequest request = Json.Deserialize<PendingRequest>(File.ReadAllText(PendingPath));
                File.Delete(PendingPath);
                if (request == null || String.IsNullOrWhiteSpace(request.manifest_path))
                    throw new InvalidDataException("pending.json has no manifest_path");
                manifestPath = Path.GetFullPath(request.manifest_path);
                AppendLog("Processing manifest: " + manifestPath);
                Task.Run(() => ProcessInBackground(manifestPath));
            }
            catch (Exception exception)
            {
                _processing = false;
                AppendLog("ERROR " + exception);
                try
                {
                    File.WriteAllText(
                        Path.Combine(BridgeDirectory, "last_error.txt"),
                        DateTime.Now.ToString("s") + System.Environment.NewLine + exception);
                }
                catch
                {
                }
            }
        }

        private static void ProcessInBackground(string manifestPath)
        {
            try
            {
                StationGenerator.ProcessManifest(manifestPath);
                AppendLog("Manifest completed: " + manifestPath);
            }
            catch (Exception exception)
            {
                AppendLog("ERROR " + exception);
                try
                {
                    File.WriteAllText(
                        Path.Combine(BridgeDirectory, "last_error.txt"),
                        DateTime.Now.ToString("s") + System.Environment.NewLine + exception);
                }
                catch
                {
                }
            }
            finally
            {
                _processing = false;
            }
        }

        private static void AppendLog(string message)
        {
            File.AppendAllText(
                Path.Combine(BridgeDirectory, "addin.log"),
                DateTime.Now.ToString("s") + " " + message + System.Environment.NewLine);
        }
    }
}
