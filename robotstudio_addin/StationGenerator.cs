using ABB.Robotics.Controllers;
using ABB.Robotics.Controllers.RapidDomain;
using ABB.Robotics.RobotStudio;
using ABB.Robotics.RobotStudio.Stations;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Web.Script.Serialization;

namespace ABB6700.RobotStudioExport
{
    public static class StationGenerator
    {
        private const string ManifestSchema = "base_casting_abb6700.robotstudio_jobs";
        private const string SidecarSchema = "base_casting_abb6700.robotstudio_station_job";
        private const string BridgeModuleName = "RSBRIDGE";
        private const string ManagedPathModulePrefix = "VALIDATE_R";
        private static readonly JavaScriptSerializer Json = new JavaScriptSerializer();
        private static string _lastSynchronizedKey = "";
        private static string _observedStationPath = "";
        private static DateTime _observedStationAt;

        public static void ProcessManifest(string manifestPath)
        {
            JobManifest manifest = Json.Deserialize<JobManifest>(File.ReadAllText(manifestPath));
            if (manifest == null || manifest.schema != ManifestSchema || manifest.version != 1)
                throw new InvalidDataException("Unsupported RobotStudio job manifest: " + manifestPath);
            if (manifest.jobs == null || manifest.jobs.Count == 0)
                throw new InvalidDataException("RobotStudio manifest has no jobs.");

            foreach (StationJob job in manifest.jobs)
            {
                if (!File.Exists(job.output_station))
                    throw new FileNotFoundException("Generated station is missing.", job.output_station);
                if (!File.Exists(job.station_job))
                    throw new FileNotFoundException("Station RAPID sidecar is missing.", job.station_job);
            }

            var status = new ExportStatus
            {
                state = "completed",
                manifest_path = manifestPath,
                completed = manifest.jobs.Count,
                total = manifest.jobs.Count,
                current_region = "",
                message = "Station files prepared. Opening any generated station auto-loads its RAPID modules; manual posture review remains required.",
                jobs = manifest.jobs.Select(job => new JobStatus
                {
                    region_label = job.region_label,
                    state = "saved",
                    output_station = job.output_station,
                    message = "Scene installation written; RAPID sidecar ready for automatic load on open."
                }).ToList()
            };
            WriteStatus(Path.Combine(manifest.result_dir, "robotstudio_status.json"), status);
        }

        public static string TrySynchronizeActiveStation()
        {
            Station station = Project.ActiveProject as Station;
            if (station == null || station.FileInfo == null)
                return null;
            string stationPath = station.FileInfo.FullName;
            if (!System.String.Equals(stationPath, _observedStationPath, StringComparison.OrdinalIgnoreCase))
            {
                _observedStationPath = stationPath;
                _observedStationAt = DateTime.UtcNow;
                return null;
            }
            if (DateTime.UtcNow - _observedStationAt < TimeSpan.FromSeconds(10))
                return null;
            string sidecarPath = Path.Combine(
                Path.GetDirectoryName(stationPath),
                Path.GetFileNameWithoutExtension(stationPath) + ".robotstudio_job.json");
            if (!File.Exists(sidecarPath))
                return null;

            string key = sidecarPath + "|" + File.GetLastWriteTimeUtc(sidecarPath).Ticks;
            if (System.String.Equals(key, _lastSynchronizedKey, StringComparison.OrdinalIgnoreCase))
                return null;

            StationSidecar sidecar = Json.Deserialize<StationSidecar>(File.ReadAllText(sidecarPath));
            if (sidecar == null || sidecar.schema != SidecarSchema || sidecar.version != 1)
                throw new InvalidDataException("Unsupported station RAPID sidecar: " + sidecarPath);
            if (!System.String.Equals(Path.GetFullPath(sidecar.output_station), stationPath, StringComparison.OrdinalIgnoreCase))
                throw new InvalidDataException("Station sidecar does not match the active station: " + sidecarPath);
            LoadRapidModules(station, sidecar);
            _lastSynchronizedKey = key;
            return stationPath;
        }

        private static void LoadRapidModules(Station station, StationSidecar sidecar)
        {
            RsTask rsTask = station.Irc5Controllers
                .SelectMany(controller => controller.Tasks.Cast<RsTask>())
                .FirstOrDefault(task => System.String.Equals(task.Name, sidecar.controller_task, StringComparison.Ordinal));
            if (rsTask == null)
                throw new InvalidOperationException("RobotStudio task not found: " + sidecar.controller_task);
            RsIrc5Controller rsController = rsTask.Parent as RsIrc5Controller;
            if (rsController == null || System.String.IsNullOrWhiteSpace(rsController.SystemId))
                throw new InvalidOperationException("Active station virtual controller is unavailable.");

            using (Controller controller = new Controller(new Guid(rsController.SystemId)))
            {
                controller.Logon(UserInfo.DefaultUser);
                try
                {
                    ABB.Robotics.Controllers.RapidDomain.Task rapidTask = controller.Rapid.GetTask(sidecar.controller_task);
                    rapidTask.Stop();
                    using (Mastership.Request(controller.Rapid))
                    {
                        string bridgeModulePath = Path.Combine(
                            System.Environment.GetFolderPath(System.Environment.SpecialFolder.LocalApplicationData),
                            "ABB6700RobotStudioBridge",
                            "RSBRIDGE.mod");
                        File.WriteAllText(
                            bridgeModulePath,
                            "MODULE " + BridgeModuleName + "\r\n    PROC bridge_hold()\r\n    ENDPROC\r\nENDMODULE\r\n");
                        Module existingBridge = FindModule(rapidTask, BridgeModuleName);
                        if (existingBridge == null
                            && !rapidTask.LoadModuleFromFile(bridgeModulePath, RapidLoadMode.Replace))
                        {
                            throw new InvalidOperationException("RobotStudio rejected the temporary program-pointer bridge module.");
                        }
                        if (existingBridge == null)
                            rapidTask.SetProgramPointer(BridgeModuleName, "bridge_hold");

                        // Only remove modules owned by this add-in.  Calling GetRoutine on a
                        // partially loaded module can itself fail in RobotWare 6 and prevent
                        // recovery on every later station open.
                        foreach (Module module in rapidTask.GetModules().ToArray())
                        {
                            if (IsManagedPathModule(module.Name))
                                module.Delete();
                        }
                        // A failed earlier switch can legitimately leave the program pointer in
                        // RSBRIDGE. RobotWare refuses any pointer operation while a partially
                        // loaded path still has syntax errors, so remove that managed path first.
                        if (existingBridge != null)
                            rapidTask.SetProgramPointer(BridgeModuleName, "bridge_hold");
                        if (!rapidTask.LoadModuleFromFile(sidecar.calib_module, RapidLoadMode.Replace))
                            throw new InvalidOperationException("RobotStudio rejected CalibData: " + sidecar.calib_module);
                        if (!rapidTask.LoadModuleFromFile(sidecar.path_module, RapidLoadMode.Replace))
                        {
                            DeleteModuleIfPresent(rapidTask, sidecar.path_module_name);
                            throw new InvalidOperationException("RobotStudio rejected path module: " + sidecar.path_module);
                        }
                        rapidTask.SetProgramPointer(sidecar.path_module_name, "main");
                        DeleteModuleIfPresent(rapidTask, BridgeModuleName);
                    }
                    rsTask.EntryPoint = "main";
                }
                finally
                {
                    controller.Logoff();
                }
            }
        }

        private static bool IsManagedPathModule(string moduleName)
        {
            return !System.String.IsNullOrWhiteSpace(moduleName)
                && moduleName.StartsWith(ManagedPathModulePrefix, StringComparison.OrdinalIgnoreCase);
        }

        private static void DeleteModuleIfPresent(
            ABB.Robotics.Controllers.RapidDomain.Task rapidTask,
            string moduleName)
        {
            Module module = FindModule(rapidTask, moduleName);
            if (module != null)
                module.Delete();
        }

        private static Module FindModule(
            ABB.Robotics.Controllers.RapidDomain.Task rapidTask,
            string moduleName)
        {
            return rapidTask.GetModules()
                .FirstOrDefault(item => System.String.Equals(item.Name, moduleName, StringComparison.OrdinalIgnoreCase));
        }

        private static void WriteStatus(string path, ExportStatus status)
        {
            File.WriteAllText(path, Json.Serialize(status));
        }
    }
}
