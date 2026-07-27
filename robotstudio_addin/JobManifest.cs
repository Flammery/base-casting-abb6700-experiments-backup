using System.Collections.Generic;

namespace ABB6700.RobotStudioExport
{
    public sealed class PendingRequest
    {
        public string manifest_path { get; set; }
    }

    public sealed class JobManifest
    {
        public string schema { get; set; }
        public int version { get; set; }
        public string result_dir { get; set; }
        public string template_station { get; set; }
        public string controller_task { get; set; }
        public string calib_module_name { get; set; }
        public string workpiece_component_name { get; set; }
        public List<StationJob> jobs { get; set; }
    }

    public sealed class StationJob
    {
        public string region_label { get; set; }
        public ModelInstallation model_installation { get; set; }
        public bool rapid_coordinates_are_independent { get; set; }
        public string source_rapid { get; set; }
        public string calib_module { get; set; }
        public string path_module { get; set; }
        public string path_module_name { get; set; }
        public string output_station { get; set; }
        public string station_job { get; set; }
    }

    public sealed class StationSidecar
    {
        public string schema { get; set; }
        public int version { get; set; }
        public string output_station { get; set; }
        public string controller_task { get; set; }
        public string calib_module_name { get; set; }
        public string calib_module { get; set; }
        public string path_module { get; set; }
        public string path_module_name { get; set; }
    }

    public sealed class ModelInstallation
    {
        public double x { get; set; }
        public double y { get; set; }
        public double z { get; set; }
        public double rz_deg { get; set; }
    }

    public sealed class ExportStatus
    {
        public string state { get; set; }
        public string manifest_path { get; set; }
        public int completed { get; set; }
        public int total { get; set; }
        public string current_region { get; set; }
        public string message { get; set; }
        public List<JobStatus> jobs { get; set; }
    }

    public sealed class JobStatus
    {
        public string region_label { get; set; }
        public string state { get; set; }
        public string output_station { get; set; }
        public string message { get; set; }
    }
}
