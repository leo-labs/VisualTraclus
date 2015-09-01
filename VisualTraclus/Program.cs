using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using VisualTraclus;

namespace VisualTraclus {
    class Program  {

        private const string DATA_FILE = @"/Users/leo/Documents/taxi/Porto_taxi_data_training.csv";


        static void Main () {
			GenerateTrajFile();

			var trajectories = TrajectoryFileUtils.readData(DATA_FILE, lineCount:5000);

			Console.WriteLine ("Partitioning Phase");

			var ls = trajectories.Where(t => t.Count > 0)
				.Select (t => Algorithm.ApproximateTrajectoryPartition (t))
				.Where(l => l.Count() > 1)
				.ToList();

			Console.WriteLine ("GroupingPhase");
			var c = new LineSegmentClustering (ls);
			c.Cluster ();
        }

		static void GenerateTrajFile() {
			var trajectories = TrajectoryFileUtils.readData(DATA_FILE, lineCount:1000);
			TrajectoryFileUtils.ExportTrajectoriesIntoTraFile(trajectories, @"/Users/leo/Documents/uni/fp/1000.tra");
		}
    }
}
