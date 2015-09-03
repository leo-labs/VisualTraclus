//#using <mscorlib.dll>

using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices;
using VisualTraclus;


namespace VisualTraclus {
    class Program  {

        private const string DATA_FILE = @"/Users/leo/Documents/taxi/Porto_taxi_data_training.csv";


        static void Main () {
			//GenerateTrajFile();
			TestDistanceFunctons();


			var trajectories = TrajectoryFileUtils.readData(DATA_FILE, lineCount:20);

			Console.WriteLine ("Partitioning Phase");

			var ls = trajectories.Where(t => t.Count > 0)
				.Select (t => Algorithm.ApproximateTrajectoryPartition (t))
				.Where(l => l.Count() > 1)
				.ToList();

			Console.WriteLine ("GroupingPhase");
			var c = new LineSegmentClustering (ls);
			var clusters = c.GenerateCluster ();

			double gamma = 0;
			var r = clusters.Select(cl => RepresentativeTrajectoryGenerator.RepresentativeTrajectory(cl, 2.0d, gamma)).ToList();
			Console.WriteLine("represented");
        }

		static void GenerateTrajFile() {
			var trajectories = TrajectoryFileUtils.readData(DATA_FILE, lineCount:5);
			TrajectoryFileUtils.ExportTrajectoriesIntoTraFile(trajectories.Where(t => t.Count > 1).ToList(), @"/Users/leo/Documents/uni/fp/5_2_c.tra");
		}
			
		static void TestDistanceFunctons() {

			var c1 = new Coordinate(-8.618643f, 41.141412f);
			var c2 = new Coordinate(-8.618499f, 41.141376f);

			var c3 = new Coordinate(-8.639847f, 41.159825999999995f);
			var c4 = new Coordinate(-8.640350999999999f, 41.159871f);

			//euclidian Distance
			var dist12 = Algorithm.euclidianDistance(c1.Vector, c2.Vector);
			var dist34 = Algorithm.euclidianDistance(c3.Vector, c4.Vector);

			Console.WriteLine("Dist12: " + dist12 + "   Dist34: " + dist34);

			var dist123 = Algorithm.DistanceFromPointToLineSegment(c1, c2, c3);
			var dist324 = Algorithm.DistanceFromPointToLineSegment(c3, c2, c4);

			Console.WriteLine("Dist123: " + dist123 + "   Dist324: " + dist324);


			var ls1 = new List<Coordinate>();
			ls1.Add(c1);
			ls1.Add(c2);
			var ls2 = new List<Coordinate>();
			ls2.Add(c3);
			ls2.Add(c4);

			var distance1 = Algorithm.DistanceBetweenLineSegments(ls1, ls2);
			var distance2 = Algorithm.DistanceBetweenLineSegments(ls2, ls1);

			Console.WriteLine("Distance1: " + distance1 + "Distance2: " + distance2);
		}
    }
}
