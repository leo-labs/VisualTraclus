//#using <mscorlib.dll>

using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices;
using VisualTraclus;
using GoogleMapsApi;
using GoogleMapsApi.StaticMaps.Entities;
using GoogleMapsApi.Entities.Common;

namespace VisualTraclus {
    class Program  {

        private const string DATA_FILE = @"/Users/leo/Documents/taxi/Porto_taxi_data_training.csv";


		static int Main (string[] args) {
			//GenerateTrajFile();
			//TestDistanceFunctons();

			if (args.Length != 3) {
				Console.WriteLine("Usage: Visualtraclus.exe <epsilon> <minLns> <lineCount>");
				return 1;
			}
			double epsilon;
			int minLns;
			int lines;

			if(!Double.TryParse(args[0], out epsilon)) { return 1; }
			if(!int.TryParse(args[1], out minLns)) { return 1; }
			if(!int.TryParse(args[2], out lines)) { return 1; }


			//double epsilon = 2.0;
			//int minLns = 250;
			//int lines = 10000;

			Console.WriteLine("Reading data! Please wait...");

			var trajectories = TrajectoryFileUtils.readData(DATA_FILE, lineCount:lines);

			Console.WriteLine ("Partitioning Phase");

			var characteristicPointsByTrajectories = trajectories.Where(t => t.Count > 0)
				.Select (t => new Tuple<IEnumerable<Coordinate>, int>(Algorithm.ApproximateTrajectoryPartition (t), t.Id))
				.Where(l => l.Item1.Count() > 1)
				.ToList();
			
			var ls = characteristicPointsByTrajectories.Select(
				charPoints => charPoints.Item1.Zip(charPoints.Item1.Skip(1), 
					(first, second) => new LineSegment(first.Vector, second.Vector, charPoints.Item2)));

			var lineSegments = ls.SelectMany(l => l);
			Console.WriteLine("Extracted " + lineSegments.Count() + " linesegments");

			//var avgLs = new List<int>();
			/*foreach(var l in ls) {
				avgLs.Add(Algorithm.epsilonNeighborhood(ls, l, epsilon).Count());
				Console.WriteLine(avgLs.Count());
			}
			var meanMinLns = avgLs.Sum() / avgLs.Count();
			Console.WriteLine("AverageMINLns: " + meanMinLns);*/

			Console.WriteLine ("GroupingPhase");
			var c = new LineSegmentClustering (lineSegments, epsilon, minLns);
			var clusters = c.GenerateCluster ();

			double gamma = 0;
			var r = clusters.Select(cl => RepresentativeTrajectoryGenerator.RepresentativeTrajectory(cl, 2, gamma)).ToList();
			var r2 = clusters.Select(cl => RepresentativeTrajectoryGenerator.RepresentativeTrajectory(cl, minLns, gamma)).ToList();

			Console.WriteLine("Representative Trajectory computed!");
			/*IList<IList<ILocationString>> trajectoryLocations = r.Select(
				l => (IList<ILocationString>)l.Select(
					t => (ILocationString)new Location(Math.Asin(t.Z / Coordinate.RADIUS_EARTH), Math.Atan2(t.Y, t.X))).ToList()
			).ToList();

			IList<GoogleMapsApi.StaticMaps.Entities.Path> pathes = trajectoryLocations.Select(tl => new GoogleMapsApi.StaticMaps.Entities.Path() {
				Style = new PathStyle() {
					Color = "red"
				},
				Locations = tl
			}).ToList();


			var engine = new GoogleMapsApi.StaticMaps.StaticMapsEngine();
			string url = engine.GenerateStaticMapURL(new GoogleMapsApi.StaticMaps.Entities.StaticMapRequest(new Location(41.159364, -8.629749), 16, new GoogleMapsApi.StaticMaps.Entities.ImageSize(1600, 1200)) 
				{
					Pathes = pathes
				});
			
			Console.WriteLine(url);
			*/
			var datetime = DateTime.Now.ToString("yyyy-MM-dd-HH-mm-ss");

			TrajectoryFileUtils.ExportClusters(r, @"/Users/leo/Documents/uni/fp/oc"+ lines + "_" + epsilon + "_" + minLns + "_" + datetime + ".clus");
			TrajectoryFileUtils.ExportClusters(r2, @"/Users/leo/Documents/uni/fp/wc"+ lines + "_" + epsilon + "_" + minLns + "_" + datetime + ".clus");

			return 0;
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


			Console.WriteLine("" + TrajectoryFileUtils.RadianToDegree(Math.Asin(c1.Z / Coordinate.RADIUS_EARTH)) + ", " + TrajectoryFileUtils.RadianToDegree(Math.Atan2(c1.Y, c1.X)));

			Console.WriteLine("X: " + c1.X + ", Y: " + c1.Y + ", Z: " + c1.Z);
			//var dist123 = Algorithm.DistanceFromPointToLineSegment(c1, c2, c3);
			//var dist324 = Algorithm.DistanceFromPointToLineSegment(c3, c2, c4);

			//Console.WriteLine("Dist123: " + dist123 + "   Dist324: " + dist324);


			var ls1 = new List<Coordinate>();
			ls1.Add(c1);
			ls1.Add(c2);
			var ls2 = new List<Coordinate>();
			ls2.Add(c3);
			ls2.Add(c4);

			//var distance1 = Algorithm.DistanceBetweenLineSegments(ls1, ls2);
			//var distance2 = Algorithm.DistanceBetweenLineSegments(ls2, ls1);

			//Console.WriteLine("Distance1: " + distance1 + "Distance2: " + distance2);
		}
    }
}
