using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using VisualTraclus;

namespace VisualTraclus {
    class Program  {

        private const string DATA_FILE = @"/Users/leo/Documents/taxi/Porto_taxi_data_training.csv";

        private static ICollection<Trajectory> trajectories;

        static void Main () {
			trajectories = new List<Trajectory> ();
            readData();
        }


        private static void readData() {
            var reader = new StreamReader(File.OpenRead(DATA_FILE));

            int i = 0;
			// skip first line
			reader.ReadLine();

            while (!reader.EndOfStream && i<10000) {
                var line = reader.ReadLine();
				var sep = new char[]{ ',' };
				var values = line.Split(sep, count:9);
				trajectories.Add(parseTrajectory(values[8]));
                i++;
				if (i % 10000 == 0)
					Console.WriteLine (i);
				
            }
			Console.WriteLine (s + " points ignored");
			Console.WriteLine ("Partitioning Phase");

			var ls = trajectories.Where(t => t.Count > 0)
				.Select (t => Algorithm.ApproximateTrajectoryPartition (t))
				.Where(l => l.Count() > 1)
				.ToList();
			
			Console.WriteLine ("GroupingPhase");
			var c = new LineSegmentClustering (ls);
			c.Cluster ();

        }
		private static int s = 0;

		private static Trajectory parseTrajectory(String polyline) {
			var t = new Trajectory();

			var sep = new char[] { ']' };
			var split = polyline.Split(sep, options:StringSplitOptions.RemoveEmptyEntries);
			IEnumerable<string> points = split.Take(split.Count() - 1).Select(p => p.Remove(0, 3));


			foreach(var p in points) {
				var c = parseCoordinate (p);
				//if(!(t.Coordinates.LastOrDefault()?.Latitude == c.Latitude && t.Coordinates.LastOrDefault()?.Longitude == c.Longitude))


				if (t.Coordinates.LastOrDefault () == null || fairEnough (t.Coordinates.LastOrDefault (), c)) {
					t.Add (c);
				}
				else {
					s++;
					//Console.WriteLine ("Too Shrt:" + s);
				}
			}
			return t;
		}

		private static Coordinate parseCoordinate(String coordinate) {
			var splitCoordinate = coordinate.Split (',');
			float longitude;
			float latitude;
			if(splitCoordinate.Count() == 2 && float.TryParse(splitCoordinate[0], out longitude) && float.TryParse(splitCoordinate[1], out latitude)) {
					return new Coordinate(longitude, latitude);
			}
			throw new Exception("dewdewd");
		}

		public static bool fairEnough(Coordinate c1, Coordinate c2) {
		
			var dif = c2.Vector - c1.Vector;
			var l = Math.Sqrt((dif.X*dif.X) + (dif.Y*dif.Y) + (dif.Z*dif.Z));


			bool b = l >= float.Epsilon;

			return b;
		}
    }
}
