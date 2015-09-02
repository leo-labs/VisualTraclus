using System;
using System.IO;
using System.Collections.Generic;
using System.Linq;

namespace VisualTraclus {
	public static class TrajectoryFileUtils {

		public static ICollection<Trajectory> readData(string filePath, int lineCount) {
			var trajectories = new List<Trajectory> ();

			var reader = new StreamReader(File.OpenRead(filePath));

			int i = 0;
			// skip first line
			reader.ReadLine();

			while (!reader.EndOfStream && i<lineCount) {
				var line = reader.ReadLine();
				var sep = new char[]{ ',' };
				var values = line.Split(sep, count:9);
				trajectories.Add(TrajectoryFileUtils.ParseTrajectory(values[8]));
				i++;
				if (i % 10000 == 0)
					Console.WriteLine (i);
			}

			return trajectories;
		}

		public static Trajectory ParseTrajectory(String polyline) {
			var trajectory = new Trajectory();

			var sep = new char[] { ']' };
			var split = polyline.Split(sep, options:StringSplitOptions.RemoveEmptyEntries);
			IEnumerable<string> points = split.Take(split.Count() - 1).Select(p => p.Remove(0, 3));
		
			foreach(var point in points) {
				var coordinate = ParseCoordinate (point);
					
				if (trajectory.Coordinates.LastOrDefault () == null || isFarEnough (trajectory.Coordinates.LastOrDefault (), coordinate)) {
					trajectory.Add (coordinate);
				}
			}
			return trajectory;
		}

		private static Coordinate ParseCoordinate(String coordinate) {
			var splitCoordinate = coordinate.Split (',');
			float longitude;
			float latitude;

			if(splitCoordinate.Count() == 2 && float.TryParse(splitCoordinate[0], out longitude) && float.TryParse(splitCoordinate[1], out latitude)) {
				return new Coordinate(longitude, latitude);
			}
			throw new Exception("Error while trying to parse coordinate: " + coordinate);
		}

		/// <summary>
		/// Exports the trajectories into .tra file.
		/// </summary>
		/// <param name="trajectories">Trajectories for export.</param>
		public static void ExportTrajectoriesIntoTraFile(ICollection<Trajectory> trajectories, string filePath) {

			using(var stream = new System.IO.FileStream(filePath, FileMode.CreateNew)) {
				using(var writer = new System.IO.StreamWriter(stream)) {
						
					writer.WriteLine("2");
					writer.WriteLine(trajectories.Count);

					int index = 0;
					foreach(var trajectory in trajectories) {
						writer.Write(index);
						foreach(var coordinate in trajectory.Coordinates) {
							writer.Write(" " + Math.Ceiling(coordinate.X) + " " + Math.Ceiling(coordinate.Y));
						}
						writer.WriteLine();
						index++;
					}
				}
			}
		}
			
		/// <summary>
		/// Tests if the euclidian norm is greater than the smallest possible positive value of an float.
		/// </summary>
		/// <returns><c>true</c>, if greater the smallest possible positive value of an float, <c>false</c> otherwise.</returns>
		/// <param name="c1">First Coordinate</param>
		/// <param name="c2">Second Coordinate</param>
		private static bool isFarEnough(Coordinate c1, Coordinate c2) {

			var dif = c2.Vector - c1.Vector;
			var l = Math.Sqrt((dif.X*dif.X) + (dif.Y*dif.Y) + (dif.Z*dif.Z));

			bool b = l >= float.Epsilon;

			return b;
		}
	}
}

