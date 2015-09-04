using System;
using MathNet.Spatial.Euclidean;
using System.Collections.Generic;
using System.Linq;
using VisualTraclus;

namespace VisualTraclus {
	public static class RepresentativeTrajectoryGenerator {


		public static IEnumerable<Point3D> RepresentativeTrajectory(LineSegmentClustering.Cluster cluster, int minLns, double gamma) {

			var representativeTrajectory = new List<Point3D>();

			var averageDirectionVector = averageDirectionVectorOfCluster(cluster);
			var theta = averageDirectionVector.AngleTo(new Vector3D(1, 0, 0));

			//var firstAndLast = cluster.lineSegments.Select(l => l.First()).Union(cluster.lineSegments.Select(l => l.Last()));

			// order by X'
			//firstAndLast = firstAndLast.Select(c => c.Vector.ToPoint3D().Rotate(new Vector3D(0, 0, 0), theta)).OrderBy(p => p.X);

			var cs = CoordinateSystem.RotateTo(new Vector3D(1, 0, 0).Normalize(), averageDirectionVector.Normalize());
			var csrev = CoordinateSystem.RotateTo(averageDirectionVector.Normalize(), new Vector3D(1, 0, 0).Normalize());


			/*var firstAndLastStructs = cluster.lineSegments.Select(ls => new { 
				first = ls.First().Vector.ToPoint3D().Rotate(averageDirectionVector.Orthogonal, theta), 
				last = ls.Last().Vector.ToPoint3D().Rotate(averageDirectionVector.Orthogonal, theta)
			});*/

			var firstAndLastStructs = cluster.lineSegments.Select(ls => new { 
				first = cs.Transform(ls.start.ToPoint3D()), 
				last = cs.Transform(ls.end.ToPoint3D())
			});

			var firstAndLastStructsOrdered = firstAndLastStructs.OrderBy(fl => fl.first.X);
			var firstAndLast = firstAndLastStructs.SelectMany(fl => new[] {fl.first, fl.last}).OrderBy(p => p.X);


			for (int i = 0; i < firstAndLast.Count(); i++) {
				Point3D point = firstAndLast.ElementAt(i);

				var hittingLineSegments = new List<Line3D>();
				foreach (var linesegment in firstAndLastStructsOrdered) {
					if (linesegment.first.X <= linesegment.last.X) {
						if (linesegment.first.X <= point.X && linesegment.last.X >= point.X) {
							hittingLineSegments.Add(new Line3D(linesegment.first, linesegment.last));
						} 
						//if (linesegment.first.X > point.X) {
						//	break;
						//}
					} else {
						if (linesegment.first.X >= point.X && linesegment.last.X <= point.X) {
							hittingLineSegments.Add(new Line3D(linesegment.first, linesegment.last));
						} 
						//if (linesegment.first.X < point.X) {
					//		break;
					//	}
					}

				}
				if (hittingLineSegments.Count() >= minLns) {
					//var diff = point.X - firstAndLast.ElementAt(i - 1).X;

//					if (diff >= gamma) {
					var q = hittingLineSegments;
					//var q = firstAndLastStructsOrdered.Select(fl => new Line3D(fl.first, fl.last));
					try {
						var avgCoord = averageCoordinate(q, point, cs);
						avgCoord = csrev.Transform(avgCoord);
						representativeTrajectory.Add(avgCoord);
					
					} catch (Exception e) {
						//Console.WriteLine("Plane problem...");
					}
//					}
				}

			}

			return representativeTrajectory;
		}

		private static Vector3D averageDirectionVectorOfCluster(LineSegmentClustering.Cluster cluster) {

			var lineSegmentVectors = cluster.lineSegments.Select(ls => ls.end - ls.start);

			/*var averageDirectionVector = new List<Vector3D>();
			foreach(var lineSegment in cluster.lineSegments) {
				var directionVectors = lineSegment.end - lineSegment.start
				averageDirectionVector.Add(directionVectors.Sum()/directionVectors.Count());
			}*/
				
			return lineSegmentVectors.Sum()/lineSegmentVectors.Count();
		}

		public static Vector3D Sum(this IEnumerable<Vector3D> vector) {
			return vector.Aggregate((x, y) => x + y);
		}

		private static Point3D averageCoordinate(IEnumerable<Line3D> hittingLineSegments, Point3D point, CoordinateSystem cs) {


			Plane plane = new Plane(cs.Transform(new UnitVector3D(1, 0, 0)).Normalize(), point);
			//Plane plane = new Plane(point, new UnitVector3D(1, 0, 0));

			var hitPoints = hittingLineSegments.Select(hls => IntersectionWith(plane, hls));

			var y = hitPoints.Select(h => h.Y).Sum() / hitPoints.Count();
			var z = hitPoints.Select(h => h.Z).Sum() / hitPoints.Count();

			return new Point3D(point.X, y, z);
		}

		/// <summary>
		/// Find intersection between Line3D and Plane
		/// http://geomalgorithms.com/a05-_intersect-1.html
		/// 
		/// From MathNet.Spatial 
		/// </summary>
		/// <param name="line"></param>
		/// <param name="tolerance"></param>
		/// <returns>Intersection Point or null</returns>
		public static Point3D IntersectionWith(Plane plane, Line3D line, double tolerance = float.Epsilon) {
			if (line.Direction.IsPerpendicularTo(plane.Normal)) { //either parallel or lies in the plane
				Point3D projectedPoint = plane.Project(line.StartPoint, line.Direction);
				if (projectedPoint == line.StartPoint) { //Line lies in the plane
					throw new InvalidOperationException("Line lies in the plane"); //Not sure what should be done here
				} else { // Line and plane are parallel
					throw new InvalidOperationException("NULLLLLLL");
				}
			}
			var d = plane.SignedDistanceTo(line.StartPoint);
			var u = line.StartPoint.VectorTo(line.EndPoint);
			var t = -1 * d / u.DotProduct(plane.Normal);
			if (t > 1 || t < 0) { // They are not intersected
				throw new InvalidOperationException("NULLLLLLL");
			}
			return line.StartPoint + (t * u);
		}
	}
}

