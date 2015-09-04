using System;
using System.Collections.Generic;
using System.Linq;
using VisualTraclus;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics;
using MathNet.Spatial.Euclidean;
using MathNet.Spatial.Units;

namespace VisualTraclus {
	public static class Algorithm {
		private static int cw = 0;

		public static IEnumerable<Coordinate> ApproximateTrajectoryPartition(Trajectory trajectory) {

			var characteristicPoints = new List<Coordinate> ();
			characteristicPoints.Add (trajectory.Coordinates.First ());

			int startIndex = 0;
			int length = 0;

			while (startIndex + length < trajectory.Count) {

				int currentIndex = startIndex + length;

				int costPar = minimumDescriptionLengthPar (trajectory, startIndex, currentIndex);
				int costNoPar = minimumDescriptionLengthNoPar (trajectory, startIndex, currentIndex);

				if (costPar > costNoPar) {
					characteristicPoints.Add (trajectory.Coordinates.ElementAt (currentIndex - 1));
					startIndex = currentIndex - 1;
					length = 1;
				} else {
					length++;
				}
			}
			cw++;

			//Console.WriteLine("Progress: Trajectory " + cw + "/100000");

			return characteristicPoints.Distinct();

		}


		public static int minimumDescriptionLengthPar(Trajectory trajectory, int startIndex, int endIndex) {
			return descriptionCost (trajectory, startIndex, endIndex) + encodingCost (trajectory, startIndex, endIndex);
		}

		public static int minimumDescriptionLengthNoPar(Trajectory trajectory, int startIndex, int endIndex) {
			return descriptionCost (trajectory, startIndex, endIndex);
		}

		private static int descriptionCost(Trajectory trajectory, int startIndex, int endIndex) {

			Coordinate startSegment = trajectory.Coordinates.ElementAt (startIndex);
			Coordinate endSegment = trajectory.Coordinates.ElementAt (endIndex);

			double distance = euclidianDistance (startSegment.Vector, endSegment.Vector);
			if (distance < 1.0)
				distance = 1.0f;

			return (int)Math.Ceiling (Math.Log (distance, 2));

		}

		private static int encodingCost(Trajectory trajectory, int startIndex, int endIndex) {
			double perpendicularDist;
			double angleDist;
			int encodingCost = 0;

			Coordinate startComponent = trajectory.Coordinates.ElementAt (startIndex);
			Coordinate endComponent = trajectory.Coordinates.ElementAt (endIndex);

			//TODO correct this
			if(startComponent.Vector == endComponent.Vector) return 0;

			for (int i = startIndex; i < endIndex; i++) {
				Coordinate startSegment = trajectory.Coordinates.ElementAt (i);
				Coordinate endSegment = trajectory.Coordinates.ElementAt (i + 1);

				perpendicularDist = perpendicularDistance(startComponent.Vector, endComponent.Vector, startSegment.Vector, endSegment.Vector);
				angleDist = angleDistance(startComponent.Vector, endComponent.Vector, startSegment.Vector, endSegment.Vector);

				if (perpendicularDist< 1.0) 
					perpendicularDist = 1.0f;

				if (angleDist < 1.0)
					angleDist = 1.0f;


				encodingCost += (int)Math.Ceiling(Math.Log(perpendicularDist, 2)) + 
					(int)Math.Ceiling(Math.Log(angleDist, 2));
			}

			return encodingCost;
		}

		public static double euclidianDistance(Vector3D v1, Vector3D v2) {

			/*int radiusEarth = 6371;

			double latitude = Math.Abs (x.Latitude - y.Latitude) * Math.PI / 180;
			double longitude = Math.Abs (x.Longitude - y.Longitude) * Math.PI / 180;

			double latitudeX = x.Latitude * Math.PI / 180;
			double latitudeY = y.Latitude * Math.PI / 180;

			double a = Math.Pow(Math.Sin(latitude/2),2) + Math.Cos(latitudeX * Math.Cos(latitudeY) * Math.Pow(Math.Sin(longitude/2), 2));
			double distance = 2 * Math.Atan2(Math.Sqrt(a), Math.Sqrt(1-a));
			
			return distance * radiusEarth;*/

			return Math.Sqrt (Math.Pow (v1.X - v2.X, 2) + Math.Pow (v1.Y - v2.Y, 2) + Math.Pow (v1.Z - v2.Z, 2));
			//return Distance.Euclidean (x.Vector.ToPoint3D, y.Vector.ToPoint3D);
		}

		private static double perpendicularDistance(Vector3D start1, Vector3D end1, Vector3D start2, Vector3D end2) {

			// we assume that the first line segment is longer than the second one

			// distance to cluster component
			double length1 = euclidianDistance(start1, end1);
			double length2 = euclidianDistance(start2, end2);

			double distanceFromStart;
			double distanceFromEnd;
			if (length1 > length2) {
				distanceFromStart = DistanceFromPointToLineSegment(start1, end1, start2);
				distanceFromEnd = DistanceFromPointToLineSegment(start1, end1, end2);
			} else {
				distanceFromStart = DistanceFromPointToLineSegment(start2, end2, start1);
				distanceFromEnd = DistanceFromPointToLineSegment(start2, end2, end1);
			}


			// if the first line segment is exactly the same as the second one, the perpendicular distance should be zero
			if (distanceFromStart == 0.0 && distanceFromEnd == 0.0) 
				return 0.0d;

			// return (d1^2 + d2^2) / (d1 + d2) as the perpendicular distance
			return ((Math.Pow(distanceFromStart, 2) + Math.Pow(distanceFromEnd, 2)) / (distanceFromStart + distanceFromEnd));

		}

		private static double angleDistance(Vector3D start1, Vector3D end1, Vector3D start2, Vector3D end2) {

			double length1 = euclidianDistance(start1, end1);
			double length2 = euclidianDistance(start2, end2);

			Vector3D vector1, vector2;

			if(length1 > length2) {
				vector1 = end1 - start1;
				vector2 = end2 - start2;
			} else {
				vector2 = end1 - start1;
				vector1 = end2 - start2;
			}

			var theta = vector1.AngleTo (vector2);
			if (theta >= Angle.FromDegrees (0) && theta <= Angle.FromDegrees (90)) {
				return vector2.Length * theta.Radians;
			}
			return vector2.Length;
		}

		public static double DistanceFromPointToLineSegment(Vector3D start, Vector3D end, Vector3D p) {
			Vector3D projection = projectionOfPointToLineSegment (start, end, p);
			return euclidianDistance(p, projection);
		}

		public static Vector3D projectionOfPointToLineSegment(Vector3D lineSegmentStart, Vector3D lineSegmentEnd, Vector3D point) {
			var vector1 = point - lineSegmentStart;
			var vector2 = lineSegmentEnd - lineSegmentStart;

			// a coefficient (0 <= b <= 1)
			//m_coefficient = ComputeInnerProduct (&m_vector1, &m_vector2) / ComputeInnerProduct (&m_vector2, &m_vector2);
			double coeff = vector1.DotProduct(vector2) / vector2.DotProduct(vector2);

			// the projection on the cluster component from a given point	
			// NOTE: the variable m_projectionPoint is declared as a member variable

			//for (int i = 0; i < nDimensions; i++)
			//	m_projectionPoint.SetCoordinate (i, s->GetCoordinate (i) + m_coefficient * m_vector2.GetCoordinate (i));
			var projection = lineSegmentStart + coeff * vector2;
			return projection;
		}


		private const double WEIGHT_PERPENIDCULAR = 1.0d;
		private const double WEIGHT_PARALLEL = 1.0d;
		private const double WEIGHT_ANGLE = 1.0d;


		public static double DistanceBetweenLineSegments(LineSegment linesegment1, LineSegment linesegment2) {
			double perpendicularDist =  WEIGHT_PERPENIDCULAR * perpendicularDistance (linesegment1.start, linesegment1.end, linesegment2.start, linesegment2.end);
			double parallelDist = WEIGHT_PARALLEL * parallelDistance (linesegment1, linesegment2);
			double angleDist = WEIGHT_ANGLE * angleDistance(linesegment1.start, linesegment1.end, linesegment2.start, linesegment2.end);
		
			//Console.WriteLine("PerpendicularDistance: " + perpendicularDist);
			//Console.WriteLine("ParallelDistance: " + parallelDist);
			//Console.WriteLine("AngleDistance: " + angleDist);

			return perpendicularDist + parallelDist + angleDist;
		}

		private static double parallelDistance(LineSegment linesegment1, LineSegment linesegment2) {
			var start1 = linesegment1.start;
			var start2 = linesegment2.start;
			var end1 = linesegment1.end;
			var end2 = linesegment2.end;

			double length1 = euclidianDistance(start1, end1);
			double length2 = euclidianDistance(start2, end2);

			double parallelDist1, parallelDist2;

			if (length1 > length2) {
				var projection1 = projectionOfPointToLineSegment(start1, end1, start2);
				var projection2 = projectionOfPointToLineSegment(start1, end1, end2);
				parallelDist1 = Math.Min (euclidianDistance (projection1, start1), euclidianDistance (projection1, end1)); 
				parallelDist2 = Math.Min (euclidianDistance (projection2, start1), euclidianDistance (projection2, end1)); 

			} else {
				var projection1 = projectionOfPointToLineSegment(start2, end2, start1);
				var	projection2 = projectionOfPointToLineSegment(start2, end2, end1);
				parallelDist1 = Math.Min (euclidianDistance (projection1, start2), euclidianDistance (projection1, end2)); 
				parallelDist2 = Math.Min (euclidianDistance (projection2, start2), euclidianDistance (projection2, end2)); 
			}
			return Math.Min(parallelDist1, parallelDist2);
		}


		public static IEnumerable<LineSegment> EpsilonNeighborhood (IEnumerable<LineSegment> lineSegments, LineSegment lineSegment, double epsilon) {
			//var tr = lineSegments.Select(l => Algorithm.DistanceBetweenLineSegments(l, lineSegment));
			//var mean = tr.Average();
			return lineSegments.Where (l => Algorithm.DistanceBetweenLineSegments (l, lineSegment) <= epsilon);
		}

		public static double DegreesToRadians(double angle) {
			return angle * Math.PI / 180.0;
		}
	}


}

