using System;
using MathNet.Spatial.Euclidean;

namespace VisualTraclus {
	public struct LineSegment {

		public Vector3D start { get; }
		public Vector3D end { get; }
		public int Id { get; }

		public LineSegment(Vector3D start, Vector3D end, int id) {
			if (start == end) {
				throw new ArgumentException("start and end is equal");
			}
			this.start = start;
			this.end = end;
			this.Id = id;
		}

		public override bool Equals(Object obj) 
		{
			return obj is LineSegment && this == (LineSegment)obj;
		}
		public override int GetHashCode() 
		{
			return Id ^ (10037) * start.GetHashCode() ^ end.GetHashCode();
		}
		public static bool operator ==(LineSegment x, LineSegment y) 
		{
			return x.start == y.start && x.end == y.end && x.Id == y.Id;
		}
		public static bool operator !=(LineSegment x, LineSegment y) 
		{
			return !(x == y);
		}
	}
}

