using System;
//using MathNet.Numerics.LinearAlgebra;
using MathNet.Spatial.Euclidean;


namespace VisualTraclus {

    public class Coordinate {

		public const float RADIUS_EARTH = 6371;

        public float Longitude { get; }
        public float Latitude { get;  }

        public Coordinate(float longitude, float latitude) {
            Longitude = longitude;
            Latitude = latitude;
        }

		public Vector3D Vector {
			get {
				return new Vector3D (X, Y, Z);
			}
		}

		public double X {
			get {
				return RADIUS_EARTH * Math.Cos (Algorithm.DegreesToRadians(Latitude)) * Math.Cos (Algorithm.DegreesToRadians(Longitude));
			}
		}

		public double Y {
			get {
				return RADIUS_EARTH * Math.Cos (Algorithm.DegreesToRadians(Latitude)) * Math.Sin (Algorithm.DegreesToRadians(Longitude));
			}
		}

		public double Z {
			get {
				return RADIUS_EARTH * Math.Sin (Algorithm.DegreesToRadians(Latitude));
			}
		}


		public override bool Equals(Object obj) {
			return obj is Coordinate && this == (Coordinate)obj;
		}
		public override int GetHashCode() {
			return Longitude.GetHashCode() ^ Latitude.GetHashCode();
		}
		public static bool operator ==(Coordinate x, Coordinate y) {
			
			return x?.Longitude == y?.Longitude && x?.Latitude == y?.Latitude;
		}
		public static bool operator !=(Coordinate x, Coordinate y) {
			return !(x == y);
		}
    }
}
