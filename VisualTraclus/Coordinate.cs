using System;
//using MathNet.Numerics.LinearAlgebra;
using MathNet.Spatial.Euclidean;


namespace VisualTraclus {

    public class Coordinate {

		const float RADIUS_EARTH = 6371;

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
				return RADIUS_EARTH * Math.Cos (Latitude) * Math.Cos (Longitude);
			}
		}

		public double Y {
			get {
				return RADIUS_EARTH * Math.Cos (Latitude) * Math.Sin (Longitude);
			}
		}

		public double Z {
			get {
				return RADIUS_EARTH * Math.Sin (Latitude);
			}
		}
    }
}
