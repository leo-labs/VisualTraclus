using System.Collections;
using System.Collections.Generic;
using VisualTraclus;

namespace VisualTraclus {
    public class Trajectory {

		public ICollection<Coordinate> Coordinates { get; }
        public int Id { get; }

        public Trajectory() {
            
            Coordinates = new List<Coordinate>();
			this.Id = nextId();
        }

        public int Count {
            get { return this.Coordinates.Count; }
        }

        public void Add(Coordinate coordinate) {
            this.Coordinates.Add(coordinate);
        }


		private static int currentId = 0;
		private static int nextId() {

			return currentId++;

		}
    }
}
