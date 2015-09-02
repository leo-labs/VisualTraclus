using System;
using System.Collections.Generic;
using VisualTraclus;
using System.Linq;

namespace VisualTraclus
{
	public class LineSegmentClustering
	{
		//private Dictionary<IEnumerable<Coordinate>, bool> classifyStatus;
		private List<IEnumerable<Coordinate>> noise;

		private Dictionary<IEnumerable<Coordinate>, int> clusterIds;

		private const double EPSILON = 20.0;
		private const int MIN_DENSITY = 8;
		private IEnumerable<IEnumerable<Coordinate>> lineSegments;

		public LineSegmentClustering(IEnumerable<IEnumerable<Coordinate>> lineSegments) {
			//classifyStatus = new Dictionary<IEnumerable<Coordinate>, bool> ();
			noise = new List<IEnumerable<Coordinate>> ();
			clusterIds = new Dictionary<IEnumerable<Coordinate>, int> ();

			this.lineSegments = lineSegments;
		}

		public void Cluster() {
			int clusterId = 0;

			int progress = 0;
			foreach (IEnumerable<Coordinate> lineSegement in lineSegments) {
				if (!clusterIds.ContainsKey(lineSegement)) {
					var neighborHood = epsilonNeighborhood (lineSegement);
					if (neighborHood.Count() >= MIN_DENSITY) {
						foreach (IEnumerable<Coordinate> neighbor in neighborHood) {
							clusterIds [neighbor] = clusterId;
						}
						Queue<IEnumerable<Coordinate>> queue = new Queue<IEnumerable<Coordinate>> ();
						foreach(IEnumerable<Coordinate> lineSegment in neighborHood.Where(ls => ls != lineSegement)) {
							queue.Enqueue(lineSegement);
						}
						expandCluster (queue, clusterId);
						clusterId++;
					} else {
						noise.Add (lineSegement);
					}

				}
				progress++;
				Console.WriteLine("Progress: Linesegment " + progress + "/" + lineSegments.Count());
			
			}


			// step 3
			//List<ICollection<IEnumerable<Coordinate>>> cluster = new List<IEnumerable<Coordinate>>(clusterId + 1);
			var clusters = clusterIds.GroupBy(kvp => kvp.Value, x => x.Key, (key, z) => new { clusterId = key, lineSegments = z.ToList()});
			clusters.ToList ();

			Console.WriteLine ("clustered");
			// TODO
		}

		private void expandCluster(Queue<IEnumerable<Coordinate>> queue, int clusterId) {
			while(queue.Count() != 0) {
				var head = queue.Dequeue();
				var neighborHood = this.epsilonNeighborhood (head);
				if (neighborHood.Count () >= MIN_DENSITY) {
					foreach (var lineSegment in neighborHood) {
						if(clusterIds.ContainsKey(lineSegment) || noise.Contains(lineSegment)){
							if(clusterIds.ContainsKey(lineSegment)) {
								queue.Enqueue(lineSegment);
							}
							clusterIds [lineSegment] = clusterId;

						}
					
					}
				}
			}

		}

		private IEnumerable<IEnumerable<Coordinate>> epsilonNeighborhood (IEnumerable<Coordinate> lineSegment) {
			var tr = lineSegments.Select(l => Algorithm.DistanceBetweenLineSegments(l, lineSegment));
			var mean = tr.Average();
			return lineSegments.Where (l => Algorithm.DistanceBetweenLineSegments (l, lineSegment) <= EPSILON);
		}


	}
}

