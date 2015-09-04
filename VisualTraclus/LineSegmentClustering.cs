using System;
using System.Collections.Generic;
using VisualTraclus;
using System.Linq;

namespace VisualTraclus
{
	public class LineSegmentClustering
	{
		//private Dictionary<IEnumerable<Coordinate>, bool> classifyStatus;
		private List<LineSegment> noise;

		private Dictionary<LineSegment, int> clusterIds;

		private double epsilon; 
		private int minLns; 
		private IEnumerable<LineSegment> lineSegments;

		public struct Cluster {
			public int clusterId;
			public IList<LineSegment> lineSegments;
		}


		public LineSegmentClustering(IEnumerable<LineSegment> lineSegments, double epsilon, int minLns) {
			//classifyStatus = new Dictionary<IEnumerable<Coordinate>, bool> ();
			noise = new List<LineSegment> ();
			clusterIds = new Dictionary<LineSegment, int> ();

			this.lineSegments = lineSegments;
			this.epsilon = epsilon;
			this.minLns = minLns;
		}

		public IEnumerable<Cluster> GenerateCluster() {
			int clusterId = 0;

			int progress = 0;
			foreach (LineSegment lineSegement in lineSegments) {
				if (!clusterIds.ContainsKey(lineSegement)) {
					var neighborHood = Algorithm.EpsilonNeighborhood (lineSegments, lineSegement, epsilon);
					Console.WriteLine("Neighborhood.Count = " + neighborHood.Count());
					if (neighborHood.Count() >= minLns) {
						foreach (LineSegment neighbor in neighborHood) {
							clusterIds [neighbor] = clusterId;
						}
						Queue<LineSegment> queue = new Queue<LineSegment> ();
						foreach(LineSegment lineSegment in neighborHood.Where(ls => ls != lineSegement)) {
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
			var clusters = clusterIds.GroupBy(kvp => kvp.Value, x => x.Key, (key, z) => new Cluster { clusterId = key, lineSegments = z.ToList()});

			// Filter by count of participating trajectories
			clusters = clusters.Where(
				c => c.lineSegments.Select(ls => ls.Id).Distinct().Count() < minLns).ToList();

			Console.WriteLine ("clustered");
			return clusters;
		}

		private void expandCluster(Queue<LineSegment> queue, int clusterId) {
			while(queue.Count() != 0) {
				var head = queue.Dequeue();
				var neighborHood = Algorithm.EpsilonNeighborhood (lineSegments, head, epsilon);
				if (neighborHood.Count () >= minLns) {
					foreach (var lineSegment in neighborHood) {
						if(!clusterIds.ContainsKey(lineSegment) || noise.Contains(lineSegment)){
							if(!clusterIds.ContainsKey(lineSegment)) {
								queue.Enqueue(lineSegment);
							}
							clusterIds [lineSegment] = clusterId;
						}
					
					}
				}
			}

		}
			


	}
}

