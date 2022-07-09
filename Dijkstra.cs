using GeometryFriends;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace GeometryFriendsAgents
{
	class Dijkstra
	{
		List<DijkstraNode> Q;
		int[] V;
		int[,] cost;
		int[] dist;//distancia entre el nodo inicial y los demas nodos
		int[] previus;
		int start;
		List<int> goals;
		int numberOfDiamonds;
		int INF = 99999;

		bool output = false;

		public Dijkstra(int start, List<int> goals, int diamonds)
		{
			V = new int[RectangleAgent.nodes.Count];
			for(int i = 0; i < RectangleAgent.nodes.Count; i++)
			{
				V[i] = i;
			}
			this.start = start;
			this.goals = goals;
			dist = new int[RectangleAgent.nodes.Count];
			numberOfDiamonds = goals.Count;
			if (diamonds > 0)
			{
				numberOfDiamonds = diamonds;
			}

			for (int i = 0; i < V.Length; i++)
			{
				for (int j = 0; j < V.Length; j++)
				{
					cost[i, j] = RectangleAgent.directDistanceMap[i, j];
				}
			}

			Q = new List<DijkstraNode>();

			DijkstraNode startNode = new DijkstraNode(start);
			Q.Add(startNode);

			for(int i = 1; i < V.Length; i++)
			{
				Q.Add(new DijkstraNode(i));
			}
		}
		public Queue<Node> Run()
		{
			//asignando los valores en dist
			foreach(int v in V)
			{
				dist[v] = INF;
			}
			dist[0] = 0;

			while (Q.Count > 0)
			{
				int u = Q[0].nodeIndex;
                //u = nodo con menor costo en Q
                foreach (DijkstraNode i in Q)
                {
                    if (dist[i.nodeIndex] < dist[u])
                    {
						u = i.nodeIndex;
                    }
                }
				Q.Remove(Q[u]);

				if (dist[u] == INF)
				{
					break;
				}

				//la funcion Neighbours() retorna una lista de los vecinos de u
				List<int> neighbors = Neighbors(u);

				//se revisa cada vecino del nodo actual 
				foreach(int v in neighbors)
				{
					int alt = dist[u] + cost[u, v];
					if (alt < dist[v])
					{
						dist[v] = alt;
						previus[v] = u;
					}
				}
			}
			return Route(dist, previus);
		}

		private Queue<Node> Route(int[] dist, int[] previus)
        {
			Queue<Node> route = new Queue<Node>();
			
			List<int> diamondNodes= new List<int>();
			//que nodos tienen un diamante
			int i=0;
            foreach (Node n in RectangleAgent.nodes)
            {
                if (n.getDiamond())
                {
					diamondNodes.Add(i);
                }
				i++;
            }
			//nodo con menor distancia hacia el inicio
			int m=diamondNodes[0];
            foreach (int v in diamondNodes)
            {
                if (dist[v] < dist[m])
                {
					m = v;
                }
            }

			while(previus[m] != 0)
            {
				route.Enqueue(RectangleAgent.nodes[m]);
				m = previus[m];
            }

			return route;
        }
		private List<int> Neighbors(int current)
		{
			List<int> neighbors = new List<int>();
			for (int i = 0; i < RectangleAgent.nodes.Count; i++)
			{
				if (current != i)
				{
					if (RectangleAgent.adjacencyMatrix[current, i] > 0)
					{
						neighbors.Add(i);
					}
				}
			}
			return neighbors;
		}
	}
}
