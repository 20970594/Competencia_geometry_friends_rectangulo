using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace GeometryFriendsAgents
{
    class DijkstraNode
    {
        public int nodeIndex;
        public DijkstraNode cameFrom;
        public List<int> collectedDiamonds;

        public DijkstraNode(int nodeIndex, DijkstraNode cameFrom, List<int> collectedDiamonds)
        {
            this.nodeIndex = nodeIndex;
            this.cameFrom = cameFrom;
            this.collectedDiamonds = collectedDiamonds;
        }
    }
}
