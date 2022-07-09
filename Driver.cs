using GeometryFriends;
using GeometryFriends.AI;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace GeometryFriendsAgents
{
    class Driver
    {

        private List<Node> nodes;
        private int[,] adjacencyMatrix;
        // private float[,] distanceMap;
        private int[,] directionMap;
        private Queue<Node> route;

        private Node previousNode;
        private Node nextNode;
        private Node nextNode2;

        private int previousDirection;
        private int direction;
        private int direction2;
        private int previousAction;
        private int action;
        private int action2;

        private float distance;
        private List<float> distanceList;

        bool output = false;

        enum Direction { Right, RightDown, Down, LeftDown, Left, LeftUp, Up, RightUp };

        public Driver(List<Node> nodes, int[,] adjacencyMatrix, int[,] directionMap, Queue<Node> route) 
        {
            this.nodes = nodes;
            this.adjacencyMatrix = adjacencyMatrix;
            //this.distanceMap = distanceMap;
            this.directionMap = directionMap;
            this.route = route;
            distanceList = new List<float>();

            this.previousNode = route.Dequeue();
            this.nextNode = route.Dequeue();
            this.nextNode2 = route.Dequeue();
            this.action = adjacencyMatrix[nodes.IndexOf(previousNode), nodes.IndexOf(nextNode)];
            this.action2 = adjacencyMatrix[nodes.IndexOf(nextNode), nodes.IndexOf(nextNode2)];
            this.direction = directionMap[nodes.IndexOf(previousNode), nodes.IndexOf(nextNode)];
            this.direction2 = directionMap[nodes.IndexOf(nextNode), nodes.IndexOf(nextNode2)];
            //fall down case, create pseudo node
            if (direction2 == 2)
            {
                Node newNext = GetOppositeFallDownNode(nextNode);
                if (newNext != null)
                {
                    nextNode = newNext;
                    if (output)
                    {
                        Log.LogInformation("Driver - FALL DOWN - new nextNode - " + nextNode);
                    }
                    this.action = 2;

                    distance = (float)Math.Sqrt(Math.Pow(nodes[0].getX() - nextNode.getX(), 2) + Math.Pow(nodes[0].getY() - nextNode.getY(), 2));
                    distanceList.Add(distance);
                    if (output)
                    {
                        Log.LogInformation("Driver - FALL DOWN - new distance - " + distance);
                    }
                }
            }
            if (output)
            {
                Log.LogInformation("Driver - Constructor");
                Log.LogInformation("Driver - preNode - " + previousNode);
                Log.LogInformation("Driver - nextNode - " + nextNode);
                Log.LogInformation("Driver - nextNode2 - " + nextNode2);
                Log.LogInformation("Driver - action - " + action);
                Log.LogInformation("Driver - action2 - " + action2);
                Log.LogInformation("Driver - direction - " + direction);
                Log.LogInformation("Driver - direction2 - " + direction2);
            }
        }

        public Moves GetAction(float[] squareInfo)
        {
            int x = (int)squareInfo[0];
            int y = (int)squareInfo[1];
            int vX = (int)squareInfo[2];
            int vY = (int)squareInfo[3];
            int h = (int)squareInfo[4];

            int hHalf = h / 2;
            int w = 10000 / h;

            int centerY = y;

            int alwaysCorrectH = h;
            int alwaysCorrectHHalf = hHalf;
            int alwaysCorrectW = w;
            //if (IsTwisted(hHalf, h, x, centerY, w))
            //{
            //    alwaysCorrectH = w;
            //    alwaysCorrectW = 10000 / alwaysCorrectH;
            //    alwaysCorrectHHalf = alwaysCorrectH / 2;
            //}
            y = y + alwaysCorrectHHalf;

            distance = (float)Math.Sqrt(Math.Pow(x - nextNode.getX(), 2) + Math.Pow(y - nextNode.getY(), 2));
            distanceList.Add(distance);
            if (output)
            {
                Log.LogInformation("Driver - Distance - " + distance);
            }

            //Algorithms
            if (distanceList.Count == 40 && distanceList[0] == distanceList[39] && RectangleAgent.nCollectiblesLeft > 0)
            {
                System.Diagnostics.Stopwatch sw = System.Diagnostics.Stopwatch.StartNew();
                int dijkstraAction = UseDijkstra(x, centerY);
                Log.LogInformation("Driver - Route recalc with Subgoal AStar in ms: " + sw.ElapsedMilliseconds);
                if (dijkstraAction >= 0)
                {
                    return (Moves)dijkstraAction;
                }

            }
            //Algorithms end

            if (distanceList.Count >= 40)
            {
                distanceList = new List<float>();
            }

            if (((distance - 3) <= (alwaysCorrectW / 2) && !nextNode.getPseudo()) || (direction == 6 && distance <= alwaysCorrectH) || ((direction == 2 || direction == 1 || direction == 3) && (nextNode.getY() - y) < 4 && distance < 3 * alwaysCorrectW) || (nextNode.getPseudo() && (distance - 3 < 3)))// (direction == 2)
            {
                distanceList = new List<float>();
                if (output)
                {
                    Log.LogInformation("Driver - old preNode - " + previousNode);
                }
                this.previousAction = action;
                this.previousDirection = direction;
                previousNode = nextNode;
                if (output)
                {
                    Log.LogInformation("Driver - new preNode - " + previousNode);
                }
                if (nextNode2 != null)
                {
                    nextNode = nextNode2;
                    if (output)
                    {
                        Log.LogInformation("Driver - new nextNode - " + nextNode);
                    }
                    this.action = action2;
                    this.direction = direction2;
                    if (output)
                    {
                        Log.LogInformation("Driver - new action - " + action);
                        Log.LogInformation("Driver - new direction - " + direction);
                    }

                    distance = (float)Math.Sqrt(Math.Pow(x - nextNode.getX(), 2) + Math.Pow(y - nextNode.getY(), 2));
                    distanceList.Add(distance);
                    if (output)
                    {
                        Log.LogInformation("Driver - new distance - " + distance);
                    }
                }
                else
                {
                    if (output)
                    {
                        Log.LogInformation("Driver - end of route, nextNode, no actions");
                    }
                    //Algorithms
                    /*if (runAlgorithm == 0)
                    {
                        System.Diagnostics.Stopwatch sw = System.Diagnostics.Stopwatch.StartNew();
                        UseMCTSWithAStar(x, centerY);
                        Log.LogInformation("Driver - Route recalc with MCTS and AStar in ms: " + sw.ElapsedMilliseconds);
                    }
                    else if (runAlgorithm == 1)
                    {
                        System.Diagnostics.Stopwatch sw = System.Diagnostics.Stopwatch.StartNew();
                        UseMCTS(x, centerY);
                        Log.LogInformation("Driver - Route recalc with MCTS in ms: " + sw.ElapsedMilliseconds);
                    }*/
                    
                    return Moves.NO_ACTION;
                    //Algorithms end
                }
                if (route.Count > 0)
                {
                    nextNode2 = route.Dequeue();
                    if (output)
                    {
                        Log.LogInformation("Driver - new nextNode2 - " + nextNode2);
                    }
                    this.action2 = adjacencyMatrix[nodes.IndexOf(nextNode), nodes.IndexOf(nextNode2)];
                    if (output)
                    {
                        Log.LogInformation("Driver - new action2 - " + action2);
                    }
                    this.direction2 = directionMap[nodes.IndexOf(nextNode), nodes.IndexOf(nextNode2)];
                    if (output)
                    {
                        Log.LogInformation("Driver - new direction2 - " + direction2);
                    }

                    //fall down case, create pseudo node
                    if (direction2 == 2)
                    {
                        Node newNext = GetOppositeFallDownNode(nextNode);
                        if (newNext != null)
                        {
                            nextNode = newNext;
                            if (output)
                            {
                                Log.LogInformation("Driver - FALL DOWN - new nextNode - " + nextNode);
                            }
                            this.action = 2;

                            distance = (float)Math.Sqrt(Math.Pow(x - nextNode.getX(), 2) + Math.Pow(y - nextNode.getY(), 2));
                            distanceList.Add(distance);
                            if (output)
                            {
                                Log.LogInformation("Driver - FALL DOWN - new distance - " + distance);
                            }
                        }
                    }
                }
                else
                {
                    if (output)
                    {
                        Log.LogInformation("Driver - end of route, nextNode2, no actions");
                    }
                    nextNode2 = null;
                    this.action2 = -1;
                    this.direction2 = -1;
                }

            }

            if (nextNode.getPseudo() && distanceList.Count >= 10 && distanceList[0] == distanceList[9])
            {
                if (direction == 0 && distance < 200)
                {
                    return Moves.MOVE_LEFT;
                }
                if (direction == 4 && distance < 200)
                {
                    return Moves.MOVE_RIGHT;
                }
            }

            if (IsDiagonalOrientation(hHalf, h, x, centerY, w) && distanceList.Count >= 15 && distanceList[0] == distanceList[14])
            {
                Random random = new Random();
                int ran = random.Next(2);
                if (ran == 0)
                {
                    if (output)
                    {
                        Log.LogInformation("Driver - Moves RIGHT because DiagonalOrientation");
                    }
                    return Moves.MOVE_RIGHT;
                }
                else if (ran == 1)
                {
                    if (output)
                    {
                        Log.LogInformation("Driver - Moves LEFT because DiagonalOrientation");
                    }
                    return Moves.MOVE_LEFT;
                }
            }

            if (previousDirection == 6 && (direction == 0 || direction == 4) && (Math.Abs(y - previousNode.getY()) > 4))
            {
                if (alwaysCorrectH < 160 && CanMorphUp(y - alwaysCorrectH - 35, x, alwaysCorrectW))
                {
                    return Moves.MORPH_UP;
                }
                else if (direction == 0)
                {
                    return Moves.MOVE_RIGHT;
                }
                else if (direction == 4)
                {
                    return Moves.MOVE_LEFT;
                }

            }

            // possible actions for direction upper right and upper left
            //if (direction == 7 || direction == 5)
            //{
            //    if (alwaysCorrectH < 160 && CanMorphUp(y - alwaysCorrectH - 35, x, alwaysCorrectW))//!RectangleAgent.obstacleOpenSpace[y - alwaysCorrectH - 7, x])
            //    {
            //        Log.LogInformation("Driver - test52");
            //        return Moves.MORPH_UP;
            //    }
            //    else if (direction == 7)
            //    {
            //        Log.LogInformation("Driver - test112");
            //        return Moves.MOVE_RIGHT;
            //    }
            //    else if (direction == 5)
            //    {
            //        Log.LogInformation("Driver - test102");
            //        return Moves.MOVE_LEFT;
            //    }
            //}

            if (action == 1 && alwaysCorrectH > 102 && direction != 6)
            {
                return Moves.MORPH_DOWN;
            }
            if (action == 1 && alwaysCorrectH < 98 && CanMorphUp(y - alwaysCorrectH, x, alwaysCorrectW))
            {
                return Moves.MORPH_UP;
            }
            if (action == 2 && alwaysCorrectH > 52)
            {
                return Moves.MORPH_DOWN;
            }
            if (action == 3 && direction != 6 && alwaysCorrectH < 194 && CanMorphUp(y - alwaysCorrectH, x, alwaysCorrectW) && !IsDiagonalOrientation(hHalf, h, x, centerY, w) && vY < 5)
            {
                return Moves.MORPH_UP;
            }
            if (direction == 0 || direction == 7 || direction == 1)
            {
                if ((distance < 110 && vX > 50 && (direction2 != 0)) || vX > 200)
                {
                    return Moves.MOVE_LEFT;
                }
                else
                {
                    if (nextNode.getPseudo() && distance < 12 && vX > 2)
                    {
                        return Moves.MOVE_LEFT;
                    }
                    if (nextNode.getPseudo() && distance < 6 && vX < 2 && vX > -2)
                    {
                        return Moves.NO_ACTION;
                    }
                    return Moves.MOVE_RIGHT;
                }
                
            }
            if (direction == 4 || direction == 5 || direction == 3)
            {
                if ((distance < 110 && vX < -50 && (direction2 != 4)) || vX < -200)
                {
                    return Moves.MOVE_RIGHT;
                }
                else
                {
                    if (nextNode.getPseudo() && distance < 12 && vX < -2)
                    {
                        return Moves.MOVE_RIGHT;
                    }
                    if (nextNode.getPseudo() && distance < 6 && vX < 2 && vX > -2)
                    {
                        return Moves.NO_ACTION;
                    }
                    return Moves.MOVE_LEFT;
                }
            }
            if (direction == 6)
            {
                if (alwaysCorrectH < 194 && CanMorphUp(y - alwaysCorrectH, x, alwaysCorrectW))
                {
                    return Moves.MORPH_UP;
                }
                else if (Math.Abs(nextNode.getX() - x) > (alwaysCorrectW / 2))
                {
                    if ((x - nextNode.getX()) < 0)
                    {
                        return Moves.MOVE_RIGHT;
                    }
                    else
                    {
                        return Moves.MOVE_LEFT;
                    }
                }

            }
            if ((direction == 2) && nextNode.getLeadsToFallDown() && ((Math.Abs(y - previousNode.getY()) > alwaysCorrectH - 15 && Math.Abs(previousNode.getY() - nextNode.getY()) > 200) || (Math.Abs(y - previousNode.getY()) > alwaysCorrectH - 45 && Math.Abs(previousNode.getY() - nextNode.getY()) <= 200)))
            {
                return Moves.MORPH_DOWN;
            }
            if ((!previousNode.getPseudo() && direction == 2 && (previousDirection == 0 || previousDirection == 4) && (Math.Abs(y - previousNode.getY()) < 5)) || ((previousDirection == 0 || previousDirection == 4) && !CanMorphUp(y - alwaysCorrectH, x, alwaysCorrectW)))
            {
                if (previousDirection == 0 && vX < 40)
                {
                    return Moves.MOVE_RIGHT;
                }
                else if (previousDirection == 4 && vX < -40)
                {
                    return Moves.MOVE_LEFT;
                }
            }
            if (previousNode.getLeadsToFallDown() && !previousNode.getPseudo() && direction == 2 || direction == 3 || direction == 1 && CanMorphUp(y - alwaysCorrectH, x, alwaysCorrectW) && (vX > 50 || vX < -50))
            {
                if (previousNodeToObstacleDistance() <= 125)
                {
                    return Moves.MORPH_UP;
                }
            }
            if (output)
            {
                Log.LogInformation("Driver - end of driver, no actions");
            }
            return Moves.NO_ACTION;
        }

        public bool IsDiagonalOrientation(int hHalf, int h, int x, int y, int w)
        {
            int index = 1;
            while (!RectangleAgent.obstacleOpenSpace[y + index, x])
            {
                index++;
            }
            if (Math.Abs(hHalf - index) >= 2.5 && Math.Abs((index * 2) - w) > 5)
            {
                if (index > 101)
                {
                    if (output)
                    {
                        Log.LogInformation("Driver - DiagonalOrientation false, FALL DOWN, " + hHalf + " " + index);
                    }
                    return false;
                }
                if (output)
                {
                    Log.LogInformation("Driver - DiagonalOrientation true, " + hHalf + " " + index);
                }
                return true;
            }
            if (output)
            {
                Log.LogInformation("Driver - DiagonalOrientation false, " + hHalf + " " + index);
            }
            return false;
        }
        public Node GetOppositeFallDownNode(Node nextNode)
        {
            List<Node> possibleNodes = new List<Node>();
            foreach (Node node in nodes)
            {
                if (node.getLeadsToFallDown() && node.getY() == nextNode.getY() && Math.Abs(node.getX() - nextNode.getX()) <= 200 && (node.getX() - nextNode.getX()) != 0)
                {
                    Node newNode = new Node(((node.getX() + nextNode.getX()) / 2), nextNode.getY(), false);
                    newNode.setPseudo(true);
                    possibleNodes.Add(newNode);
                }
            }
            Node selectedNode = null;
            if (output)
            {
                Log.LogInformation("Driver - GetOppositeFallDownNode, found: " + possibleNodes.Count);
            }
            for (int i = 0; i < possibleNodes.Count; i++)
            {
                if (output)
                {
                    Log.LogInformation("Driver - GetOppositeFallDownNode, found: " + possibleNodes[i]);
                }
                if (direction == 0 && possibleNodes[i].getX() > previousNode.getX())
                {
                    selectedNode = possibleNodes[i];
                }
                if (direction == 4 && possibleNodes[i].getX() < previousNode.getX())
                {
                    selectedNode = possibleNodes[i];
                }
            }
            if (output)
            {
                Log.LogInformation("Driver - GetOppositeFallDownNode, selected: " + selectedNode);
            }
            return selectedNode;
        }

        private int previousNodeToObstacleDistance()
        {
            int iter = 7;
            int i = 1;
            int x = previousNode.getX();
            int y = previousNode.getY();
            if (previousDirection == 3 || previousDirection == 4)
            {
                iter = iter * -1;
            }
            while (!RectangleAgent.obstacleOpenSpace[y, x + (i * iter)])
            {
                i++;
            }
            return Math.Abs(i * iter);
        }
        public bool CanMorphUp(int upperY, int xCenter, int w)
        {
            int yUpperThreshold = 10;
            int step = 10;
            int xStart = xCenter - (w / 2);

            for (int i = 0; i < w; i = i + step)
            {
                if (RectangleAgent.obstacleOpenSpace[upperY - yUpperThreshold, xStart + i])
                {
                    return false;
                }
            }
            if (RectangleAgent.obstacleOpenSpace[upperY - yUpperThreshold, xStart + w - 1])
            {
                return false;
            }
            return true;
        }

        private int UseDijkstra(int x, int centerY)
        {
            Log.LogInformation("Driver Subgoal AStar - Subgoal AStar start");

            int s = 1;
            while (!RectangleAgent.obstacleOpenSpace[centerY + s, x])
            {
                s++;
            }
            Node square = new Node(x, centerY + s - 1, false);
            this.nodes[0] = square;
            int y = square.getY();

            deleteCollectedDiamonds();

            List<int> diamondNodes = new List<int>();
            for (int n = 0; n < nodes.Count; n++)
            {
                if (nodes[n].getDiamond())
                {
                    diamondNodes.Add(n);
                }
            }

            RectangleAgent.nodes = this.nodes;
            RectangleAgent.CreateEdgesAndAdjacencyMatrix();
            this.adjacencyMatrix = RectangleAgent.adjacencyMatrix;
            this.directionMap = RectangleAgent.directionMap;

            Dijkstra dijkstra = new Dijkstra(0, diamondNodes, 0);
            route = dijkstra.Run();
            int diamondsToCollect = diamondNodes.Count - 1;
            while (route == null)
            {
                dijkstra = new Dijkstra(0, diamondNodes, 0);
                route = dijkstra.Run();
                diamondsToCollect--;
                if (diamondsToCollect == 0)
                {
                    route = new Queue<Node>();
                }
            }

            return recalcNextNodes("Subgoal AStar", x, y);
        }
        private void deleteCollectedDiamonds()
        {
            float[] colInfo = RectangleAgent.collectiblesInfo;
            List<Node> colLeftList = new List<Node>();
            for (int i = 0; i < colInfo.Length; i = i + 2)
            {
                Node node = new Node((int)colInfo[i], (int)colInfo[i + 1], true);
                colLeftList.Add(node);
            }

            for (int index = 0; index < nodes.Count; index++)
            {
                Node nodeOfFullList = nodes[index];
                if (nodeOfFullList.getDiamond())
                {
                    bool isDiamond = false;
                    foreach (Node leftDiamond in colLeftList)
                    {
                        if ((nodeOfFullList.getX() == leftDiamond.getX() && nodeOfFullList.getY() == leftDiamond.getY()) || ((nodeOfFullList.getY() - leftDiamond.getY()) <= 80 && (nodeOfFullList.getY() - leftDiamond.getY()) > 0 && nodeOfFullList.getX() == leftDiamond.getX()))
                        {
                            isDiamond = true;
                        }
                    }
                    if (!isDiamond)
                    {
                        nodeOfFullList.setDiamond(false);
                        nodes[index] = nodeOfFullList;
                    }
                }
            }
        }

        public int recalcNextNodes(String output, int x, int y)
        {
            if (route == null)
            {
                return -1;
            }
            if (route.Count > 0)
            {
                this.previousNode = route.Dequeue();
                if (this.output)
                {
                    Log.LogInformation("Driver " + output + " - preNode - " + previousNode);
                }
            }
            else
            {
                if (this.output)
                {
                    Log.LogInformation("Driver " + output + " - end of route, preNode,  no actions");
                }
                return (int)Moves.NO_ACTION;
            }
            if (route.Count > 0)
            {
                this.nextNode = route.Dequeue();
                if (this.output)
                {
                    Log.LogInformation("Driver " + output + " - nextNode - " + nextNode);
                }
                this.action = adjacencyMatrix[nodes.IndexOf(previousNode), nodes.IndexOf(nextNode)];
                this.direction = directionMap[nodes.IndexOf(previousNode), nodes.IndexOf(nextNode)];
                if (this.output)
                {
                    Log.LogInformation("Driver " + output + " - action - " + action);
                    Log.LogInformation("Driver " + output + " - direction - " + direction);
                }
                distance = (float)Math.Sqrt(Math.Pow(x - nextNode.getX(), 2) + Math.Pow(y - nextNode.getY(), 2));
                if (this.output)
                {
                    Log.LogInformation("Driver " + output + " - Distance - " + distance);
                }

            }
            else
            {
                if (this.output)
                {
                    Log.LogInformation("Driver " + output + " - end of route, nextNode, no actions");
                }
                return (int)Moves.NO_ACTION;
            }
            if (route.Count > 0)
            {
                this.nextNode2 = route.Dequeue();
                if (this.output)
                {
                    Log.LogInformation("Driver " + output + " - nextNode2 - " + nextNode2);
                }
                this.action2 = adjacencyMatrix[nodes.IndexOf(nextNode), nodes.IndexOf(nextNode2)];
                if (this.output)
                {
                    Log.LogInformation("Driver " + output + " - action2 - " + action2);
                }
                this.direction2 = directionMap[nodes.IndexOf(nextNode), nodes.IndexOf(nextNode2)];
                if (this.output)
                {
                    Log.LogInformation("Driver " + output + " - direction2 - " + direction2);
                }
            }
            else
            {
                if (this.output)
                {
                    Log.LogInformation("Driver " + output + " - end of route, nextNode2, no actions");
                }
                nextNode2 = null;
                this.action2 = -1;
                this.direction2 = -1;
            }
            //fall down case, create pseudo node
            if (direction2 == 2)
            {
                Node newNext = GetOppositeFallDownNode(nextNode);
                if (newNext != null)
                {
                    nextNode = newNext;
                    if (this.output)
                    {
                        Log.LogInformation("Driver - FALL DOWN - new nextNode - " + nextNode);
                    }
                    this.action = 2;

                    distance = (float)Math.Sqrt(Math.Pow(nodes[0].getX() - nextNode.getX(), 2) + Math.Pow(nodes[0].getY() - nextNode.getY(), 2));
                    distanceList.Add(distance);
                    if (this.output)
                    {
                        Log.LogInformation("Driver - FALL DOWN - new distance - " + distance);
                    }
                }
            }
            Log.LogInformation("Driver " + output + " - " + output + " end");
            return -1;
        }
    }

}
