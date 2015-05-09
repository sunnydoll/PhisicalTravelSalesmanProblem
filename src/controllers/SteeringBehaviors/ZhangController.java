package controllers.SteeringBehaviors;
import framework.core.*;
import framework.graph.Graph;
import framework.graph.Node;
import framework.graph.Path;
import framework.utils.Vector2d;
import java.util.HashMap;

/**
 * ZhangController combines steering behaviors and OutputFiltering method to pick out the best action to perform.
 * The major steering behaviors in this controller are the "seek" and the "PathFollowing". 
 * Path is found by the built in "PathFinder" Class, the getNextNode() and the GetClosestWaypoint() functions of "GreedyController" 
 * Class are also used to find the next node along the path and the current closest waypoint. 
 * Similar to the GreedyController, two scenarios are considered: ObstacleInBetween and NoObstacleInBetween.
 * 
 * 
 * Flow Chart:
 *                                     ---------------------------------------------
 *                                    |ship.velocity, ship.position, target.position|
 * Steering Behavior                   ---------------------------------------------   
 * ------------------------------------------------------- | ----------------------------------------------------------------------------                                
 *                                                 -----------------                        
 *                                                |  seek(nextNode) |                       
 *                                                 -----------------
 *                                                         |                       
 *                                                    ------------                                
 *                                                   |acceleration|
 * Motor Control Layer                                ------------                          
 * ------------------------------------------------------- | ----------------------------------------------------------------------------
 *                                                 -----------------                                
 *                                                | OutputFiltering |                       
 *                                                 ----------------- 
 *                                                  _______|_______
 *                                                 |               |
 *                                    -------------------      ------------------                          
 *                                   | Linear Projection |    |  Angle Rotation  |                 
 *                                    -------------------      ------------------
 * Command                                         |_______________|                
 * ------------------------------------------------------- | ---------------------------------------------------------------------------- 
 *                                                     ----------                                       
 *                                                    |bestAction|
 *                                                     ----------                           
 *                                                         
 */

public class ZhangController extends Controller
{
    
    private Graph MapGraph;
    private Node ClosestNode;
    private Path PathToWaypoint;
    private HashMap<Waypoint, Node> BetaNodes;
    private Waypoint ClosestWaypoint;

    public ZhangController(Game a_gameCopy, long a_timeDue)
    {
        //Create the graph based on current game(map).
    	MapGraph = new Graph(a_gameCopy);
    	
        //BetaNodes are nodes closest to the WayPoints.
        BetaNodes = new HashMap<Waypoint, Node>();
        for(Waypoint way: a_gameCopy.getWaypoints())
        {
        	BetaNodes.put(way, MapGraph.getClosestNodeTo(way.s.x, way.s.y,true));
        }

        //Initialize the first closest WayPoint.
        GetClosestWaypoint(a_gameCopy);
    }

    public int getAction(Game a_gameCopy, long a_timeDue)
    {
        //Find the closest node to the ship current location.
        ClosestNode = MapGraph.getClosestNodeTo(a_gameCopy.getShip().s.x, a_gameCopy.getShip().s.y,false);
        
        //Update the closest WayPoint to the ship current location.
        GetClosestWaypoint(a_gameCopy);

        //Get the path to Beta node from CloestNode.
        PathToWaypoint = MapGraph.getPath(ClosestNode.id(), BetaNodes.get(ClosestWaypoint).id());
        
        //Two scenarios are considered: ObstacleInBetween and NoObstacleInBetween.
        boolean NoObstacleInBetween = a_gameCopy.getMap().LineOfSight(a_gameCopy.getShip().s,ClosestWaypoint.s);
           
        //Scenario 1:
        if(NoObstacleInBetween)
        {
        	//Initialize the bestAction
        	int bestAction = Controller.ACTION_NO_FRONT;
                
            //Get the unit vector to the target
            Vector2d ND = ClosestWaypoint.s.copy();
            Vector2d EndCoordinates = a_gameCopy.getShip().s;
            ND.subtract(EndCoordinates);
            double distance = ND.mag();
            ND.normalise();

            //Computer the angle between DV and ND; Based on the visibility of the WayPoint, we perform different actions.
            Vector2d DV = a_gameCopy.getShip().d;
            boolean WaypointNotVisible = DV.scalarProduct(ND) < 0.9;
            
            //Adjust maxSpeed based on the time left and the distance to the WayPoint
            double maxSpeed = 0.4;
            if(distance>100){maxSpeed = 0.9;}
            else if(distance<30){maxSpeed = 0.3;}

            //Computer the acceleration from seek() and perform OutputFiltering() to figure out the bestAction
            if(WaypointNotVisible || a_gameCopy.getShip().v.mag() > maxSpeed)
            {
            	Node nextNode = getNextNode();  
                Vector2d shipPosition = a_gameCopy.getShip().s;
                Vector2d nextNodePosition = new Vector2d(nextNode.x(), nextNode.y());
                Vector2d acceleration = seek(shipPosition, nextNodePosition);
                bestAction = OutputFiltering(acceleration,a_gameCopy,ND);
                
            } 
            else
            { 
            	return Controller.ACTION_THR_FRONT;
            }
            
            return bestAction;
          
        }
        //Scenario 2:
        else
        {
        	if(PathToWaypoint != null)
            {
                int bestAction = OutputFiltering(a_gameCopy);
                return bestAction;
            }
        }

        return Controller.ACTION_NO_FRONT;
    }
      
    /**
     * Author: Created by Diego Perez, University of Essex.
     * Returns the first node in the way to the destination
     * @return the node in the way to destination.
     */
    private Node getNextNode()
    {
        Node n0 = MapGraph.getNode(PathToWaypoint.m_points.get(0));

        //If only one node in the path, return it.
        if(PathToWaypoint.m_points.size() == 1)
            return n0;

        //Heuristic: Otherwise, take the closest one to the destination
        Node n1 = MapGraph.getNode(PathToWaypoint.m_points.get(1));
        Node destination =  MapGraph.getNode(PathToWaypoint.m_destinationID);

        if(n0.euclideanDistanceTo(destination) < n1.euclideanDistanceTo(destination))
            return n0;
        else return n1;
    }

    /**
     * Author: Created by Diego Perez, University of Essex.
     * Calculates the closest waypoint to the ship.
     * @param a_gameCopy the game copy.
     */
    private void GetClosestWaypoint(Game a_gameCopy)
    {
        double minDistance = Double.MAX_VALUE;
        for(Waypoint way: a_gameCopy.getWaypoints())
        {
            if(!way.isCollected())     //Only consider those not collected yet.
            {
                double fx = way.s.x-a_gameCopy.getShip().s.x, fy = way.s.y-a_gameCopy.getShip().s.y;
                double dist = Math.sqrt(fx*fx+fy*fy);
                if( dist < minDistance )
                {
                    //Keep the minimum distance.
                    minDistance = dist;
                    ClosestWaypoint = way;
                }
            }
        }
    }
 

    /**
     * Returns the current acceleration from the shipPosition and targetPosition.
     * @param shipPosition current ship position.
     * @param targetPosition current target position.
     * @return the acceleration.
     */
    private Vector2d seek(Vector2d shipPosition,Vector2d targetPosition)
    {
    	Vector2d D = targetPosition.subtract(shipPosition);
    	D.normalise();
    	Vector2d acceleration = D.mul(1);
    	return acceleration;
    }
    
    /**
     * Returns the bestAction based on the acceleration given.
     * @param acceleration current acceleration.
     * @param DirectVector current DirectVector of ship.
     * @return the acceleration.
     */
    private int OutputFiltering(Vector2d acceleration,Vector2d DirectVector)
    {
    	int bestAction = Controller.ACTION_NO_FRONT;
    	double projection = acceleration.scalarProduct(DirectVector);
    	double rotation = acceleration.y-((double)(DirectVector.y/DirectVector.x))*acceleration.x;
    	if(projection>0 && rotation>0){
    		bestAction = Controller.ACTION_THR_LEFT;
    		System.out.println("ACTION_THR_LEFT" );
    	}
    	else if(projection>0 && rotation<0){
    		bestAction = Controller.ACTION_THR_RIGHT;
    		System.out.println("ACTION_THR_RIGHT" );
    	}
    	else if(projection<0 && rotation>0){
    		bestAction = Controller.ACTION_NO_LEFT;
    		System.out.println("ACTION_NO_LEFT" );
    	}
    	else if(projection<0 && rotation<0){
    		bestAction = Controller.ACTION_NO_RIGHT;
    		System.out.println("ACTION_NO_RIGHT" );
    	}
    	else if(projection>0 && rotation==0){
    		bestAction = Controller.ACTION_THR_FRONT;
    		System.out.println("ACTION_THR_FRONT" );
    	}
    	else if(projection<0 && rotation==0){
    		bestAction = Controller.ACTION_NO_FRONT;
    		System.out.println("ACTION_NO_FRONT" );
    	}
    	else if(projection==0 && rotation>0){
    		bestAction = Controller.ACTION_NO_LEFT;
    		System.out.println("ACTION_NO_LEFT" );
    	}
    	else if(projection==0 && rotation<0){
    		bestAction = Controller.ACTION_NO_RIGHT;
    		System.out.println("ACTION_NO_RIGHT" );
    	}
    	return bestAction;
    }
    
    private int OutputFiltering(Vector2d acceleration,Game a_gameCopy,Vector2d ND)
    {
    	double bestAcceleration = -2;
    	int bestAction=Controller.ACTION_NO_FRONT;;
    	for(int action = Controller.ACTION_NO_FRONT; action <= Controller.NUM_ACTIONS; ++action)
    	{
    		Game forThisAction = a_gameCopy.getCopy();
    		forThisAction.getShip().update(action);
                
    		Vector2d UD = forThisAction.getShip().d;
    		double newAcceleration = UD.scalarProduct(ND);
    		if(newAcceleration > bestAcceleration)
    		{
    			bestAcceleration = newAcceleration;
    			bestAction = action;
    		}
         }
    	return bestAction;
    }
    
    private int OutputFiltering(Game a_gameCopy)
    {
    	double minDistance = Double.MAX_VALUE;
        int bestAction = Integer.MIN_VALUE;
    	double bestAcceleration = -2;
        for(int action = Controller.ACTION_NO_FRONT; action < Controller.NUM_ACTIONS; ++action)
        {
            //Simulate that we execute the action and get my potential position and direction
            Game forThisAction = a_gameCopy.getCopy();
            forThisAction.getShip().update(action);
            Vector2d NP = forThisAction.getShip().s;
            Vector2d UD = forThisAction.getShip().d;

            //seek behavior to the next node
            Node nextNode = getNextNode();
            Vector2d ND = new Vector2d(nextNode.x(),nextNode.y());
            ND.subtract(NP);
        	ND.normalise();
        	double Acceleration = UD.scalarProduct(ND);
            
            //Get the distance to the next node in the tree and update the total distance until the closest waypoint
            double dist = nextNode.euclideanDistanceTo(NP.x, NP.y);
            double totalDistance = PathToWaypoint.m_cost + dist;

            //Keep the best action so far.
            if(totalDistance < minDistance)
            {
                minDistance = totalDistance;
                bestAction = action;
                bestAcceleration = Acceleration;
            }
            //If the distance is the same, keep the action that faces the ship more towards the next node
            else if((int)totalDistance == (int)minDistance && Acceleration > bestAcceleration)
            {
                minDistance = totalDistance;
                bestAction = action;
                bestAcceleration = Acceleration;
            }
        }
        return bestAction;
    }
    
}