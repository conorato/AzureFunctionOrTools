using System;
using System.Collections.Generic;
using Google.OrTools.ConstraintSolver;

/// <summary>
///   Minimal Pickup & Delivery Problem (PDP).
/// </summary>
public class OrToolsBug
{
    class DataModel
    {
        public long[, ] DistanceMatrix = {
            { 0, 100, 100, 80, 100 },
            { 100, 0, 80, 100, 100 },
            { 100, 100, 0, 100, 80 },
            { 100, 80, 100, 0, 100 },
            { 100, 100, 100, 100, 0 },
        };
        public int[][] PickupsDeliveries = {
            new int[] { 1, 2 },
            new int[] { 3, 4 },
            new int[] { 4, 1 },
        };

        //public int[][] PickupsDeliveries = {
        //   new int[] {1, 2},
        //   new int[] {2, 3},
        //   new int[] {3, 4},
        // };

        public int VehicleNumber = 1;
        public int Depot = 0;
    };

    /// <summary>
    ///   Print the solution.
    /// </summary>
    static void PrintSolution(DataModel data, RoutingModel routing, RoutingIndexManager manager, Assignment solution)
    {
        long totalDistance = 0;
        for (int i = 0; i < data.VehicleNumber; ++i)
        {
            Console.WriteLine("Route for Vehicle {0}:", i);
            long routeDistance = 0;
            var index = routing.Start(i);
            while (routing.IsEnd(index) == false)
            {
                Console.Write("{0} -> ", manager.IndexToNode((int)index));
                var previousIndex = index;
                index = solution.Value(routing.NextVar(index));
                routeDistance += routing.GetArcCostForVehicle(previousIndex, index, 0);
            }
            Console.WriteLine("{0}", manager.IndexToNode((int)index));
            Console.WriteLine("Distance of the route: {0}m", routeDistance);
            totalDistance += routeDistance;
        }
        Console.WriteLine("Total Distance of all routes: {0}m", totalDistance);
    }

    public static void Main(String[] args)
    {
        // Instantiate the data problem.
        DataModel data = new DataModel();

        // Create Routing Index Manager
        RoutingIndexManager manager = new RoutingIndexManager(
            data.DistanceMatrix.GetLength(0),
            data.VehicleNumber,
            data.Depot);

        // Create Routing Model.
        RoutingModel routing = new RoutingModel(manager);

        // Create and register a transit callback.
        int transitCallbackIndex = routing.RegisterTransitCallback(
            (long fromIndex, long toIndex) =>
            {
                // Convert from routing variable Index to distance matrix NodeIndex.
                var fromNode = manager.IndexToNode(fromIndex);
                var toNode = manager.IndexToNode(toIndex);
                return data.DistanceMatrix[fromNode, toNode];
            }
        );

        // Define cost of each arc.
        routing.SetArcCostEvaluatorOfAllVehicles(transitCallbackIndex);

        // Add Distance constraint.
        routing.AddDimension(
            transitCallbackIndex,
            0,
            3000,
            true, // start cumul to zero
            "Distance");
        RoutingDimension distanceDimension = routing.GetMutableDimension("Distance");
        distanceDimension.SetGlobalSpanCostCoefficient(100);

        // Define Transportation Requests.
        Solver solver = routing.solver();
        for (int i = 0; i < data.PickupsDeliveries.GetLength(0); i++)
        {
            long pickupIndex = manager.NodeToIndex(data.PickupsDeliveries[i][0]);
            long deliveryIndex = manager.NodeToIndex(data.PickupsDeliveries[i][1]);
            routing.AddPickupAndDelivery(pickupIndex, deliveryIndex);
            solver.Add(solver.MakeEquality(
                routing.VehicleVar(pickupIndex),
                routing.VehicleVar(deliveryIndex)));
            solver.Add(solver.MakeLessOrEqual(
                distanceDimension.CumulVar(pickupIndex),
                distanceDimension.CumulVar(deliveryIndex)));
        }

        // Setting first solution heuristic.
        RoutingSearchParameters searchParameters =
            operations_research_constraint_solver.DefaultRoutingSearchParameters();
        searchParameters.FirstSolutionStrategy =
            FirstSolutionStrategy.Types.Value.PathCheapestArc;

        // Solve the problem.
        Assignment solution = routing.SolveWithParameters(searchParameters);

        // Print solution on console.
        PrintSolution(data, routing, manager, solution);
    }
}