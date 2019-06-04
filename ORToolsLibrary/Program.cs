using System;
using Google.OrTools.ConstraintSolver;
using Google.Protobuf.WellKnownTypes;

namespace ORTools
{
    class Program
    {
        static void Main(string[] args)
        {
            TimeConstrainedVRP.Execute();
        }
    }

    class TimeDataModel
    {
        // Time between two locations
        public long[, ] TimeMatrix = { { 0, 6, 9, 8, 7, 3, 6, 2, 3, 2, 6, 6, 4, 4, 5, 9, 7 },
            { 6, 0, 8, 3, 2, 6, 8, 4, 8, 8, 13, 7, 5, 8, 12, 10, 14 },
            { 9, 8, 0, 11, 10, 6, 3, 9, 5, 8, 4, 15, 14, 13, 9, 18, 9 },
            { 8, 3, 11, 0, 1, 7, 10, 6, 10, 10, 14, 6, 7, 9, 14, 6, 16 },
            { 7, 2, 10, 1, 0, 6, 9, 4, 8, 9, 13, 4, 6, 8, 12, 8, 14 },
            { 3, 6, 6, 7, 6, 0, 2, 3, 2, 2, 7, 9, 7, 7, 6, 12, 8 },
            { 6, 8, 3, 10, 9, 2, 0, 6, 2, 5, 4, 12, 10, 10, 6, 15, 5 },
            { 2, 4, 9, 6, 4, 3, 6, 0, 4, 4, 8, 5, 4, 3, 7, 8, 10 },
            { 3, 8, 5, 10, 8, 2, 2, 4, 0, 3, 4, 9, 8, 7, 3, 13, 6 },
            { 2, 8, 8, 10, 9, 2, 5, 4, 3, 0, 4, 6, 5, 4, 3, 9, 5 },
            { 6, 13, 4, 14, 13, 7, 4, 8, 4, 4, 0, 10, 9, 8, 4, 13, 4 },
            { 6, 7, 15, 6, 4, 9, 12, 5, 9, 6, 10, 0, 1, 3, 7, 3, 10 },
            { 4, 5, 14, 7, 6, 7, 10, 4, 8, 5, 9, 1, 0, 2, 6, 4, 8 },
            { 4, 8, 13, 9, 8, 7, 10, 3, 7, 4, 8, 3, 2, 0, 4, 5, 6 },
            { 5, 12, 9, 14, 12, 6, 6, 7, 3, 3, 4, 7, 6, 4, 0, 9, 2 },
            { 9, 10, 18, 6, 8, 12, 15, 8, 13, 9, 13, 3, 4, 5, 9, 0, 9 },
            { 7, 14, 9, 16, 14, 8, 5, 10, 6, 5, 4, 10, 8, 6, 2, 9, 0 },
        };

        public long[, ] TimeWindows = { { 0, 5 }, // depot
            { 7, 12 }, // 1
            { 10, 15 }, // 2
            { 16, 18 }, // 3
            { 10, 13 }, // 4
            { 0, 5 }, // 5
            { 5, 10 }, // 6
            { 0, 4 }, // 7
            { 5, 10 }, // 8
            { 0, 3 }, // 9
            { 10, 16 }, // 10
            { 10, 15 }, // 11
            { 0, 5 }, // 12
            { 5, 10 }, // 13
            { 7, 8 }, // 14
            { 10, 15 }, // 15
            { 11, 15 }, // 16
        };
        public int VehicleNumber = 4;
        public int Depot = 0;
    }

    public class TimeConstrainedVRP
    {
        private const string TIME_DIMENSION_NAME = "Time";

        static void PrintSolution(in TimeDataModel data, in RoutingModel routing, in RoutingIndexManager manager, in Assignment solution)
        {
            RoutingDimension timeDimension = routing.GetMutableDimension(TIME_DIMENSION_NAME);
            // Inspect solution.
            long totalTime = 0;
            for (int i = 0; i < data.VehicleNumber; ++i)
            {
                Console.WriteLine("Route for Vehicle {0}:", i);
                var index = routing.Start(i);
                while (!routing.IsEnd(index))
                {
                    var timeVar = timeDimension.CumulVar(index);
                    Console.Write("{0} Time({1},{2}) -> ",
                        manager.IndexToNode(index),
                        solution.Min(timeVar),
                        solution.Max(timeVar));
                    index = solution.Value(routing.NextVar(index));
                }
                var endTimeVar = timeDimension.CumulVar(index);
                Console.WriteLine("{0} Time({1},{2})",
                    manager.IndexToNode(index),
                    solution.Min(endTimeVar),
                    solution.Max(endTimeVar));
                Console.WriteLine("Time of the route: {0}min", solution.Min(endTimeVar));
                totalTime += solution.Min(endTimeVar);
            }
            Console.WriteLine("Total time of all routes: {0}min", totalTime);
        }

        private static RoutingDimension SetTimeDimension(
            ref TimeDataModel data, in RoutingModel routing, in RoutingIndexManager manager, in int transitCallbackIndex)
        {
            routing.AddDimension(
                transitCallbackIndex,
                // Since a vehicule may have to wait for a place to open, we allow the vehicule to wait at 
                // a location before moving to the next
                30, // Slack-max: allows slack time up to 30, because of the time window constraints
                30, // Capacity: Maximum of accumulated time               
                false, // Start-cumul can start from number different from 0
                TIME_DIMENSION_NAME
            );
            RoutingDimension timeDimension = routing.GetMutableDimension(TIME_DIMENSION_NAME);

            // Add time window constraints for each location except depot (first index).
            for (int i = 1; i < data.TimeWindows.GetLength(0); i++)
            {
                // checks every node in graph
                long index = manager.NodeToIndex(i);
                // The cumulative variable at i has to be inbetween the TimeWindows(i)=[x,y]
                timeDimension.CumulVar(index).SetRange(
                    data.TimeWindows[i, 0],
                    data.TimeWindows[i, 1]
                );
            }

            // Add time window constraints for each vehicle start node.
            for (int i = 0; i < data.VehicleNumber; ++i)
            {
                // checks every route
                long index = routing.Start(i);
                // the first cumul index has to be in the opened hours
                timeDimension.CumulVar(index).SetRange(
                    data.TimeWindows[0, 0],
                    data.TimeWindows[0, 1]);
            }
            for (int i = 0; i < data.VehicleNumber; ++i)
            {
                routing.AddVariableMinimizedByFinalizer(
                    timeDimension.CumulVar(routing.Start(i)));
                routing.AddVariableMinimizedByFinalizer(
                    timeDimension.CumulVar(routing.End(i)));
            }
            return timeDimension;
        }

        public static void Execute()
        {
            // 1. Create the data and the routing model. Also creates the routing index manager.
            TimeDataModel data = new TimeDataModel();
            RoutingIndexManager manager = new RoutingIndexManager(
                data.TimeMatrix.GetLength(0),
                data.VehicleNumber,
                data.Depot
            );
            RoutingModel routing = new RoutingModel(manager);

            // 2. Create the callback to calculate the distance between two nodes
            int transitCallbackIndex = routing.RegisterTransitCallback((long fromIndex, long toIndex) =>
            {
                // Convert from routing variable Index to distance matrix NodeIndex.
                var fromNode = manager.IndexToNode(fromIndex);
                var toNode = manager.IndexToNode(toIndex);
                return data.TimeMatrix[fromNode, toNode];
            });
            routing.SetArcCostEvaluatorOfAllVehicles(transitCallbackIndex);

            // 3. Describe another dimension to consider the cumulative time + transit time 
            RoutingDimension timeDimension = SetTimeDimension(ref data, routing, manager, transitCallbackIndex);

            // 4. Set the search strategy to be used in the graph
            RoutingSearchParameters searchParams = operations_research_constraint_solver.DefaultRoutingSearchParameters();
            searchParams.FirstSolutionStrategy = FirstSolutionStrategy.Types.Value.PathCheapestArc;
            searchParams.LnsTimeLimit = new Duration { Seconds = 20 };
            searchParams.TimeLimit = new Duration { Seconds = 20 };

            // 5. Start the solver with the defined params (sync) and print the solution
            Assignment solution = routing.SolveWithParameters(searchParams);
            PrintSolution(data, routing, manager, solution);
        }
    }
}