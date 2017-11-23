#include <discorde_tsp/discorde_cpp.h>
#include <cstdio>

extern "C"{
#include <mod_concorde/concorde.h>
}

int discorde::concorde(int n_nodes, std::vector< int >& edges, std::vector< int >& edges_costs,
        std::vector< int >& out_tour, double& out_cost, int* out_status, std::vector< int > in_tour,
        bool verbose, double time_limit)
{
    out_tour.resize( n_nodes );

    /* Variables and structures used by Concorde solver */
    int success;        /* Output flag: set to 1 if a feasible tour is found */
    int optimal;        /* Output flag: set to 1 if tour found is optimal */
    int hit_timelimit;  /* Output flag: set to 1 if the time limit is reached */
    CCrandstate rstate; /* Rand state structure defined by Concorde library */

    /* Initialize Concorde structures */
    CCutil_sprand(rand(), &rstate);

    /* Set a name for temporary files created by Concorde */
    char filename[ L_tmpnam ];     /* Name of temporary files created by Concorde */
    if( !std::tmpnam( filename ) ) {
      return DISCORDE_RETURN_FAILURE;
    }

    /* Call Concorde solver */
    int n_edges = static_cast< int >( edges.size() );
    CCtsp_solve_sparse(n_nodes, n_edges, edges.data(), edges_costs.data(), in_tour.empty() ? NULL : in_tour.data(),
            out_tour.data(), NULL, &out_cost, &optimal, &success, filename, (time_limit == -1. ) ? NULL : &time_limit,
            &hit_timelimit, !verbose, &rstate);

    /* Set solver status */
    if (out_status != NULL) {
        if (success == 1 && optimal != 0) {
            *out_status = DISCORDE_STATUS_OPTIMAL;
        } else if (hit_timelimit == 1) {
            *out_status = DISCORDE_STATUS_TIMELIMIT;
        } else {
            *out_status = DISCORDE_STATUS_UNKNOWN;
        }
    }

    /* Set the return value */
    if (success == 1) {
        return DISCORDE_RETURN_OK;
    } else {
        return DISCORDE_RETURN_FAILURE;
    }
}

int discorde::linkernighan(int n_nodes, std::vector< int >& edges,
        std::vector< int >& edges_costs, std::vector< int >& out_tour, double& out_cost, std::vector< int > in_tour,
        bool verbose, double time_limit, double target)
{
    out_tour.resize( n_nodes );

    /* Variables and structures used by Lin-Kernighan heuristic */
    CCrandstate rstate;      /* Rand state structure (in Concorde library) */
    CCdatagroup data;        /* Coordinate data defined by Concorde library */
    int stallcount = 100000000;          /* Maximum number of 4-swaps without progress */
    int repeatcount = -1;         /* Number of 4-swap kicks */

    /*
     * Available values for type of kick are:
     *  - CC_LK_RANDOM_KICK
     *  - CC_LK_GEOMETRIC_KICK
     *  - CC_LK_CLOSE_KICK
     *  - CC_LK_WALK_KICK
     */
    int kicktype = CC_LK_RANDOM_KICK;            /* Type of kick */

    /* Initialize Lin-Kernighan structures */
    CCutil_sprand(rand(), &rstate);

    /* Initialize the coordinate data */
    CCutil_init_datagroup (&data);
    int n_edges = static_cast< int >( edges.size() );
    CCutil_graph2dat_sparse(n_nodes, n_edges, edges.data(), edges_costs.data(), 0, &data);

    /* Call Lin-Kernighan heuristic */
    CClinkern_tour(n_nodes, &data, n_edges, edges.data(), stallcount, repeatcount,
            in_tour.empty() ? NULL : in_tour.data(), out_tour.data(), &out_cost, !verbose, time_limit, target,
            NULL, kicktype, &rstate);

    /* Set return value */
    return DISCORDE_RETURN_OK;
}

