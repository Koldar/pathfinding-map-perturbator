#include "globals.hpp"
#include "CLI11.hpp"

#include <pathfinding-map-perturbator-utils/utils.hpp>
#include <pathfinding-map-perturbator-utils/AbstractPerturbator.hpp>
#include <pathfinding-map-perturbator-utils/RandomPerturbator.hpp>

using namespace pathfinding;
using namespace pathfinding::maps;
using namespace pathfinding::map_perturbator::utils;

int main(int argc, char* args[]) {
    
    Globals globals;

	CLI::App app{"pathfinding-map-perturbator"};

	// global_perturbationAreaRadiusStr=std::string{"[3,3]"};
	// global_perturbationDensityStr=std::string{};


	app.add_option("--map-filename", globals.mapPath,
			"The file where it is contained the map"
	)->required();
    app.add_option("--perturbation-kind", globals.type,
		"Tells how you want to perturbate the map. Possible values are:"
		"RANDOM (generate random perturbations); "
	)->required();
	app.add_option("--random-seed", globals.randomSeed,
		"the seed for any pseudo random generator."
	)->required();
	app.add_option("--output-main-directory", globals.outputMainDirectory,
		"The directory every MAIN output file will be positioned into. Main output files are:"
		" - place where we will generate perturbated map"
	)->required();
    app.add_option("--query-per-perturbated-map", globals.queryPerPerturbatedMap,
        "number of pathfinding query we need to execute per each perturbated map we create"
    )->required();
    app.add_option("--number-of-perturbation-pystring", globals.numberOfPerturbationPyString,
			"Represents the number of edges to perturbate. You can use the following variables to perform operations:"
			"V: number of vertices in the graph"
			"E: number of undirected edges in the graph"
			"Semantically meaningful for RANDOM"
	);
    app.add_option("--perturbation-range-pystring", globals.perturbationRangePyString,
			"If -p is enabled, the range of the perturbation of the weights arcs. This is a math range (e.globals., [4,100[ or (4,10) )"
			"Semantically meaningful for RANDOM"
	);
    app.add_flag("--include-infinity", globals.includeInfinite,
        "If true, we will sometimes use infinity to perutrbate an arc"
    );
    app.add_option("--infinity-probability-pystring", globals.infinityProbabilityPyString,
        "If --include-infinity is declared, the probability we use to generate infinity"
    );
    

    CLI11_PARSE(app, argc, args);
    
    // ********************************************************************
	// *********************** SET DERIVED GLOBAL VALUES ******************
	// ********************************************************************

	globals.randomGenerator = Random{globals.randomSeed};
	globals.mapName = globals.mapPath.stem().string();

	// ********************************************************************
	// *********************** CREATE VALUE *******************************
	// ********************************************************************

	// LOAD THE MAP
	info("map involved is ", globals.mapPath);
	auto baseMapGraph = loadMap(globals.mapPath);

    // ********************************************************************
	// ********************* PERTURBATOR **********************************
	// ********************************************************************

	std::unique_ptr<AbstractPerturbator<std::string, xyLoc>> perturbator{nullptr};
	if (globals.type == std::string{"RANDOM"}) {
		//RANDOM
		uint32_t edgesToAlter = callPyEvalAndCastNumberTo<uint32_t>(globals.numberOfPerturbationPyString,
			"V", baseMapGraph.numberOfVertices(),
			"E", baseMapGraph.numberOfEdges()
		);

        auto edgesToAlterString = callPyEvalWithEval(globals.perturbationRangePyString,
			"V", baseMapGraph.numberOfVertices(),
			"E", baseMapGraph.numberOfEdges()
		);

		cpp_utils::Interval<int> perturbationRange = cpp_utils::Interval<int>::fromMath(edgesToAlterString);

		perturbator = std::unique_ptr<AbstractPerturbator<std::string, xyLoc>>{new RandomPerturbator<std::string, xyLoc>{
            true,
			edgesToAlter, perturbationRange, 
			"MULTIPLY", 
			globals.randomGenerator
		}};
	} else {
		throw cpp_utils::exceptions::InvalidScenarioException<std::string>{globals.type};
	}

    // ***********************************************************************************
	// ********************* CREATE PERTURBATED QUERIES **********************************
	// ***********************************************************************************

    GridMapScenarioExperiment gridMapScenarioExperiment{0, 0, 0, 0, 0, 0, ""};

    for (int perturbatedMapId=0; perturbatedMapId<globals.numberOfPerturbatedMaps; ++perturbatedMapId) {
        critical("generating perturbated map #%04ld", perturbatedMapId);

        auto perturbatedGraph = perturbator->perturbateGraph(
            perturbatedMapId,
            gridMapScenarioExperiment, //dummy grid map scenario experiment
            0, //dummy start node
            0, //dummy goal node
            baseMapGraph
        );

        // create a number of pathfinding queries and solve them
        for (int queryId=0; queryId<globals.queryPerPerturbatedMap; ++queryId) {
            auto startVertex = perturbatedGraph->getRandomVertex();
            auto goalVertex = perturbatedGraph->getRandomVertex();

            //solve 
        }
    }
}