#include "globals.hpp"
#include "CLI11.hpp"

#include <iostream>
#include <sstream>

#include <boost/smart_ptr.hpp>
#include <boost/make_unique.hpp>

#include <cpp-utils/vectorplus.hpp>
#include <cpp-utils/CSVWriter.hpp>

#include <pathfinding-map-perturbator-utils/utils.hpp>
#include <pathfinding-map-perturbator-utils/AbstractPerturbator.hpp>
#include <pathfinding-map-perturbator-utils/RandomPerturbator.hpp>
#include <pathfinding-utils/ALTHeuristic.hpp>
#include <pathfinding-utils/DifferentHeuristicAdvancePlacingLandmarkStrategy.hpp>
#include <pathfinding-utils/GraphState.hpp>
#include <pathfinding-utils/StandardStateExpander.hpp>
#include <pathfinding-utils/IStatePruner.hpp>
#include <pathfinding-utils/StandardLocationGoalChecker.hpp>
#include <pathfinding-utils/AStarAllSolutions.hpp>
#include <pathfinding-utils/NeverPrune.hpp>
#include "SolutionPerNodeListener.hpp"


using namespace cpp_utils;
using namespace pathfinding;
using namespace pathfinding::search;
using namespace pathfinding::maps;
using namespace pathfinding::map_perturbator::utils;

template <typename G, typename V>
static void solveQuery(int perturbatedMapId, int queryId, const IImmutableGraph<G,V,cost_t>& perturbatedGraph, nodeid_t start, nodeid_t goal, const Globals& globals, int& numberOfSolutions, const IImmutableGraph<G,int,int>*& countGraph, cost_t& optimalSolutionCost, std::string& solutionExample);
template <typename G, typename V>
static GridMapImage* generatePerturbatedImage(int perturbatedMapId, const GridMap& gridMap, const IImmutableGraph<G, V, cost_t>& baseMapGraph, const IImmutableGraph<G, V, PerturbatedCost>& perturbatedGraph);

static boost::filesystem::path getOutputOriginalJsonPath(const Globals& globals);
static boost::filesystem::path getOutputPerturbatedJsonPath(int perturbatedGraphId, const Globals& globals);
static std::string getSolutionString(const ISolutionPath<const GraphState<std::string, xyLoc, cost_t>*, const GraphState<std::string, xyLoc, cost_t>&>& solution);
static std::string getVertexDistribution(const pathfinding::maps::GridMap& gridMap, const IImmutableGraph<std::string, xyLoc, cost_t>& baseMap, const IImmutableGraph<std::string, int, int>& countGraph);

static boost::filesystem::path getLandmarkFilename(int perturbatedGraphId, const Globals& globals);
static boost::filesystem::path getOutputCSVPath(int perturbatedGraphId, const Globals& globals);

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
    app.add_option("--output-temp-directory", globals.outputMainDirectory,
		"The directory every TEMP output file will be positioned into. Main output files are:"
		" - place where we will generate perturbated map"
	)->required();
    app.add_option("--query-per-perturbated-map", globals.queryPerPerturbatedMap,
        "number of pathfinding query we need to execute per each perturbated map we create"
    )->required();
    app.add_option("--number-of-perturbated-map-pystring", globals.numberOfPerturbatedMapsPyString,
			"Represents the number of times we need to perturbate the original map."
            "V: number of vertices in the graph"
			"E: number of undirected edges in the graph"
	);
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
    app.add_option("--landmark-path-pystring", globals.landmarkPathPyString,
        "Name of the file containing the database of a perturbated graph"
        " - MAPNAME: name of the original map which was perturbated"
        " - PERTURBATEDMAPID: id of the perturbated map generated by the application"
    )->required();
    app.add_option("--output-main-csv-pystring", globals.otuputMainCsvFilenamePyString,
        "Name of the file CSV containing information about the solutions"
        " - PERTURBATEDMAPID: id of the perturbated map generated by the application"
        " - MAPNAME: name of the original map which was perturbated"
    )->required();
    app.add_option("--output-json-map-pystring", globals.outputMainJsonMapFilenamePyString,
        "Name of the file json containing a json representing the perturbated map"
        " - PERTURBATEDMAPID: id of the perturbated map generated by the application"
        " - MAPNAME: name of the original map which was perturbated"
    )->required();
    app.add_option("--output-json-original-map-pystring", globals.outputMainJsonOriginalMapFilenamePyString,
        "Name of the file json containing a json representing the original map"
        " - MAPNAME: name of the original map which was perturbated"
    )->required();
    

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
    MovingAIGridMapReader reader{'.', 1000, color_t::WHITE};
    reader.addTerrain('T', 1500, color_t::GREEN);
	reader.addTerrain('S', 2000, color_t::CYAN);
	reader.addTerrain('W', 2500, color_t::BLUE);
	reader.addTerrain('@', cost_t::INFTY, color_t::BLACK);
	pathfinding::maps::GridMap gridMap = reader.load(globals.mapPath);
	GridMapGraphConverter converter{GridBranching::EIGHT_CONNECTED};
	auto baseMapGraph = AdjacentGraph<std::string, xyLoc, cost_t>(*converter.toGraph(gridMap));

    // dump original map in a JSON
    {
        std::string jsonString{converter.toJson(baseMapGraph)};
        boost::filesystem::path jsonPerturbatedMapPath{getOutputOriginalJsonPath(globals)};
        std::ofstream f;
        f.open(jsonPerturbatedMapPath.native());
        f << jsonString;
        f.close();
    }

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

		cpp_utils::Interval<double> perturbationRange = cpp_utils::Interval<double>::parseDoubleInterval(edgesToAlterString);

        critical("perturbation is ", perturbationRange);
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

    int numberOfPerturbatedMaps = cpp_utils::callPyEvalAndCastNumberTo<int>(globals.numberOfPerturbatedMapsPyString,
        "V", baseMapGraph.numberOfVertices(),
        "E", baseMapGraph.numberOfEdges()
    );

    critical("we need to generate ", numberOfPerturbatedMaps, "perturbated maps");
    for (int perturbatedMapId=0; perturbatedMapId<numberOfPerturbatedMaps; ++perturbatedMapId) {
        critical("generating perturbated map", perturbatedMapId, "out of", numberOfPerturbatedMaps);

        boost::filesystem::path csvOutputPath{getOutputCSVPath(perturbatedMapId, globals)};

        //check if we need to generate the map
        if (boost::filesystem::exists(csvOutputPath)) {
            warning("csv output file ", csvOutputPath, "exists. Do nothing");
            continue;
        }

        perturbator->cleanup();
        auto p = perturbator->perturbateGraph(
            perturbatedMapId,
            gridMapScenarioExperiment, //dummy grid map scenario experiment
            0, //dummy start node
            0, //dummy goal node
            baseMapGraph
        );
        auto perturbatedGraph = std::unique_ptr<IImmutableGraph<std::string, xyLoc, PerturbatedCost>>{p};
        critical("we have perturbated", perturbator->getEdgeChanged(), "edges!");

        auto perturbatedGraphWithCost = perturbatedGraph->mapEdges(std::function<cost_t(const PerturbatedCost&)>{[&](const PerturbatedCost& pe) { return pe.getCost();}});

        //generate the image of the perturbated map
        auto perturbatedImage = generatePerturbatedImage(perturbatedMapId, gridMap, baseMapGraph, *perturbatedGraph);

        //dump the map in a JSON
        {
            std::string jsonString{converter.toJson(*perturbatedGraphWithCost)};
            boost::filesystem::path jsonPerturbatedMapPath{getOutputPerturbatedJsonPath(perturbatedMapId, globals)};
            std::ofstream f;
            f.open(jsonPerturbatedMapPath.native());
            f << jsonString;
            f.close();
        }


        //create one CSV per perturbatedMapId
        
        CSVWriter<int, int, ucood_t, ucood_t, ucood_t, ucood_t, cost_t, int, std::string, std::string, std::string> mainCSVWriter{csvOutputPath, ',', vectorplus<std::string>{
            "perturbatedMapId", 
            "queryId",
            "startx",
            "starty",
            "goalx",
            "goaly",
            "optimalSolutionCost",
            "numberOfSolutionsFound"
            "lastSolutionFound",
            "edgeDistribution",
            "vertexDistribution",
            }};

        // create a number of pathfinding queries and solve them
        for (int queryId=0; queryId<globals.queryPerPerturbatedMap; ++queryId) {
            auto startVertex = perturbatedGraph->getRandomVertex();
            auto goalVertex = perturbatedGraph->getRandomVertex();

            auto startLoc = perturbatedGraph->getVertex(startVertex);
            auto goalLoc = perturbatedGraph->getVertex(goalVertex);

            //solve instance
            try {

                int numberOfSolutions;
                const IImmutableGraph<std::string, int, int>* countGraph = nullptr;
                cost_t optimalSolutionCost;
                std::string solutionString;

                //the line create a new graph and puts it in countGraph
                solveQuery(
                    perturbatedMapId, queryId, 
                    *perturbatedGraphWithCost, 
                    startVertex, goalVertex, 
                    globals, 
                    numberOfSolutions, countGraph, optimalSolutionCost, solutionString
                );

                //DO NOT USE aStarOutput ANYMORE!

                mainCSVWriter.writeRow(
                    perturbatedMapId,
                    queryId,
                    startLoc.x,
                    startLoc.y,
                    goalLoc.x,
                    goalLoc.y,
                    optimalSolutionCost,
                    numberOfSolutions,
                    solutionString,
                    getVertexDistribution(gridMap, baseMapGraph, *countGraph),
                    getVertexDistribution(gridMap, baseMapGraph, *countGraph)
                );

                // GridMapImage* perturbatedImageWithLocations = new GridMapImage{*perturbatedImage};
                // for (auto it=countGraph->beginVertices(); it!=countGraph->endVertices(); ++it) {
                //     xyLoc loc = it->second;
                //     perturbatedImageWithLocations->lerpGridCellColor(loc, color_t::YELLOW);
                // }
                // perturbatedImageWithLocations->saveBMP(scout("perturbated-with-locatons-", perturbatedMapId, "-", queryId));
                // delete perturbatedImageWithLocations;

                delete countGraph;
            } catch (SolutionNotFoundException e) {
                mainCSVWriter.writeRow(
                    perturbatedMapId,
                    queryId,
                    startLoc.x,
                    startLoc.y,
                    goalLoc.x,
                    goalLoc.y,
                    0,
                    0,
                    "[]",
                    "[]",
                    "[]"
                );
            }
        }

        delete perturbatedImage;
    }
}

template <typename G, typename V>
static GridMapImage* generatePerturbatedImage(int perturbatedMapId, const GridMap& gridMap, const IImmutableGraph<G, V, cost_t>& baseMapGraph, const IImmutableGraph<G, V, PerturbatedCost>& perturbatedGraph) {
    auto image = std::unique_ptr<GridMapImage>(const_cast<GridMapImage*>(static_cast<const GridMapImage*>(gridMap.getPPM())));
    for (auto it=perturbatedGraph.beginEdges(); it!=perturbatedGraph.endEdges(); ++it) {
        if (!it->getPayload().isPerturbated()) {
            continue;
        }
        xyLoc source = perturbatedGraph.getVertex(it->getSourceId());
        xyLoc sink = perturbatedGraph.getVertex(it->getSinkId());
        int pixel1x, pixel1y, pixel2x, pixel2y;
        switch (xyLoc::getDirection(source, sink)) {
            case Direction::SOUTHEAST:  pixel1x=2; pixel1y=2; pixel2x=0; pixel2y=0; break;
            case Direction::EAST:       pixel1x=2; pixel1y=1; pixel2x=0; pixel2y=1; break;
            case Direction::NORTHEAST:  pixel1x=2; pixel1y=0; pixel2x=0; pixel2y=2; break;

            case Direction::SOUTHWEST:  pixel1x=0; pixel1y=2; pixel2x=2; pixel2y=0; break;
            case Direction::WEST:       pixel1x=0; pixel1y=1; pixel2x=2; pixel2y=1; break;
            case Direction::NORTHWEST:  pixel1x=0; pixel1y=0; pixel2x=2; pixel2y=2; break;
            
            case Direction::SOUTH:      pixel1x=1; pixel1y=2; pixel2x=1; pixel2y=0; break;
            case Direction::NORTH:      pixel1x=1; pixel1y=0; pixel2x=1; pixel2y=2; break;
            default:
                throw cpp_utils::exceptions::makeImpossibleException("invalid direction!");
        }
        cost_t oldWeight = baseMapGraph.getEdge(it->getSourceId(), it->getSinkId());
        cost_t newWeight = perturbatedGraph.getEdge(it->getSourceId(), it->getSinkId()).getCost();
        color_t perturbatedColor;
        if (newWeight > oldWeight) {
            //cost has increased
            perturbatedColor = color_t::RED;
        } else {
            //cost has decreased
            perturbatedColor = color_t::BLUE;
        }
        image->setPixelInGrid(source.x, source.y, pixel1x, pixel1y, perturbatedColor); 
        image->setPixelInGrid(sink.x, sink.y, pixel2x, pixel2y, perturbatedColor);
    }
    image->saveBMP(scout("hello-", perturbatedMapId));

    return image.release();
}

static std::string getVertexDistribution(const pathfinding::maps::GridMap& gridMap, const IImmutableGraph<std::string, xyLoc, cost_t>& baseMap, const IImmutableGraph<std::string, int, int>& countGraph) {
    std::stringstream ss;

    ss << "[";
    for (ucood_t y=0; y<gridMap.getHeight(); ++y) {
        for (ucood_t x=0; x<gridMap.getWidth(); ++x) {
            xyLoc position{x, y};
            if (gridMap.isTraversable(position)) {
                nodeid_t id = baseMap.idOfVertex(position);
                ss << countGraph.getVertex(id);
            } else {
                ss << "-";
            }

            if ((x+1) < gridMap.getWidth()) {
                ss << "|";
            }
        }
        ss <<"@";
    }
    ss <<"]";

    return ss.str();
}


static std::string getSolutionString(const ISolutionPath<const GraphState<std::string, xyLoc, cost_t>*, const GraphState<std::string, xyLoc, cost_t>&>& solution) {
    const GraphState<std::string, xyLoc, cost_t>* tmp = &solution.getGoal();

    std::stringstream ss;

    ss << "[";
    for (int i=0; i<solution.size(); ++i) {
        nodeid_t idInvolved = solution.at(i)->getId();
        xyLoc loc = solution.at(i)->getGraph().getVertex(idInvolved);
        ss << "{" << loc.x << "|" << loc.y << "}";
        if ((i + 1) < solution.size()) {
            ss << "@";
        }
    }
    ss << "]";

    return ss.str();
}

template <typename G, typename V>
static void solveQuery(int perturbatedMapId, int queryId, const IImmutableGraph<G,V,cost_t>& perturbatedGraph, nodeid_t start, nodeid_t goal, const Globals& globals, int& numberOfSolutions, const IImmutableGraph<G,int,int>*& countGraph, cost_t& optimalSolutionCost, std::string& solutionExample) {
    //create ALT database: we will use it to solve each query
    auto landmarkDatabaseName = getLandmarkFilename(perturbatedMapId, globals);
    info("landmark database (if needs to be used) of", globals.mapName, "(available at ", landmarkDatabaseName, ")");	
    auto landmarkDatabasePath = boost::filesystem::path{globals.outputTempDirectory} / landmarkDatabaseName;

    DifferentHeuristicAdvancePlacingLandmarkStrategy<std::string, xyLoc> landmarkStrategy{6};
    LandmarkDatabase<std::string, xyLoc> landmarkDatabase{LandmarkDatabase<std::string, xyLoc>::fetchOrCompute(perturbatedGraph, landmarkStrategy, landmarkDatabasePath)};

    GraphStateSupplier<G, V, cost_t> supplier{perturbatedGraph};
    StandardStateExpander<GraphState<G, V, cost_t>, G, V, cost_t> stateExpander{perturbatedGraph};
    
    ALTHeuristic<GraphState<G, V, cost_t>, std::string, V> heuristic{perturbatedGraph, landmarkDatabase};
    NeverPrune<GraphState<G, V, cost_t>> pruner{};
    StandardLocationGoalChecker<GraphState<G, V, cost_t>> goalChecker{};
    SolutionPerNodeListener<G, GraphState<G, V, cost_t>> listener{perturbatedGraph};

    AStarAllSolutions<GraphState<G, V, cost_t>, nodeid_t> search{
        heuristic, 
        goalChecker, 
        supplier,
        stateExpander,
        pruner
    };
    search.setListener(listener);

    search.setupSearch(nullptr, nullptr);
    auto startState = &supplier.getState(start);
    auto goalState = &supplier.getState(goal);
    auto solution = search.search(*startState, *goalState, false, false);
    //listener data is now populated correctly

    numberOfSolutions = listener.getNumberOfSolutions();
    countGraph = new AdjacentGraph<std::string, int, int>{listener.getCountGraph()};
    optimalSolutionCost = solution->getCost();
    solutionExample = getSolutionString(*solution);
}

static boost::filesystem::path getOutputOriginalJsonPath(const Globals& globals) {

	boost::filesystem::path p{globals.outputMainDirectory};

	auto fileName = callPyEval(globals.outputMainJsonOriginalMapFilenamePyString, false,
		"MAPNAME", globals.mapName
	);
	critical("output json file name is", fileName);

	p /= fileName;
    boost::filesystem::absolute(p).normalize();
	return p;
}

static boost::filesystem::path getOutputPerturbatedJsonPath(int perturbatedGraphId, const Globals& globals) {

	boost::filesystem::path p{globals.outputMainDirectory};

	auto fileName = callPyEval(globals.outputMainJsonMapFilenamePyString, false,
		"MAPNAME", globals.mapName,
        "PERTURBATEDMAPID", perturbatedGraphId
	);
	critical("output json file name is", fileName);

	p /= fileName;
    boost::filesystem::absolute(p).normalize();
	return p;
}

static boost::filesystem::path getOutputCSVPath(int perturbatedGraphId, const Globals& globals) {

	boost::filesystem::path p{globals.outputMainDirectory};

	auto fileName = callPyEval(globals.otuputMainCsvFilenamePyString, false,
		"MAPNAME", globals.mapName,
        "PERTURBATEDMAPID", perturbatedGraphId
	);
	critical("output csv file name is", fileName);

	p /= fileName;
    boost::filesystem::absolute(p).normalize();
	return p;
}

static boost::filesystem::path getLandmarkFilename(int perturbatedGraphId, const Globals& globals) {

	boost::filesystem::path p{globals.outputTempDirectory};

	auto fileName = callPyEval(globals.landmarkPathPyString, false,
		"MAPNAME", globals.mapName,
        "PERTURBATEDMAPID", perturbatedGraphId
	);
	critical("landmark file name is", fileName);

	p /= fileName;
    boost::filesystem::absolute(p).normalize();
	return p;
}