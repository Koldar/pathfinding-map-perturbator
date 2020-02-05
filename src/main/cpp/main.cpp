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
#include <pathfinding-utils/map_base_reason_e.hpp>

#include "SolutionPerNodeListener.hpp"


using namespace cpp_utils;
using namespace pathfinding;
using namespace pathfinding::search;
using namespace pathfinding::maps;
using namespace pathfinding::map_perturbator::utils;

using State = GraphState<std::string, xyLoc, cost_t, map_base_reason_e>;

template <typename G, typename V>
static void solveQuery(int perturbatedMapId, int queryId, const IImmutableGraph<G,V,cost_t>& perturbatedGraph, nodeid_t start, nodeid_t goal, const Globals& globals, int& numberOfSolutions, const IImmutableGraph<G,int,int>*& countGraph, cost_t& optimalSolutionCost, std::string& solutionExample);

template <typename G, typename V>
static GridMapImage* generatePerturbatedImage(const Globals& glopbals, int perturbatedMapId, const GridMap& gridMap, const IImmutableGraph<G, V, cost_t>& baseMapGraph, const IImmutableGraph<G, V, PerturbatedCost>& perturbatedGraph, double minMultiplier, double maxMultiplier);

template <typename G, typename V, typename E>
void generateMapWithSolutions(const GridMapImage& baseGridMapImage, const IImmutableGraph<G, V, cost_t>& costMap, const IImmutableGraph<G, int, E>& countGraph, const boost::filesystem::path& imageName);

static boost::filesystem::path getOutputOriginalJsonPath(const Globals& globals);
static boost::filesystem::path getOutputPerturbatedJsonPath(int perturbatedGraphId, const Globals& globals);
static boost::filesystem::path getOutputMainSolutionImagePath(int perturbatedMapId, int queryId, const Globals& globals);
static boost::filesystem::path getOutputMainPerturbatedImagePath(int perturbatedMapId, const Globals& globals);

static std::string getSolutionString(const ISolutionPath<State, const State*, const State&>& solution);
static std::string getEdgeDistribution(const IImmutableGraph<std::string, int, int>& countGraph);
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
    app.add_option("--output-main-perturbated-map-image-pystring", globals.outputMainPerturbatedMapImagePystring,
        "Name of the image to generate repersenting perturbated map where the queries will be performed"
        " - MAPNAME: name of the original map which was perturbated"
        " - PERTURBATEDMAPID: id of the perturbated map generated by the application"
    )->required();
    app.add_option("--output-main-solution-image-pystring", globals.outputMainSolutionImagePyString,
        "Name of the image to generate repersenting the \"corridor\" where the solutions actually are"
        " - MAPNAME: name of the original map which was perturbated"
        " - PERTURBATEDMAPID: id of the perturbated map generated by the application"
        " - QUERYID: id of the pathfinding query involved"
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

    auto perturbationRangeStr = callPyEvalWithEval(globals.perturbationRangePyString,
			"V", baseMapGraph.numberOfVertices(),
			"E", baseMapGraph.numberOfEdges()
		);
    cpp_utils::Interval<double> perturbationRange = cpp_utils::Interval<double>::parseDoubleInterval(perturbationRangeStr);
    critical("perturbation is ", perturbationRange);

	std::unique_ptr<AbstractPerturbator<std::string, xyLoc>> perturbator{nullptr};
	if (globals.type == std::string{"RANDOM"}) {
		//RANDOM
		uint32_t edgesToAlter = callPyEvalAndCastNumberTo<uint32_t>(globals.numberOfPerturbationPyString,
			"V", baseMapGraph.numberOfVertices(),
			"E", baseMapGraph.numberOfEdges()
		);

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
        auto perturbatedImage = generatePerturbatedImage(
            globals,
            perturbatedMapId, gridMap, 
            baseMapGraph, *perturbatedGraph,
            perturbationRange.getLowerbound(),
            perturbationRange.getUpperbound()
        );

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
                    getEdgeDistribution(*countGraph),
                    getVertexDistribution(gridMap, baseMapGraph, *countGraph)
                );

                generateMapWithSolutions(
                    *perturbatedImage, 
                    *perturbatedGraphWithCost, 
                    *countGraph, 
                    getOutputMainSolutionImagePath(perturbatedMapId, queryId, globals)
                );

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


template <typename G, typename V, typename E>
void generateMapWithSolutions(const GridMapImage& baseGridMapImage, const IImmutableGraph<G, V, cost_t>& costMap, const IImmutableGraph<G, int, E>& countGraph, const boost::filesystem::path& imageName) {
    GridMapImage* perturbatedImageWithLocations = new GridMapImage{baseGridMapImage};
    for (auto it=countGraph.beginVertices(); it!=countGraph.endVertices(); ++it) {
        xyLoc loc = costMap.getVertex(it->first);
        if (it->second > 0) {
            perturbatedImageWithLocations->lerpGridCellColor(loc, color_t::YELLOW);
        }
    }
    perturbatedImageWithLocations->saveJPEG(imageName);
    delete perturbatedImageWithLocations;
}

/**
 * @brief create an image representing the perturbated map
 * 
 * @tparam G type of the payload of the whole graph
 * @tparam V type of the payload of each vertex
 * @param perturbatedMapId id representing the perturbated map
 * @param gridMap grid map involved
 * @param baseMapGraph graph representing the original gridmap
 * @param perturbatedGraph graph representing the graph where we pathfind
 * @param minMultiplier the minimum multiplier that can be multiplied to an original weight
 * @param maxMultiplier the maximum multiplier that can be multiplied to an original weight
 * @return GridMapImage* an image representing the perturbated gridmap
 */
template <typename G, typename V>
static GridMapImage* generatePerturbatedImage(const Globals& globals, int perturbatedMapId, const GridMap& gridMap, const IImmutableGraph<G, V, cost_t>& baseMapGraph, const IImmutableGraph<G, V, PerturbatedCost>& perturbatedGraph, double minMultiplier, double maxMultiplier) {
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

        cost_t maxPerturbation = static_cast<double>(oldWeight) * maxMultiplier;
        cost_t minPerturbation = static_cast<double>(oldWeight) * minMultiplier;
        if (newWeight > oldWeight) {
            //cost has increased
            double n = static_cast<double>(newWeight) - static_cast<double>(oldWeight);
            double d = static_cast<double>(maxPerturbation) - static_cast<double>(oldWeight);
            perturbatedColor = color_t::RED.darker(bound(n/d, 0.0, 0.90));
        } else {
            //cost has decreased
            double n = static_cast<double>(oldWeight) - static_cast<double>(newWeight);
            double d = static_cast<double>(oldWeight) - static_cast<double>(minPerturbation);
            perturbatedColor = color_t::BLUE.darker(bound(n/d, 0.0, 0.90));
        }
        image->setPixelInGrid(source.x, source.y, pixel1x, pixel1y, perturbatedColor); 
        image->setPixelInGrid(sink.x, sink.y, pixel2x, pixel2y, perturbatedColor);
    }

    auto imageName = getOutputMainPerturbatedImagePath(perturbatedMapId, globals);
    image->saveJPEG(imageName);

    return image.release();
}

/**
 * @brief a JSON-like containing, for each edge, the number of solutions passing over it
 * 
 * The JSON return look like this:
 * 
 * @code
 * [
 *  {"source": 3@ "sink": 4@ "solutions": 6}@
 *  {"source": 3@ "sink": 5@ "solutions": 0}@
 *  ...
 * ]
 * @endcode
 * 
 * To obtain a JSON, simply replace "@" with ","
 * 
 * @param countGraph 
 * @return std::string 
 */
static std::string getEdgeDistribution(const IImmutableGraph<std::string, int, int>& countGraph) {
    vectorplus<std::string> json{};
    for (auto it=countGraph.beginEdges(); it!=countGraph.endEdges(); ++it) {
        json.add(scout("{\"s\": ", it->getSourceId(), "@ \"t\": ", it->getSinkId(), "@ \"sol\": ", it->getPayload(), "}"));
    }
    return json.makeString("[", "@", "]");
}

static std::string getVertexDistribution(const pathfinding::maps::GridMap& gridMap, const IImmutableGraph<std::string, xyLoc, cost_t>& baseMap, const IImmutableGraph<std::string, int, int>& countGraph) {
    vectorplus<std::string> json{};
    for (auto it=countGraph.beginVertices(); it!=countGraph.beginVertices(); ++it) {
        json.add(scout("{\"v\": ", it->first, "@ \"sol\": ", it->second, "}"));
    }
    return json.makeString("[", "@", "]");
    // std::stringstream ss;

    // ss << "[";
    // for (ucood_t y=0; y<gridMap.getHeight(); ++y) {
    //     for (ucood_t x=0; x<gridMap.getWidth(); ++x) {
    //         xyLoc position{x, y};
    //         if (gridMap.isTraversable(position)) {
    //             nodeid_t id = baseMap.idOfVertex(position);
    //             ss << countGraph.getVertex(id);
    //         } else {
    //             ss << "-";
    //         }

    //         if ((x+1) < gridMap.getWidth()) {
    //             ss << "|";
    //         }
    //     }
    //     ss <<"@";
    // }
    // ss <<"]";

    // return ss.str();
}


static std::string getSolutionString(const ISolutionPath<State, const State*, const State&>& solution) {
    const State* tmp = &solution.getGoal();

    std::stringstream ss;

    ss << "[";
    for (int i=0; i<solution.size(); ++i) {
        nodeid_t idInvolved = solution.at(i).getId();
        xyLoc loc = solution.at(i).getGraph().getVertex(idInvolved);
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

    GraphStateSupplier<G, V, cost_t, map_base_reason_e> supplier{perturbatedGraph};
    cpp_utils::function_t<cost_t, cost_t> costFunction = [&](auto c) { return c;};
    StandardStateExpander<State, G, V, cost_t, map_base_reason_e> stateExpander{perturbatedGraph, costFunction};
    
    ALTHeuristic<State, std::string, V> heuristic{perturbatedGraph, landmarkDatabase};
    NeverPrune<State> pruner{};
    StandardLocationGoalChecker<State> goalChecker{};
    SolutionPerNodeListener<G, State> listener{perturbatedGraph};

    AStarAllSolutions<State, nodeid_t, map_base_reason_e> search{
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

static boost::filesystem::path getOutputMainPerturbatedImagePath(int perturbatedMapId, const Globals& globals) {

	boost::filesystem::path p{globals.outputMainDirectory};

	auto fileName = callPyEval(globals.outputMainPerturbatedMapImagePystring, false,
		"MAPNAME", globals.mapName,
        "PERTURBATEDMAPID", perturbatedMapId
	);
	critical("output json file name is", fileName);

	p /= fileName;
    boost::filesystem::absolute(p).normalize();
	return p;
}

static boost::filesystem::path getOutputMainSolutionImagePath(int perturbatedMapId, int queryId, const Globals& globals) {

	boost::filesystem::path p{globals.outputMainDirectory};

	auto fileName = callPyEval(globals.outputMainSolutionImagePyString, false,
		"MAPNAME", globals.mapName,
        "PERTURBATEDMAPID", perturbatedMapId,
        "QUERYID", queryId

	);
	critical("output json file name is", fileName);

	p /= fileName;
    boost::filesystem::absolute(p).normalize();
	return p;
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