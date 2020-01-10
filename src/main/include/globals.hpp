#ifndef __PATHFINDINGMAPPERTURBATOR_MAP_PERTURBATOR_HEADER__
#define __PATHFINDINGMAPPERTURBATOR_MAP_PERTURBATOR_HEADER__

#include <boost/filesystem.hpp>
#include <string>
#include <cpp-utils/Random.hpp>


class Globals {
public:
    /**
     * @brief path of the input map
     * 
     */
    boost::filesystem::path mapPath;
    /**
     * @brief name of the involved map in ::mapPath
     * 
     */
    std::string mapName;
    /**
     * @brief type of the perturbations to apply
     * 
     */
    std::string type;
    uint32_t randomSeed;
    /**
     * @brief directory where we will put the main contents of the generator
     * 
     */
    std::string outputMainDirectory;
    /**
     * @brief directory where we will put files which are not the main output ones
     * 
     */
    std::string outputTempDirectory;
    /**
     * @brief number of pathfinding queries to perform per perturbated map generated
     * 
     */
    int queryPerPerturbatedMap;
    /**
     * @brief python eval string representing number of perturbated maps we need to create
     * 
     */
    std::string numberOfPerturbatedMapsPyString;
    /**
     * @brief python eval string representing the number of perturbations to perform per map
     * 
     */
    std::string numberOfPerturbationPyString;
    /**
     * @brief python eval string representing the range the perturbation can have.
     * 
     * The value generated is multiplied to the original edge cost
     * 
     */
    std::string perturbationRangePyString;
    /**
     * @brief true if you want to include infinity as the perturbated value
     * 
     */
    bool includeInfinite;
    /**
     * @brief python eval string representing how much probable is to perturbate an arc with infinity
     * 
     */
    std::string infinityProbabilityPyString;
    /**
     * @brief name of the file repersenting the landmark database
     * 
     */
    std::string landmarkPathPyString;
    /**
     * @brief python eval string representing the name to generate for the CSV containing infor regarding the perturbated map
     * 
     */
    std::string otuputMainCsvFilenamePyString;
    /**
     * @brief python eval string representing the name of the JSon representing the perturbated map
     * 
     */
    std::string outputMainJsonMapFilenamePyString;
    /**
     * @brief python eval string representing the name of the JSon representing the original map
     * 
     */
    std::string outputMainJsonOriginalMapFilenamePyString;
    /**
     * @brief true if you want to generate an image representing the perturbated map image
     * 
     * In it each pixel represent the edge cost. image is nxn when n is the number of locations in the map
     * 
     * White pixel represents the minimum an edge can have. Black represent untraverable edges
     * Black -1 represents the maximum an edge can have
     */
    bool generatePerturbatedMapImage;
    /**
     * @brief if ::generatePerturbatedMapImage is true, a python eval string containing the path where to save the image
     * 
     */
    std::string perturbatedMapImagePathPyString;
    /**
     * @brief random number generator to use
     * 
     */
    cpp_utils::Random randomGenerator;
};

#endif