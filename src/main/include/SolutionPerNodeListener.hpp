#ifndef _PATHFINDINGMAPPERTURBATOR_SOLUTIONPERNODELISTENER_HEADER__
#define _PATHFINDINGMAPPERTURBATOR_SOLUTIONPERNODELISTENER_HEADER__

#include <pathfinding-utils/AstarAllSolutionsListener.hpp>
#include <cpp-utils/mapplus.hpp>
#include <cpp-utils/vectorplus.hpp>
#include <cpp-utils/listGraph.hpp>

namespace pathfinding::map_perturbator::utils {

    using namespace pathfinding;

    /**
     * @brief keep track of the solutions found by the algorithms and all their vertices and edges
     * 
     * @tparam G 
     * @tparam STATE 
     */
    template <typename G, typename STATE>
    class SolutionPerNodeListener: public AstarListener<STATE> {
    public:
        using This = SolutionPerNodeListener<G, STATE>;
        using Super = AstarListener<STATE>;
    protected:
        /**
         * @brief a graph where each vertex contains the number of solutions passing the vertex while the edge labels contains the number of solution passing over the edge. vertex ids remain the same w.r.t the base graph
         */
        ListGraph<G, int, int> solutions;
        int numberOfSolutions;
    public:
        template <typename V, typename E>
        SolutionPerNodeListener(const IImmutableGraph<G, V, E>& baseGraph): solutions{baseGraph.getPayload()}, numberOfSolutions{0} {
            for (auto it=baseGraph.beginVertices(); it!=baseGraph.endVertices(); ++it) {
                if (solutions.numberOfVertices() != it->first) {
                    throw cpp_utils::exceptions::InvalidArgumentException{"other graph output vertex id but we are in another vertex!"};
                }
                solutions.addVertex(0);
            }

            for (auto it=baseGraph.beginEdges(); it!=baseGraph.endEdges(); ++it) {
                solutions.addEdge(it->getSourceId(), it->getSinkId(), 0);
            }
        }
        virtual ~SolutionPerNodeListener() {

        }
        SolutionPerNodeListener(const This& o): solutions{o.solutions}, numberOfSolutions{o.numberOfSolutions} {

        }
        This& operator=(const This& o) {
            this->solutions = o.solutions;
            this->numberOfSolutions = o.numberOfSolutions;
            return *this;
        }
        This&& operator=(This&& o) {
            this->solutions = o.solutions;
            this->numberOfSolutions = o.numberOfSolutions;
            return *this;
        }
    public:
        /**
         * @brief Get the Count Graph object
         * 
         * @return const IImmutableGraph<G, int, int>& the graph where each vertex(edge) contain the number fo solution going through such vertex(edge)
         */
        const IImmutableGraph<G, int, int>& getCountGraph() const {
            return this->solutions;
        }
        /**
         * @brief number of solution detected during the search algorithm
         * 
         * @return int 
         */
        int getNumberOfSolutions() const {
            return this->numberOfSolutions;
        }
    public:
        virtual void onSolutionFound(int iteration, const STATE& goal) {
            const STATE* tmp = &goal;
            const STATE* prev = nullptr;
            bool first = true;
            info("solution found! it is ", goal);

            this->numberOfSolutions += 1;
            finer("the number of solution found now are", this->numberOfSolutions);
            while (tmp != nullptr) {
                debug("inspecting vertex", *tmp);
                //update vertex
                int oldVertexValue = this->solutions.getVertex(tmp->getId());
                this->solutions.changeVertexPayload(tmp->getId(), oldVertexValue + 1);
                //update edge
                if (first) {
                    prev = tmp;
                } else {
                    int oldEdgeValue = this->solutions.getEdge(tmp->getId(), prev->getId());
                    this->solutions.changeWeightEdge(tmp->getId(), prev->getId(), oldEdgeValue + 1);
                }
                tmp = tmp->getParent();
            }
        }
        virtual void onNodeExpanded(int iteration, const STATE& node) {

        }

        virtual void onNodeGenerated(int iteration, const STATE& node) {

        }

        virtual void onStartingComputingHeuristic(int iteration, const STATE& s) {

        }

        virtual void onEndingComputingHeuristic(int iteration, const STATE& s) {

        }
    public:
        virtual void cleanup() {

        }

    };

}

#endif