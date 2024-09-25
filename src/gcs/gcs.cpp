#include "gcs.h"

int BaseGCS::dimension()
{
    return _dimension;
}

BaseGCS::BaseGCS(const std::vector<drake::geometry::optimization::HPolyhedron> &regions)
{
    this->source = this->target = nullptr;
    this->regions = regions;
    for (size_t i = 0; i < regions.size(); i++)
    {
        names.push_back("v" + std::to_string(i));
    }
    assert(regions.size() > 0);
    _dimension = regions[0].ambient_dimension();
    for (auto &r : this->regions)
    {
        assert(r.ambient_dimension() == _dimension);
    }
}

BaseGCS::~BaseGCS()
{
}

std::vector<std::pair<int, int>> BaseGCS::findEdgesViaFullDimensionOverlaps()
{
    return std::vector<std::pair<int, int>>();
}

std::vector<std::pair<int, int>> BaseGCS::findEdgesViaOverlaps()
{
    return std::vector<std::pair<int, int>>();
}

std::array<std::vector<int>, 2> BaseGCS::findStartGoalEdges(drake::VectorX<double> &start, drake::VectorX<double> &goal)
{
    std::array<std::vector<int>, 2> res;
    for (size_t i = 0; i < regions.size(); i++)
    {
        if (regions[i].PointInSet(start))
        {
            res[0].push_back(i);
        }
        if (regions[i].PointInSet(goal))
        {
            res[1].push_back(i);
        }
    }
    return res;
}

std::array<std::vector<drake::geometry::optimization::GraphOfConvexSets::Edge *>, 2> BaseGCS::addSourceTarget(drake::VectorX<double> &source, drake::VectorX<double> &target, std::array<std::vector<int>, 2> edges)
{
    if (this->source && this->target)
    {
        this->gcs.RemoveVertex(this->source);
        this->gcs.RemoveVertex(this->target);
    }
    assert(source.size() == this->dimension());
    assert(target.size() == this->dimension());
    auto vertices = this->gcs.Vertices();
    // Add Edges connecting source and target to graph
    this->source = this->gcs.AddVertex(drake::geometry::optimization::Point(source), "source");
    this->target = this->gcs.AddVertex(drake::geometry::optimization::Point(target), "target");

    if (edges.empty())
    {
        edges = this->findStartGoalEdges(source, target);
    }

    if (edges[0].empty())
    {
        throw std::runtime_error("source doesn't connect any vertice");
    }
    if (edges[1].empty())
    {
        throw std::runtime_error("target doesn't connect any vertice");
    }
    std::array<std::vector<drake::geometry::optimization::GraphOfConvexSets::Edge *>, 2> connected_edges;
    for (auto &e : edges[0])
    {
        auto u = vertices[e];
        auto edge = this->gcs.AddEdge(this->source, u, "(source, " + u->name() + ")");
        connected_edges[0].push_back(edge);
    }
    for (auto &e : edges[1])
    {
        auto u = vertices[e];
        auto edge = this->gcs.AddEdge(this->target, u, "(" + u->name() + ", target)");
        connected_edges[1].push_back(edge);
    }
    return connected_edges;
}

void BaseGCS::solveGCS(bool rounding, bool preprocessing, bool verbose)
{
    std::unordered_map<std::string, std::variant<double, drake::solvers::MathematicalProgramResult>> results;
    this->options.convex_relaxation = rounding;
    this->options.preprocessing = preprocessing;
    this->options.max_rounding_trials = 0;

    auto result = this->gcs.SolveShortestPath(*this->source, *this->target, this->options);
    if (rounding)
    {
        results["relaxation_result"] = result;
        results["relaxation_solver_time"] = result.get_solver_details<drake::solvers::MosekSolver>().optimizer_time;
        results["relaxation_optimal_cost"] = result.get_optimal_cost();
    }
    else
    {
        results["mip_result"] = result;
        results["mip_solver_time"] = result.get_solver_details<drake::solvers::MosekSolver>().optimizer_time;
        results["mip_optimal_cost"] = result.get_optimal_cost();
    }
    if (!result.is_success())
    {
        drake::log()->info("Failed to solve the program");
        return;
    }
    if (verbose)
    {
        drake::log()->info("Solution\t\nSuccess: {}\nCost: {}\nSolver Time: {}",
                           result.get_solution_result(),
                           result.get_optimal_cost(),
                           result.get_solver_details<drake::solvers::MosekSolver>().optimizer_time);
    }
    // Solver with hard edge choices
    if (rounding) {
        options.max_rounded_paths = 10;
        auto rounding_result = this->gcs.SolveShortestPath(*this->source, *this->target, this->options);
        
    }
}

void BaseGCS::greedyForwardPathSearch(drake::geometry::optimization::GraphOfConvexSets &gcs, drake::solvers::MathematicalProgramResult &result, drake::geometry::optimization::GraphOfConvexSets::Vertex *source, drake::geometry::optimization::GraphOfConvexSets::Vertex *target, int max_paths, int max_trials, double flow_tol)
{
    
}

LinearGCS::LinearGCS(const std::vector<drake::geometry::optimization::HPolyhedron> regions, std::vector<std::pair<int, int>> edges, drake::VectorX<double> path_weights, bool full_dim_overlap) : BaseGCS(regions), edge_costs(Eigen::MatrixXd::Zero(this->dimension(), this->dimension() * 2), Eigen::VectorXd::Zero(this->dimension()))
{
    if (path_weights.size() == 0)
    {
        path_weights = Eigen::VectorXd::Ones(this->dimension());
    }
    else if (path_weights.size() == 1)
    {
        path_weights = path_weights(0) * Eigen::VectorXd::Ones(this->dimension());
    }
    assert(path_weights.size() == this->dimension());
    Eigen::MatrixXd cost_matrix = Eigen::MatrixXd::Zero(this->dimension(), this->dimension() * 2);
    for (int i = 0; i < this->dimension(); ++i)
    {
        cost_matrix(i, i) = -path_weights(i);
        cost_matrix(i, this->dimension() + i) = path_weights(i);
    }
    new (&edge_costs) drake::solvers::L2NormCost(cost_matrix, Eigen::VectorXd::Zero(this->dimension()));
    for (size_t i = 0; i < this->regions.size(); i++)
    {
        this->gcs.AddVertex(this->regions[i], names[i]);
    }
    if (edges.empty())
    {
        if (full_dim_overlap)
        {
            edges = findEdgesViaFullDimensionOverlaps();
        }
        else
        {
            edges = findEdgesViaOverlaps();
        }
    }
    auto vertices = this->gcs.Vertices();
    for (auto &[i, j] : edges)
    {
        auto u = vertices[i], v = vertices[j];
        auto edge = gcs.AddEdge(u, v, "(" + u->name() + "," + v->name() + ")");
        drake::log()->info("source vertice: ({1}){0} with a size of {2}", u->x(), typeid(u->x()).name(), u->x().size());
        drake::log()->info("target vertice: ({1}){0} with a size of {2}", v->x(), typeid(v->x()).name(), v->x().size());
        drake::VectorX<drake::symbolic::Variable> combine(2 * u->x().size());
        combine << u->x(), v->x();
        auto edge_length = (edge->AddCost(drake::solvers::Binding<drake::solvers::Cost>(std::shared_ptr<drake::solvers::Cost>(&edge_costs), combine)));
        auto hpoly_set = dynamic_cast<drake::geometry::optimization::HPolyhedron *>(v->set().Clone().get());
        auto linear_constraint = drake::solvers::LinearConstraint(hpoly_set->A(),
                                                                  -Eigen::VectorXd::Constant(hpoly_set->b().size(), std::numeric_limits<double>::infinity()),
                                                                  hpoly_set->b());
        edge->AddConstraint(drake::solvers::Binding<drake::solvers::Constraint>(std::shared_ptr<drake::solvers::Constraint>(&linear_constraint), v->x()));
    }
}

void LinearGCS::addSourceTarget(drake::VectorX<double> &source, drake::VectorX<double> &target, std::array<std::vector<int>, 2> edges)
{
    auto connected_edges = BaseGCS::addSourceTarget(source, target, edges);
    for (auto &e : connected_edges[0])
    {
        for (size_t j = 0; j < this->dimension(); j++)
        {
            e->AddConstraint(e->xu()[j] == e->xv()[j]);
        }
    }
    for (auto &e : connected_edges[1])
    {
        drake::VectorX<drake::symbolic::Variable> combine(2 * e->xu().size());
        combine << e->xu(), e->xv();
        e->AddCost(drake::solvers::Binding<drake::solvers::Cost>(std::shared_ptr<drake::solvers::Cost>(&edge_costs), combine));
    }
}

LinearGCS::~LinearGCS()
{
}
