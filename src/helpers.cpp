#include "helpers.h"

#include <future>

#include <drake/geometry/geometry_roles.h>
#include <drake/geometry/geometry_set.h>

#include <drake/common/timer.h>

template <typename T>
std::vector<T> helpers::concatenateVectors(const std::initializer_list<std::vector<T>> &vectors)
{
    std::vector<T> result;
    for (const auto &vec : vectors)
    {
        result.insert(result.end(), vec.begin(), vec.end());
    }
    return result;
}

void helpers::filterCollsionGeometry(drake::geometry::SceneGraph<double> &scene_graph, drake::systems::Context<double> *context)
{
    auto filter_manager = scene_graph.collision_filter_manager(context);
    auto inspector = &scene_graph.model_inspector();
    std::vector<std::vector<drake::geometry::GeometryId>> iiwa1(8), iiwa2(8);
    std::vector<drake::geometry::GeometryId> wsg1, wsg2, shelf, table;
    std::vector<std::vector<drake::geometry::GeometryId>> bins(2);
    for (auto gid : inspector->GetGeometryIds(
             drake::geometry::GeometrySet(inspector->GetAllGeometryIds()), drake::geometry::Role::kProximity))
    {
        auto gid_name = inspector->GetName(inspector->GetFrameId(gid));
        drake::log()->info(gid_name);
        if (gid_name.find("iiwa_1::iiwa_link_") == 0)
        {
            int link_num = gid_name[18] - '0';
            drake::log()->info(link_num);
            iiwa1[link_num].push_back(gid);
        }
        else if (gid_name.find("iiwa_2::iiwa_link_") == 0)
        {
            int link_num = gid_name[18] - '0';
            drake::log()->info(link_num);
            iiwa2[link_num].push_back(gid);
        }
        else if (gid_name.find("wsg_1") == 0)
        {
            wsg1.push_back(gid);
        }
        else if (gid_name.find("wsg_2") == 0)
        {
            wsg2.push_back(gid);
        }
        else if (gid_name.find("shelves::") == 0)
        {
            shelf.push_back(gid);
        }
        else if (gid_name.find("binR") == 0)
        {
            bins[0].push_back(gid);
        }
        else if (gid_name.find("binL") == 0)
        {
            bins[1].push_back(gid);
        }
        else if (gid_name.find("table") == 0)
        {
            table.push_back(gid);
        }
        else
        {
            drake::log()->info("Geometry " + gid_name + " not assigned to an object.");
        }
    }
    filter_manager.Apply(drake::geometry::CollisionFilterDeclaration().ExcludeWithin(
        drake::geometry::GeometrySet(helpers::concatenateVectors({iiwa1[0], iiwa1[1], iiwa1[2], iiwa1[3], shelf}))));
    filter_manager.Apply(drake::geometry::CollisionFilterDeclaration().ExcludeBetween(
        drake::geometry::GeometrySet(helpers::concatenateVectors({iiwa1[1], iiwa1[2], iiwa1[3]})),
        drake::geometry::GeometrySet(helpers::concatenateVectors({iiwa1[4], iiwa1[5]}))));
    filter_manager.Apply(drake::geometry::CollisionFilterDeclaration().ExcludeBetween(
        drake::geometry::GeometrySet(helpers::concatenateVectors({iiwa1[3], iiwa1[4]})), drake::geometry::GeometrySet(iiwa1[6])));
    filter_manager.Apply(drake::geometry::CollisionFilterDeclaration().ExcludeBetween(
        drake::geometry::GeometrySet(helpers::concatenateVectors({iiwa1[2], iiwa1[3], iiwa1[4], iiwa1[5], iiwa1[6]})),
        drake::geometry::GeometrySet(helpers::concatenateVectors({iiwa1[7], wsg1}))));
    filter_manager.Apply(drake::geometry::CollisionFilterDeclaration().ExcludeBetween(
        drake::geometry::GeometrySet(helpers::concatenateVectors({iiwa1[0], iiwa1[0], iiwa1[2]})), drake::geometry::GeometrySet(bins[0])));
    filter_manager.Apply(drake::geometry::CollisionFilterDeclaration().ExcludeBetween(
        drake::geometry::GeometrySet(helpers::concatenateVectors({iiwa1[0], iiwa1[1], iiwa1[2], iiwa1[3], iiwa1[4]})),
        drake::geometry::GeometrySet(bins[1])));
    filter_manager.Apply(drake::geometry::CollisionFilterDeclaration().ExcludeBetween(
        drake::geometry::GeometrySet(helpers::concatenateVectors({iiwa1[0], iiwa1[0], iiwa1[2]})), drake::geometry::GeometrySet(table)));

    filter_manager.Apply(drake::geometry::CollisionFilterDeclaration().ExcludeWithin(
        drake::geometry::GeometrySet(helpers::concatenateVectors({iiwa2[0], iiwa2[1], iiwa2[2], iiwa2[3], shelf}))));
    filter_manager.Apply(drake::geometry::CollisionFilterDeclaration().ExcludeBetween(
        drake::geometry::GeometrySet(helpers::concatenateVectors({iiwa2[1], iiwa2[2], iiwa2[3]})),
        drake::geometry::GeometrySet(helpers::concatenateVectors({iiwa2[4], iiwa2[5]}))));
    filter_manager.Apply(drake::geometry::CollisionFilterDeclaration().ExcludeBetween(
        drake::geometry::GeometrySet(helpers::concatenateVectors({iiwa2[3], iiwa2[4]})), drake::geometry::GeometrySet(iiwa2[6])));
    filter_manager.Apply(drake::geometry::CollisionFilterDeclaration().ExcludeBetween(
        drake::geometry::GeometrySet(helpers::concatenateVectors({iiwa2[2], iiwa2[3], iiwa2[4], iiwa2[5], iiwa2[6]})),
        drake::geometry::GeometrySet(helpers::concatenateVectors({iiwa2[7], wsg2}))));
    filter_manager.Apply(drake::geometry::CollisionFilterDeclaration().ExcludeBetween(
        drake::geometry::GeometrySet(helpers::concatenateVectors({iiwa2[0], iiwa2[0], iiwa2[2]})), drake::geometry::GeometrySet(bins[1])));
    filter_manager.Apply(drake::geometry::CollisionFilterDeclaration().ExcludeBetween(
        drake::geometry::GeometrySet(helpers::concatenateVectors({iiwa2[0], iiwa2[1], iiwa2[2], iiwa2[3], iiwa2[4]})),
        drake::geometry::GeometrySet(bins[0])));
    filter_manager.Apply(drake::geometry::CollisionFilterDeclaration().ExcludeBetween(
        drake::geometry::GeometrySet(helpers::concatenateVectors({iiwa2[0], iiwa2[0], iiwa2[2]})), drake::geometry::GeometrySet(table)));

    filter_manager.Apply(drake::geometry::CollisionFilterDeclaration().ExcludeBetween(
        drake::geometry::GeometrySet(helpers::concatenateVectors({iiwa1[0], iiwa1[1]})), drake::geometry::GeometrySet(helpers::concatenateVectors({iiwa2[0], iiwa2[1]}))));
    filter_manager.Apply(drake::geometry::CollisionFilterDeclaration().ExcludeBetween(
        drake::geometry::GeometrySet(iiwa1[2]), drake::geometry::GeometrySet(helpers::concatenateVectors({iiwa2[0], iiwa2[1]}))));
    filter_manager.Apply(drake::geometry::CollisionFilterDeclaration().ExcludeBetween(
        drake::geometry::GeometrySet(helpers::concatenateVectors({iiwa1[0], iiwa1[1]})), drake::geometry::GeometrySet(iiwa2[2])));

    // drake::log()->info(scene_graph.get_query_output_port().get_name());
    // drake::log()->info(scene_graph.get_query_output_port().Eval<drake::geometry::QueryObject>(context));
    auto pairs = scene_graph.get_query_output_port().Eval<drake::geometry::QueryObject<double>>(*context).inspector().GetCollisionCandidates();
    drake::log()->info("Filtered collision pairs from " + std::to_string(inspector->GetCollisionCandidates().size()) + " to " + std::to_string(pairs.size()));
}

std::unordered_map<std::string, drake::math::RigidTransform<double>> helpers::getIkSeeds()
{

    return std::unordered_map<std::string, drake::math::RigidTransform<double>>();
}

drake::geometry::optimization::HPolyhedron helpers::calcRegion(int i, const drake::VectorX<double> &seed, drake::multibody::MultibodyPlant<double> &plant, drake::systems::Context<double> *context, drake::geometry::optimization::IrisOptions &iris_options)
{
    auto timer = new drake::SteadyTimer();
    timer->Start();
    auto plan_context = &plant.GetMyMutableContextFromRoot(context);
    plant.SetPositions(plan_context, seed);
    drake::log()->info("Time: {1} minutes.", i, timer->Tick());
    sem.acquire();
    auto hpoly = drake::geometry::optimization::IrisInConfigurationSpace(plant, *plan_context, iris_options);
    sem.release();
    drake::log()->info("Seed: {0}\tTime: {1} minutes.\tFaces: {2}", i, timer->Tick(), hpoly.b().size());
    return hpoly;
}

std::vector<drake::geometry::optimization::HPolyhedron> helpers::generateRegions(std::unordered_map<std::string, drake::VectorX<double>> &seeds_points, drake::multibody::MultibodyPlant<double> &plant, drake::systems::Context<double> *context, drake::geometry::optimization::IrisOptions &iris_options)
{
    std::vector<drake::VectorX<double>> seeds;
    std::vector<drake::geometry::optimization::HPolyhedron> regions;
    for (auto &sp : seeds_points)
    {
        seeds.push_back(sp.second);
    }
    auto timer = new drake::SteadyTimer();
    std::vector<std::future<drake::geometry::optimization::HPolyhedron>> futures;
    for (int i = 0; i < seeds.size(); i++)
    {
        futures.push_back(std::async(std::launch::async, calcRegion, i, seeds[i], std::ref(plant), context, std::ref(iris_options)));
        // futures.push_back(std::async(std::launch::async, calcRegion, i, seeds[i], std::ref(plant), context, std::ref(iris_options)));
    }
    for (auto &fu : futures){
        fu.wait();
    }
    for (auto &fu : futures)
    {
        regions.push_back(fu.get());
    }
    drake::log()->info("Elapsed Time: {0}", timer->Tick());
    return regions;
}

std::unordered_map<std::string, drake::VectorX<double>> helpers::getConfigurationSeeds()
{
    std::unordered_map<std::string, drake::VectorX<double>> seeds{
        {"top_shelf/top_shelf", drake::VectorX<double>::Map(std::vector<double>{
                                                                0.37080011, 0.41394084, -0.16861973, -0.70789778, -0.37031516, 0.60412162, 0.39982981, -0.37080019, 0.41394089, 0.16861988, -0.70789766, 0.37031506, 0.60412179, -0.39982996}
                                                                .data(),
                                                            14)},
        {"top_shelf/shelf_1", drake::VectorX<double>::Map(std::vector<double>{0.37080079, 0.41394132, -0.16862043, -0.70789679, -0.37031656, 0.60412327, 0.39982969, -0.93496924, 0.46342534, 0.92801666, -1.45777635, -0.31061724, -0.0657716, -0.06019899}.data(), 14)},
        {"top_shelf/shelf_2", drake::VectorX<double>::Map(std::vector<double>{0.37086448, 0.41394538, -0.16875166, -0.70789745, -0.37020563, 0.60411217, 0.399785, -0.4416204, 0.62965228, 0.20598405, -1.73324339, -0.41354372, -0.68738414, 0.17443976}.data(), 14)},
        {"top_shelf/bin_L", drake::VectorX<double>::Map(std::vector<double>{0.37081989, 0.41394235, -0.16866012, -0.70789737, -0.37028201, 0.60411923, 0.39981634, -0.89837331, -1.1576151, 1.75505216, -1.37515153, 1.0676443, 1.56371166, -0.64126346}.data(), 14)},
        {"shelf_1/top_shelf", drake::VectorX<double>::Map(std::vector<double>{0.93496924, 0.46342534, -0.92801666, -1.45777635, 0.31061724, -0.0657716, 0.06019899, -0.37080079, 0.41394132, 0.16862043, -0.70789679, 0.37031656, 0.60412327, -0.39982969}.data(), 14)},
        {"shelf_1/shelf_1", drake::VectorX<double>::Map(std::vector<double>{0.87224109, 0.43096634, -0.82223436, -1.45840049, 0.73813452, -0.08999384, -0.41624203, -0.87556489, 0.43246906, 0.82766047, -1.45838515, -0.72259842, -0.0884963, 0.39840129}.data(), 14)},
        {"shelf_1/shelf_2", drake::VectorX<double>::Map(std::vector<double>{0.93496866, 0.463425, -0.92801564, -1.45777634, 0.3106235, -0.06577172, 0.06019173, -0.44158858, 0.62964838, 0.20594112, -1.73324341, -0.41354987, -0.6873923, 0.17446778}.data(), 14)},
        {"shelf_1/bin_L", drake::VectorX<double>::Map(std::vector<double>{0.93496918, 0.46342531, -0.92801656, -1.45777637, 0.31061728, -0.06577167, 0.06019927, -0.89837321, -1.15761746, 1.75504915, -1.37515113, 1.06764716, 1.56371454, -0.64126383}.data(), 14)},
        {"shelf_2/top_shelf", drake::VectorX<double>::Map(std::vector<double>{0.4416204, 0.62965228, -0.20598405, -1.73324339, 0.41354372, -0.68738414, -0.17443976, -0.37086448, 0.41394538, 0.16875166, -0.70789745, 0.37020563, 0.60411217, -0.399785}.data(), 14)},
        {"shelf_2/shelf_1", drake::VectorX<double>::Map(std::vector<double>{0.44158858, 0.62964838, -0.20594112, -1.73324341, 0.41354987, -0.6873923, -0.17446778, -0.93496866, 0.463425, 0.92801564, -1.45777634, -0.3106235, -0.06577172, -0.06019173}.data(), 14)},
        {"shelf_2/shelf_2", drake::VectorX<double>::Map(std::vector<double>{0.44161313, 0.62965141, -0.20597435, -1.73324346, 0.41354447, -0.68738613, -0.17444557, -0.4416132, 0.62965142, 0.20597452, -1.73324348, -0.41354416, -0.68738609, 0.17444625}.data(), 14)},
        {"shelf_2/bin_L", drake::VectorX<double>::Map(std::vector<double>{0.44161528, 0.62965169, -0.20597726, -1.73324347, 0.41354399, -0.68738565, -0.17444283, -1.37292761, -0.68372976, 2.96705973, -1.41521783, 2.96705973, -1.11343251, -3.0140737}.data(), 14)},
        {"bin_R/top_shelf", drake::VectorX<double>::Map(std::vector<double>{0.81207926, -1.25359738, -1.58098625, -1.5155474, -1.32223687, 1.50549708, -2.38221725, -0.37085114, 0.4139444, 0.16872443, -0.70789757, 0.37022786, 0.60411401, -0.39979449}.data(), 14)},
        {"bin_R/shelf_1", drake::VectorX<double>::Map(std::vector<double>{0.81207923, -1.25358454, -1.58100042, -1.51554769, -1.32222337, 1.50548369, -2.3822204, -0.9349716, 0.46342674, 0.92802082, -1.45777624, -0.31059455, -0.0657707, -0.06022391}.data(), 14)},
        {"bin_R/shelf_2", drake::VectorX<double>::Map(std::vector<double>{0.81207937, -1.25360462, -1.58097816, -1.51554761, -1.32224557, 1.50550485, -2.38221483, -0.44166552, 0.62965782, 0.20604497, -1.7332434, -0.41353464, -0.6873727, 0.17439863}.data(), 14)},
        {"bin_R/bin_L", drake::VectorX<double>::Map(std::vector<double>{-1.73637519, 0.6209681, 0.24232887, -1.51538355, -0.17977474, 0.92618894, -3.01360257, 1.31861497, 0.72394333, 0.4044295, -1.37509496, -0.27461997, 1.20038493, 0.18611701}.data(), 14)},
        {"neutral/neutral", drake::VectorX<double>::Map(std::vector<double>{0.0, -0.2, 0, -1.2, 0, 1.6, 0.0, 0.0, -0.2, 0, -1.2, 0, 1.6, 0.0}.data(), 14)},
        {"neutral/shelf_1", drake::VectorX<double>::Map(std::vector<double>{0.0, -0.2, 0, -1.2, 0, 1.6, 0.0, -0.93496866, 0.463425, 0.92801564, -1.45777634, -0.3106235, -0.06577172, -0.06019173}.data(), 14)}};
    return seeds;
}