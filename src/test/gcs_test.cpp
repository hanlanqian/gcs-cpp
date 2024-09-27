#include "gcs.h"
#include "helpers.h"
#include <drake/common/yaml/yaml_io.h>

void test1(){
        int count = 5;
    drake::VectorX<double> empty_weights;
    std::vector<drake::geometry::optimization::HPolyhedron> regs;
    for (size_t i = 0; i < count; i++)
    {
        auto matrix = Eigen::MatrixXd::Random(4, 3);
        auto vec = Eigen::VectorXd::Random(4);
        regs.push_back(drake::geometry::optimization::HPolyhedron(matrix, vec));
    }
    auto lgcs = LinearGCS(regs, {{1, 2}, {2, 3}, {1, 3}});
    lgcs.solveGCS();
}
void test2(){
    std::vector<drake::geometry::optimization::HPolyhedron> rr;
    for (size_t i = 0; i < 5; i++)
    {
        rr.push_back(drake::geometry::optimization::HPolyhedron(drake::MatrixX<double>::Random(3,3), drake::VectorX<double>::Random(3)));
    }
    helpers::IrisRegions iris(rr);
    drake::yaml::SaveYamlFile("./test.yaml", iris);
}
int main(int argc, char const *argv[])
{
    test1();
    return 0;
}
