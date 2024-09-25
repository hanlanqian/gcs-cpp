#include "gcs.h"

int main(int argc, char const *argv[])
{
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
    return 0;
}
