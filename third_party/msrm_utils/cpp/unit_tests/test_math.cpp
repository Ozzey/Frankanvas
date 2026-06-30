#include "msrm_utils/math.hpp"
#include "catch/catch.hpp"

namespace msrm_utils {

TEST_CASE("get rotation matrix from vector","[math]"){
    Eigen::Matrix<double,3,1> v1,v2;
    v1<<1,0,0;
    v2<<0,0,1;
    Eigen::Matrix<double,3,3> R1;
    R1<<0,0,-1,0,1,0,1,0,0;
    REQUIRE(build_rotation_matrix(v1,v2)==R1);

    v1<<0,2,0;
    std::cout<<build_rotation_matrix(v1,v2)<<std::endl;
}

}
