#include "msrm_utils/json.hpp"
#include "catch/catch.hpp"

namespace msrm_utils{

TEST_CASE("convert to json","[json]"){

    Eigen::Matrix<double,4,1> a;
    a<<1,2,3,4;
    nlohmann::json a_json;
    a_json={1,2,3,4};

    REQUIRE(from_eigen<double,4,1>(a)==a_json);

}

}
