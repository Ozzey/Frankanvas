#include "msrm_utils/system.hpp"
#include "catch/catch.hpp"
#include <iostream>

namespace msrm_utils {

TEST_CASE("is process running","[system]"){
    REQUIRE(process_is_running("unit_tests"));
    REQUIRE(!process_is_running("unit_tests2"));
}

TEST_CASE("system command","[system]"){
    std::cout<<"SLEEP"<<std::endl;
//    std::string result = exec("sleep 5 &");
    int result = system("sleep 5 &");
    std::cout<<result<<std::endl;
}

}
