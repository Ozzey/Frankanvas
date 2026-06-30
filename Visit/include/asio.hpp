#pragma once

#include <boost/asio.hpp>
#include <boost/system/system_error.hpp>

namespace boost
{
namespace asio
{
using error_code = boost::system::error_code;
using system_error = boost::system::system_error;
}
}

namespace asio = boost::asio;

#ifndef ASIO_INITFN_RESULT_TYPE
#define ASIO_INITFN_RESULT_TYPE BOOST_ASIO_INITFN_RESULT_TYPE
#endif

#ifndef ASIO_MOVE_ARG
#define ASIO_MOVE_ARG BOOST_ASIO_MOVE_ARG
#endif
