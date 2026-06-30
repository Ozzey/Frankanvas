// Copyright (c) 2019 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka/control_tools.h>

#include <cstring>
#include <exception>
#include <fstream>
#include <string>

#include "platform.h"

#ifdef LIBFRANKA_WINDOWS
#include <Windows.h>
#else
#include <pthread.h>
#endif

// `using std::string_literals::operator""s` produces a GCC warning that cannot be disabled, so we
// have to use `using namespace ...`.
// See https://gcc.gnu.org/bugzilla/show_bug.cgi?id=65923#c0
using namespace std::string_literals;  // NOLINT(google-build-using-namespace)

namespace franka {
}  // namespace franka
