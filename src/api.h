#pragma once

#if defined _WIN32 || defined __CYGWIN__
#  define CollisionBenchmarkController_DLLIMPORT __declspec(dllimport)
#  define CollisionBenchmarkController_DLLEXPORT __declspec(dllexport)
#  define CollisionBenchmarkController_DLLLOCAL
#else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#    define CollisionBenchmarkController_DLLIMPORT __attribute__((visibility("default")))
#    define CollisionBenchmarkController_DLLEXPORT __attribute__((visibility("default")))
#    define CollisionBenchmarkController_DLLLOCAL __attribute__((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#    define CollisionBenchmarkController_DLLIMPORT
#    define CollisionBenchmarkController_DLLEXPORT
#    define CollisionBenchmarkController_DLLLOCAL
#  endif // __GNUC__ >= 4
#endif // defined _WIN32 || defined __CYGWIN__

#ifdef CollisionBenchmarkController_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define CollisionBenchmarkController_DLLAPI
#  define CollisionBenchmarkController_LOCAL
#else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef CollisionBenchmarkController_EXPORTS
#    define CollisionBenchmarkController_DLLAPI CollisionBenchmarkController_DLLEXPORT
#  else
#    define CollisionBenchmarkController_DLLAPI CollisionBenchmarkController_DLLIMPORT
#  endif // CollisionBenchmarkController_EXPORTS
#  define CollisionBenchmarkController_LOCAL CollisionBenchmarkController_DLLLOCAL
#endif // CollisionBenchmarkController_STATIC
