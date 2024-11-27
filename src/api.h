#pragma once

#if defined _WIN32 || defined __CYGWIN__
#  define CircularOriController_DLLIMPORT __declspec(dllimport)
#  define CircularOriController_DLLEXPORT __declspec(dllexport)
#  define CircularOriController_DLLLOCAL
#else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#    define CircularOriController_DLLIMPORT __attribute__((visibility("default")))
#    define CircularOriController_DLLEXPORT __attribute__((visibility("default")))
#    define CircularOriController_DLLLOCAL __attribute__((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#    define CircularOriController_DLLIMPORT
#    define CircularOriController_DLLEXPORT
#    define CircularOriController_DLLLOCAL
#  endif // __GNUC__ >= 4
#endif // defined _WIN32 || defined __CYGWIN__

#ifdef CircularOriController_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define CircularOriController_DLLAPI
#  define CircularOriController_LOCAL
#else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef CircularOriController_EXPORTS
#    define CircularOriController_DLLAPI CircularOriController_DLLEXPORT
#  else
#    define CircularOriController_DLLAPI CircularOriController_DLLIMPORT
#  endif // CircularOriController_EXPORTS
#  define CircularOriController_LOCAL CircularOriController_DLLLOCAL
#endif // CircularOriController_STATIC
