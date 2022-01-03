#ifndef MYNAV__VISIBILITY_CONTROL_H_
#define MYNAV__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define MYNAV_EXPORT __attribute__ ((dllexport))
    #define MYNAV_IMPORT __attribute__ ((dllimport))
  #else
    #define MYNAV_EXPORT __declspec(dllexport)
    #define MYNAV_IMPORT __declspec(dllimport)
  #endif
  #ifdef MYNAV_BUILDING_DLL
    #define MYNAV_PUBLIC MYNAV_EXPORT
  #else
    #define MYNAV_PUBLIC MYNAV_IMPORT
  #endif
  #define MYNAV_PUBLIC_TYPE MYNAV_PUBLIC
  #define MYNAV_LOCAL
#else
  #define MYNAV_EXPORT __attribute__ ((visibility("default")))
  #define MYNAV_IMPORT
  #if __GNUC__ >= 4
    #define MYNAV_PUBLIC __attribute__ ((visibility("default")))
    #define MYNAV_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define MYNAV_PUBLIC
    #define MYNAV_LOCAL
  #endif
  #define MYNAV_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // MYNAV__VISIBILITY_CONTROL_H_