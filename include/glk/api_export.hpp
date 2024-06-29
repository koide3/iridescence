#pragma once

#ifndef _WIN32
  #define GLK_API
#else
  #ifdef GLK_EXPORTS
    #define GLK_API __declspec(dllexport)
  #else
    #define GLK_API __declspec(dllimport)
  #endif
#endif