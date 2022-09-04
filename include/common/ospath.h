#pragma once
#ifndef OSPATH_HP
#define OSPATH_HP
#include <boost/filesystem.hpp>
#include <string>
#include <type_traits>
class os {
 public:
  class path {
    // https://stackoverflow.com/a/71223840/16633625
   public:
    static std::string join() { return ""; }

    template <typename T, typename... Types>
    static std::string join(T firstArg, Types... args) {
      static_assert(std::is_same<T, std::string>::value || std::is_same<T, const char*>::value, "T must inherit from string or const char*");
      boost::filesystem::path boostpath1(firstArg);
      boost::filesystem::path boostpath2(join(args...));
      boost::filesystem::path joinpath = boostpath1 / boostpath2;
      return joinpath.string();
    }
  };
};
#endif